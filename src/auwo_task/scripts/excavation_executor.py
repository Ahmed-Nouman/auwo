#!/usr/bin/env python3
"""
excavation_executor.py — run dig-and-dump cycles at a target. Mode M2.

    /auwo/dig_target ─► waypoints ─► IK ─► interpolate ─► /auwo/cmd/auto
                                                               │
                                                          arbiter ─► machine

Start and stop:
    ros2 service call /auwo/start_dig std_srvs/srv/Trigger
    ros2 service call /auwo/stop_dig  std_srvs/srv/Trigger

ANALYTIC IK, NOT MoveIt
Boom, stick and bucket all rotate about parallel Y axes, with slew about Z, so
the arm is a 2-link planar chain (L1 = 2.661 m, L2 = 1.371 m) plus a wrist. That
inverts in closed form and round-trips to under a millimetre, which is far
simpler than launching a motion planner - and matches how ALICE frames it:
excavator kinematics do not permit independent 6-DoF control, so the feasible
poses form a sub-manifold and only position is solved for. The bucket angle is
a free choice, used here as the attack angle.

The boom pivot sits 0.134 m off the slew axis, so the arm plane does not pass
through the centre of rotation. The slew angle is corrected for that; getting
it wrong puts every target about a metre out.

THE CYCLE
  approach   above the target, bucket open
  penetrate  down to target depth
  drag       pull toward the machine at depth   <- this is where material loads
  curl       close the bucket to hold it
  lift       raise clear of the ground
  slew       turn to the dump bearing
  dump       open the bucket
  return     back over the dig area

ABORT IS SIMPLY CEASING TO PUBLISH
Stop requested, target lost, e-stop, or this node dying all have the same
effect: /auwo/cmd/auto goes quiet, the arbiter times out and freezes the
machine at its measured pose. One mechanism, already tested.

Params
  rate_hz        20.0
  joint_speed    0.35   rad/s, the slowest joint sets the segment duration
  approach_h     0.8    m above the target to start and finish
  drag_length    0.8    m pulled toward the machine while at depth
  lift_h         1.2    m above ground for transport
  dump_bearing   90.0   deg from the dig bearing
  dump_radius    5.0    m
  dump_height    1.5    m
  attack_deg     -35.0  bucket angle during penetration
  curl_deg       60.0   bucket angle while carrying
"""

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

LATCHED = QoSProfile(depth=1,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE)

# ---------------------------------------------------------------------------
# GEOMETRY IS READ FROM /robot_description, NOT HARD-CODED
#
# Hard-coding these cost a long debugging session. The URDF had been edited to
# centre the boom - removing a 0.134 m lateral offset - while this file still
# carried the old value, so every target landed 0.134 m to one side and the
# error rotated with the cab. Reading the live description makes that class of
# bug impossible: the model and TF cannot drift apart.
#
# The values below are a fallback used only if /robot_description never arrives.
# ---------------------------------------------------------------------------
P_BOOM = np.array([0.933, 0.0, 0.302])        # in body
P_STICK = np.array([2.586, 0.0, -0.625])      # in boom
P_BUCK = np.array([-1.362, 0.0, -0.154])      # in stick
TIP = np.array([0.59, 0.0, 0.0])              # bucket tip, from the mesh
BODY_Z = 0.6                                  # body above base_link
BASE_FLIP = True                              # base_to_base_link yaw = pi

L1 = math.hypot(P_STICK[0], P_STICK[2])
A1 = math.atan2(P_STICK[2], P_STICK[0])
OFF = P_BOOM[1]

# Usable ground-level reach with a dug-in bucket, measured by sweeping the IK
# against the joint limits. The analytic envelope is wider, but the boom and
# stick limits bite before it. Waypoints are clamped into this band rather than
# being allowed to fail.
REACH_IN, REACH_OUT = 2.55, 4.20

LIM_LO = np.array([-12.566, -1.400, -2.428, -2.000])
LIM_HI = np.array([12.566, 0.200, 0.200, 2.000])
JOINTS = ["body_rotation", "boom_rotation", "stick_rotation", "bucket_rotation"]


def load_geometry(urdf_xml):
    """Pull arm geometry and joint limits out of a URDF string."""
    global P_BOOM, P_STICK, P_BUCK, BODY_Z, BASE_FLIP, L1, A1, OFF
    global LIM_LO, LIM_HI
    import xml.etree.ElementTree as ET

    root = ET.fromstring(urdf_xml)
    joints = {j.get("name"): j for j in root.findall("joint")}
    for n in JOINTS:
        if n not in joints:
            raise KeyError("joint '%s' not in the description" % n)

    def xyz_of(name):
        o = joints[name].find("origin")
        return np.array([float(v) for v in (o.get("xyz", "0 0 0")).split()])

    BODY_Z = float(xyz_of("body_rotation")[2])
    P_BOOM = xyz_of("boom_rotation")
    P_STICK = xyz_of("stick_rotation")
    P_BUCK = xyz_of("bucket_rotation")

    BASE_FLIP = False
    for j in joints.values():
        if j.find("child").get("link") == "base":
            o = j.find("origin")
            rpy = ([float(v) for v in (o.get("rpy", "0 0 0")).split()]
                   if o is not None else [0.0, 0.0, 0.0])
            BASE_FLIP = abs(abs(rpy[2]) - math.pi) < 0.05

    L1 = math.hypot(P_STICK[0], P_STICK[2])
    A1 = math.atan2(P_STICK[2], P_STICK[0])
    OFF = P_BOOM[1]

    lo, hi = list(LIM_LO), list(LIM_HI)
    for i, n in enumerate(JOINTS):
        lim = joints[n].find("limit")
        if lim is not None and lim.get("lower") is not None:
            lo[i], hi[i] = float(lim.get("lower")), float(lim.get("upper"))
    LIM_LO, LIM_HI = np.array(lo), np.array(hi)

    return dict(body_z=BODY_Z, boom=P_BOOM, stick=P_STICK, buck=P_BUCK,
                base_flip=BASE_FLIP, lo=LIM_LO, hi=LIM_HI)


def wrap(a):
    """Into (-pi, pi]. The IK returns angles that can be a full turn out;
    unwrapped they fail the joint-limit check even though the pose is legal."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def nearest(a, ref):
    """The representation of angle `a` closest to `ref`.

    Slew has several turns of travel, so a target wrapped into (-pi, pi] can sit
    a whole revolution away from where the cab currently points. Without this
    the machine swings the long way round between cycles - which looks like it
    wandering off to a new place each time.
    """
    return ref + wrap(a - ref)


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# base_link -> base carries a 180 deg yaw (base_to_base_link rpy="0 0 3.14159"),
# so the arm chain below is expressed in `base`, not `base_link`. Targets arrive
# in base_link. Converting at the boundary is a sign flip on x and y; skipping
# it mirrors every target through the origin and the machine digs at the
# opposite bearing.
def bl_to_base(x, y):
    return (-x, -y) if BASE_FLIP else (x, y)


def fk(q):
    """Bucket tip in BASE_LINK."""
    b, bo, st, bu = q
    p = np.array([0.0, 0.0, BODY_Z]) + Rz(b) @ P_BOOM
    R = Rz(b) @ Ry(bo)
    p = p + R @ P_STICK
    R = R @ Ry(st)
    p = p + R @ P_BUCK
    R = R @ Ry(bu)
    p = p + R @ TIP
    if BASE_FLIP:
        return np.array([-p[0], -p[1], p[2]])  # base -> base_link
    return p


def _ik_branch(x, y, z, bucket, elbow):
    x, y = bl_to_base(x, y)                    # base_link -> base
    R = math.hypot(x, y)
    if R <= abs(OFF):
        return None
    b = math.atan2(y, x) - math.asin(OFF / R)
    r = math.sqrt(R * R - OFF * OFF) - P_BOOM[0]
    h = z - (BODY_Z + P_BOOM[2])
    e = P_BUCK + Ry(bucket) @ TIP
    L2 = math.hypot(e[0], e[2])
    A2 = math.atan2(e[2], e[0])
    d = math.hypot(r, h)
    if d > L1 + L2 or d < abs(L1 - L2):
        return None
    phi = math.atan2(h, r)
    al = math.acos(max(-1.0, min(1.0, (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d))))
    be = math.acos(max(-1.0, min(1.0, (L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2))))
    t1 = phi + elbow * al
    t2 = t1 - elbow * (math.pi - be)
    bo = wrap(A1 - t1)
    st = wrap(A2 - bo - t2)
    return np.array([wrap(b), bo, st, bucket])


def reach_or_lower(x, y, z, bucket, z_floor, step=0.15):
    """IK at (x, y, z), dropping the height until it is reachable.

    Excavators cannot lift high at short radius - the boom limit stops them -
    so a waypoint asking for both simply fails. Rather than abort the cycle,
    settle for the highest point the arm can actually reach.
    """
    zz = z
    while True:
        q = ik(x, y, zz, bucket)
        if q is not None:
            return q, zz
        zz -= step
        if zz < z_floor - 1e-9:
            return None, z


def ik(x, y, z, bucket=0.0):
    """Position-only IK. Returns joint angles inside limits, or None."""
    best = None
    for elbow in (-1, 1):
        q = _ik_branch(x, y, z, bucket, elbow)
        if q is None:
            continue
        if not (LIM_LO[1] <= q[1] <= LIM_HI[1]):
            continue
        if not (LIM_LO[2] <= q[2] <= LIM_HI[2]):
            continue
        err = float(np.linalg.norm(fk(q) - np.array([x, y, z])))
        if err > 0.05:
            continue
        if best is None or err < best[1]:
            best = (q, err)
    return None if best is None else best[0]


class Executor(Node):
    def __init__(self):
        super().__init__("excavation_executor")

        p = self.declare_parameter
        p("rate_hz", 20.0)
        p("joint_speed", 0.35)
        p("approach_h", 0.8)
        p("drag_length", 0.8)
        p("lift_h", 1.2)
        p("dump_bearing", 90.0)
        p("dump_radius", 3.8)
        p("dump_height", 1.5)
        p("attack_deg", -35.0)
        p("curl_deg", 60.0)
        p("cut_half_width", 0.35)   # half the bucket width, for terrain cutting

        g = self.get_parameter
        self.hz = float(g("rate_hz").value)
        self.dt = 1.0 / self.hz
        self.speed = float(g("joint_speed").value)

        self.target = None          # (x, y, z) in base_link
        self.measured = None
        self.running = False
        self.plan = []              # [(label, q)]
        self.cut = None             # swath cut by the current cycle
        self.leg = 0
        self.t_leg = 0.0
        self.q_from = None
        self.cycles = 0

        self.pub = self.create_publisher(Float64MultiArray, "/auwo/cmd/auto", 10)
        self.pub_state = self.create_publisher(String, "/auwo/dig_state", 10)
        # Announces the swath just cut, so the simulator can lower the terrain:
        #   [x1, y1, x2, y2, z_bottom, half_width]   all in base_link
        # Isaac clamps ground vertices inside that swath down to z_bottom. The
        # lidar then measures the new surface for real - the map, the progress
        # figure and the planner all respond to a genuine change rather than to
        # bookkeeping.
        self.pub_cut = self.create_publisher(
            Float64MultiArray, "/auwo/material_removed", 10)
        self.create_subscription(PoseStamped, "/auwo/dig_target",
                                 self._on_target, 1)
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.have_urdf = False
        self.create_subscription(String, "/robot_description",
                                 self._on_urdf, LATCHED)
        self.create_service(Trigger, "/auwo/start_dig", self._srv_start)
        self.create_service(Trigger, "/auwo/stop_dig", self._srv_stop)

        self.get_logger().info(
            "excavation executor ready -> /auwo/cmd/auto "
            "(arbiter must be in 'auto' mode)")
        self.get_logger().info(
            "   start: ros2 service call /auwo/start_dig std_srvs/srv/Trigger")

    # ------------------------------------------------------------- callbacks
    def _on_urdf(self, msg):
        if self.have_urdf:
            return
        try:
            g = load_geometry(msg.data)
        except Exception as e:
            self.get_logger().error(
                "could not read /robot_description (%s) - using fallback "
                "geometry, which may not match TF" % e)
            return
        self.have_urdf = True
        self.get_logger().info(
            "geometry from /robot_description: boom %s, stick %s, bucket %s, "
            "body_z %.3f, base flip %s"
            % (np.round(g["boom"], 3), np.round(g["stick"], 3),
               np.round(g["buck"], 3), g["body_z"], g["base_flip"]))
        self.get_logger().info(
            "   limits lo %s  hi %s"
            % (np.round(g["lo"], 3), np.round(g["hi"], 3)))

    def _on_target(self, msg):
        self.target = (msg.pose.position.x, msg.pose.position.y,
                       msg.pose.position.z)

    def _on_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.measured = np.array([pos[j] for j in JOINTS])

    def _srv_start(self, req, resp):
        if self.target is None:
            resp.success = False
            resp.message = "no dig target on /auwo/dig_target"
            return resp
        if self.measured is None:
            resp.success = False
            resp.message = "no /joint_states"
            return resp
        if not self.have_urdf:
            self.get_logger().warn(
                "no /robot_description yet - planning with fallback geometry")
        if not self._build_plan():
            resp.success = False
            resp.message = "target unreachable: %.2f, %.2f, %.2f" % self.target
            return resp
        self.running = True
        self.cycles = 0
        resp.success = True
        resp.message = "digging at %.2f, %.2f, %.2f" % self.target
        self.get_logger().info(resp.message)
        return resp

    def _srv_stop(self, req, resp):
        self._halt("stopped by request")
        resp.success = True
        resp.message = "stopped"
        return resp

    def _halt(self, why):
        if self.running:
            self.get_logger().info("halting: %s" % why)
        self.running = False
        self.plan = []

    # ------------------------------------------------------------------ plan
    def _build_plan(self):
        g = self.get_parameter
        x, y, z = self.target
        bearing = math.atan2(y, x)
        rad = math.hypot(x, y)

        app = float(g("approach_h").value)
        drag = float(g("drag_length").value)
        lift = float(g("lift_h").value)
        atk = math.radians(float(g("attack_deg").value))
        curl = math.radians(float(g("curl_deg").value))

        dump_b = bearing + math.radians(float(g("dump_bearing").value))
        dump_r = float(g("dump_radius").value)
        dump_z = float(g("dump_height").value)
        rad = min(max(rad, REACH_IN), REACH_OUT)
        x, y = rad * math.cos(bearing), rad * math.sin(bearing)

        # Pulling toward the machine is what fills the bucket. Clamp the end
        # of the drag into the reachable band, and shorten the drag rather
        # than fail if the target sits near the inner limit.
        r_end = max(rad - drag, REACH_IN)
        dump_r = min(max(dump_r, REACH_IN), REACH_OUT)

        # Lift while swinging outward, as a real machine does: high and close
        # is the one thing this arm cannot do.
        r_lift = min(max(0.5 * (r_end + dump_r), REACH_IN), REACH_OUT)
        b_lift = bearing + 0.5 * (dump_b - bearing)

        legs = [
            ("approach",  (x, y, z + app), atk, z + 0.1),
            ("penetrate", (x, y, z), atk, z),
            ("drag",      (r_end * math.cos(bearing),
                           r_end * math.sin(bearing), z), atk, z),
            ("curl",      (r_end * math.cos(bearing),
                           r_end * math.sin(bearing), z + 0.15), curl, z),
            ("lift",      (r_lift * math.cos(b_lift),
                           r_lift * math.sin(b_lift), lift), curl, z + 0.3),
            ("slew",      (dump_r * math.cos(dump_b),
                           dump_r * math.sin(dump_b), dump_z), curl, z + 0.3),
            ("dump",      (dump_r * math.cos(dump_b),
                           dump_r * math.sin(dump_b), dump_z), atk, z + 0.3),
            ("return",    (x, y, z + app), atk, z + 0.1),
        ]

        plan = []
        slew_ref = float(self.measured[0]) if self.measured is not None else 0.0
        for label, (px, py, pz), bu, floor in legs:
            q, zz = reach_or_lower(px, py, pz, bu, floor)
            if q is not None:
                q = np.array(q)
                q[0] = nearest(q[0], slew_ref)      # shortest way round
                q[0] = max(LIM_LO[0], min(LIM_HI[0], q[0]))
                slew_ref = float(q[0])
            if q is None:
                self.get_logger().warn(
                    "leg '%s' unreachable at (%.2f, %.2f, %.2f)"
                    % (label, px, py, pz))
                return False
            if abs(zz - pz) > 1e-6:
                self.get_logger().info(
                    "leg '%s' lowered %.2f -> %.2f m to stay inside the arm's reach"
                    % (label, pz, zz))
            plan.append((label, q))

        self.plan = plan
        self.leg = 0
        self.t_leg = 0.0
        self.q_from = np.array(self.measured)
        # remember the swath for this cycle, announced once the drag finishes
        self.cut = [x, y,
                    r_end * math.cos(bearing), r_end * math.sin(bearing),
                    z, float(g("cut_half_width").value)]
        return True

    # ------------------------------------------------------------------ loop
    def _leg_duration(self, a, b):
        return max(float(np.max(np.abs(b - a))) / max(self.speed, 1e-3), 0.4)

    def tick(self):
        s = String()
        s.data = ("%s %d/%d" % (self.plan[self.leg][0], self.leg + 1,
                                len(self.plan))) if self.running else "idle"
        self.pub_state.publish(s)

        if not self.running or not self.plan:
            return          # publish nothing: the arbiter freezes the machine

        label, q_to = self.plan[self.leg]
        dur = self._leg_duration(self.q_from, q_to)
        self.t_leg += self.dt
        a = min(self.t_leg / dur, 1.0)
        # cosine ease so the machine is not jerked at each waypoint
        s_a = 0.5 - 0.5 * math.cos(math.pi * a)
        q = self.q_from + (q_to - self.q_from) * s_a
        q = np.clip(q, LIM_LO, LIM_HI)

        m = Float64MultiArray()
        m.data = [float(v) for v in q]
        self.pub.publish(m)

        if a >= 1.0:
            # the drag is the leg that actually removes material
            if label == "drag" and self.cut is not None:
                m2 = Float64MultiArray()
                m2.data = [float(v) for v in self.cut]
                self.pub_cut.publish(m2)

            self.q_from = np.array(q_to)
            self.leg += 1
            self.t_leg = 0.0
            if self.leg >= len(self.plan):
                self.cycles += 1
                self.get_logger().info("cycle %d complete" % self.cycles)
                self.leg = 0
                if self.target is None or not self._build_plan():
                    self._halt("no further reachable target")
            else:
                self.get_logger().info("leg -> %s" % self.plan[self.leg][0])


def main():
    rclpy.init()
    node = Executor()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.tick()
            time.sleep(node.dt)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()