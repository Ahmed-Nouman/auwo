#!/usr/bin/env python3
"""
auwo_arbiter.py — single writer to the machine.

Every command source publishes to its own /auwo/cmd/* topic. The arbiter picks
one according to the control mode, checks it is fresh, checks the e-stop, and
forwards it to /arm_position_controller/commands. Nothing else may write there.

    /auwo/cmd/teleop  ─┐
    /auwo/cmd/auto    ─┼─► arbiter ─► /arm_position_controller/commands
    /auwo/cmd/markers ─┘      ▲
                              │
              /estop/stopped ─┘   mode: idle | teleop | auto | estop

WHY A SEPARATE NODE
Safety review is far easier when exactly one node writes the command topic and
that node does nothing else. It is also the natural home for the control mode
and, later, the dumper cooperation LOCK.

THE FREEZE, AND WHY IT MATTERS
This is a POSITION interface: a stale target keeps pulling the joints toward it.
So "stop" cannot mean "stop publishing" - the machine would carry on to the last
target. On e-stop, on mode change, and on source timeout the arbiter latches the
CURRENT MEASURED joint positions and publishes those, which actually halts the
machine.

WALL CLOCK, DELIBERATELY
This node runs its loop on wall time and measures staleness with
time.monotonic(), never the ROS clock. Two reasons:

  1. ROS timers stall under use_sim_time when /clock stutters. That has bitten
     several nodes in this project.
  2. More importantly, a safety timeout must not depend on the simulation
     clock. If Isaac pauses, the arbiter must still notice its source has gone
     silent and freeze. Sim time would sit waiting forever.

Run it with or without use_sim_time - it makes no difference here.

FAIL-SAFE DIRECTION
  source goes stale  -> freeze
  e-stop trips       -> freeze, and refuse to forward until re-armed
  mode is idle       -> freeze
  no /joint_states   -> forward nothing at all (no safe hold can be built)

Params
  timeout_s      0.30   a source older than this is treated as absent
  rate_hz        20.0
  start_mode     idle
  require_estop  false  if true, refuse to move unless an e-stop is publishing

Mode is settable at runtime:
  ros2 param set /auwo_arbiter mode teleop

Example
  ros2 run auwo_control auwo_arbiter.py --ros-args -p start_mode:=teleop
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import Bool, Float64MultiArray, String
from sensor_msgs.msg import JointState

JOINTS = ["body_rotation", "boom_rotation", "stick_rotation", "bucket_rotation"]
MODES = ("idle", "teleop", "auto", "estop")


class Arbiter(Node):
    def __init__(self):
        super().__init__("auwo_arbiter")

        p = self.declare_parameter
        p("out_topic", "/arm_position_controller/commands")
        p("teleop_topic", "/auwo/cmd/teleop")
        p("auto_topic", "/auwo/cmd/auto")
        p("markers_topic", "/auwo/cmd/markers")
        p("timeout_s", 0.30)
        p("rate_hz", 20.0)
        p("start_mode", "idle")
        p("require_estop", False)
        p("mode", "idle")

        g = self.get_parameter
        self.timeout = float(g("timeout_s").value)
        self.require_estop = bool(g("require_estop").value)
        self.hz = float(g("rate_hz").value)

        mode = g("start_mode").value
        self.mode = mode if mode in MODES else "idle"
        self.set_parameters([Parameter("mode", Parameter.Type.STRING, self.mode)])

        self.sources = {}
        for m, key in (("teleop", "teleop_topic"),
                       ("auto", "auto_topic"),
                       ("markers", "markers_topic")):
            topic = g(key).value
            self.sources[m] = {"topic": topic, "data": None, "t": 0.0}
            self.create_subscription(
                Float64MultiArray, topic,
                lambda msg, mm=m: self._on_cmd(mm, msg), 10)

        self.measured = None
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)

        self.estop = None
        self.create_subscription(Bool, "/estop/stopped", self._on_estop, 10)

        self.pub = self.create_publisher(
            Float64MultiArray, g("out_topic").value, 10)
        self.pub_mode = self.create_publisher(String, "/auwo/control_mode", 10)

        self.frozen_at = None
        self.last_reason = ""

        self.get_logger().info("arbiter up, mode=%s, out=%s"
                               % (self.mode, g("out_topic").value))
        for m, s in self.sources.items():
            self.get_logger().info("   source %-8s <- %s" % (m, s["topic"]))
        if not self.require_estop:
            self.get_logger().warn(
                "require_estop is false: motion is allowed with no e-stop present. "
                "Set it true before operating real hardware.")

    # ------------------------------------------------------------- callbacks
    def _on_cmd(self, mode, msg):
        s = self.sources[mode]
        s["data"] = [float(v) for v in msg.data]
        s["t"] = time.monotonic()

    def _on_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.measured = [pos[j] for j in JOINTS]

    def _on_estop(self, msg):
        was = self.estop
        self.estop = bool(msg.data)
        if self.estop and was is not True:
            self.get_logger().error("E-STOP TRIPPED - freezing")
            self._freeze()

    # ---------------------------------------------------------------- policy
    def _freeze(self):
        if self.measured is not None:
            self.frozen_at = list(self.measured)

    def _active(self):
        p = self.get_parameter("mode").value
        if p in MODES and p != self.mode:
            self.get_logger().info("mode %s -> %s" % (self.mode, p))
            self.mode = p
            self._freeze()

        if self.estop is True:
            return None, "e-stop tripped"
        if self.estop is None and self.require_estop:
            return None, "no e-stop publisher"
        if self.mode in ("idle", "estop"):
            return None, "mode is %s" % self.mode

        now = time.monotonic()
        cands = ["teleop", "markers"] if self.mode == "teleop" else ["auto"]
        fresh = [(self.sources[c]["t"], c) for c in cands
                 if self.sources[c]["data"] is not None
                 and now - self.sources[c]["t"] <= self.timeout]
        if not fresh:
            return None, "no fresh command in mode %s" % self.mode
        return max(fresh)[1], ""

    def tick(self):
        m = String()
        m.data = self.mode
        self.pub_mode.publish(m)

        src, reason = self._active()

        if src is None:
            if reason != self.last_reason:
                self.get_logger().info("holding: %s" % reason)
                self.last_reason = reason
                self._freeze()
            if self.frozen_at is None:
                return
            out = Float64MultiArray()
            out.data = self.frozen_at
            self.pub.publish(out)
            return

        if self.last_reason:
            self.get_logger().info("forwarding %s" % src)
            self.last_reason = ""

        self.frozen_at = None
        out = Float64MultiArray()
        out.data = self.sources[src]["data"]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Arbiter()
    period = 1.0 / max(node.hz, 1.0)
    try:
        # wall-clock loop, not create_timer - see the note in the docstring
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.tick()
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()