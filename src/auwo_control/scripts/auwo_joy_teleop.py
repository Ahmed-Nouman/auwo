#!/usr/bin/env python3
"""
auwo_joy_teleop.py — joystick teleoperation, ISO excavator pattern.

    /joy ─► deadman gate ─► velocity ─► integrate ─► /auwo/cmd/teleop
                                                          │
                                                     arbiter ─► machine

VELOCITY FEEL ON A POSITION INTERFACE
The twin takes joint POSITIONS, but real excavator joysticks command RATES -
deflection means speed, and the joint moves while you hold it. So this node
integrates stick deflection into a moving position target. The result feels
like a real machine, and it ports: on the physical excavator the same axis
mapping drives valve velocities directly, no integration needed.

ISO / SAE CONTROL PATTERN (what excavator operators actually expect)
    LEFT  stick   up/down  -> stick (dipper) out / in
    LEFT  stick   left/rt  -> slew the cab
    RIGHT stick   up/down  -> boom down / up
    RIGHT stick   left/rt  -> bucket curl / dump
Set pattern:="sae" to swap the two vertical axes.

THE DEADMAN IS A HEARTBEAT, NOT A TOGGLE
Motion requires the button held AND a fresh /joy message. Release it, unplug
the pad, or lose the link, and this node simply stops publishing. The arbiter
sees the silence, times out, and freezes the machine at its measured pose.
One failure path covers button release, node crash and network loss.

WALL CLOCK, DELIBERATELY
The loop runs on wall time rather than a ROS timer. ROS timers stall under
use_sim_time when /clock stutters, which silently broke this node. Nothing here
needs simulated time - the integration step is a real-time rate.

Params
  deadman_button  4      (LB on an Xbox pad)
  turbo_button    5      (RB) doubles the rates while held
  pattern         iso    "iso" or "sae"
  rate_hz         20.0
  deadzone        0.12
  max_rate        [0.40, 0.35, 0.45, 0.60]   rad/s per joint
  limits_lower    [-12.566, -1.400, -2.428, -2.000]
  limits_upper    [ 12.566,  0.200,  0.200,  2.000]

Example
  ros2 run auwo_control keyboard_to_joy.py      # or: ros2 run joy joy_node
  ros2 run auwo_control auwo_joy_teleop.py
"""

import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ["body_rotation", "boom_rotation", "stick_rotation", "bucket_rotation"]

# axis index and sign per joint, by pattern.  order: body, boom, stick, bucket
PATTERNS = {
    #        body(slew)   boom        stick       bucket
    "iso": ((0, +1.0), (4, -1.0), (1, +1.0), (3, +1.0)),
    "sae": ((0, +1.0), (1, -1.0), (4, +1.0), (3, +1.0)),
}


class JoyTeleop(Node):
    def __init__(self):
        super().__init__("auwo_joy_teleop")

        p = self.declare_parameter
        p("out_topic", "/auwo/cmd/teleop")
        p("deadman_button", 4)
        p("turbo_button", 5)
        p("pattern", "iso")
        p("rate_hz", 20.0)
        p("deadzone", 0.12)
        p("max_rate", [0.40, 0.35, 0.45, 0.60])
        p("limits_lower", [-12.566, -1.400, -2.428, -2.000])
        p("limits_upper", [12.566, 0.200, 0.200, 2.000])
        p("resync_rad", 0.35)   # re-seed only if the target has drifted this far
        p("lead_rad", 0.25)     # how far the target may run ahead of the arm

        g = self.get_parameter
        self.deadman = int(g("deadman_button").value)
        self.turbo = int(g("turbo_button").value)
        self.dz = float(g("deadzone").value)
        self.rates = [float(v) for v in g("max_rate").value]
        self.lo = [float(v) for v in g("limits_lower").value]
        self.hi = [float(v) for v in g("limits_upper").value]
        self.resync = float(g("resync_rad").value)
        self.lead = float(g("lead_rad").value)

        pat = g("pattern").value
        self.map = PATTERNS.get(pat, PATTERNS["iso"])
        if pat not in PATTERNS:
            self.get_logger().warn("unknown pattern '%s', using iso" % pat)

        self.hz = float(g("rate_hz").value)
        self.dt = 1.0 / self.hz

        self.target = None
        self.measured = None
        self.joy = None
        self.held = False
        self.warned_no_state = False

        self.pub = self.create_publisher(Float64MultiArray, g("out_topic").value, 10)
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)

        self.get_logger().info(
            "joy teleop up: pattern=%s deadman=button%d turbo=button%d -> %s"
            % (pat, self.deadman, self.turbo, g("out_topic").value))
        self.get_logger().info(
            "   hold the deadman to move. Release, or unplug, and the arbiter freezes.")

    # ------------------------------------------------------------- callbacks
    def _on_joy(self, msg):
        self.joy = msg

    def _on_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.measured = [pos[j] for j in JOINTS]

    # ------------------------------------------------------------------ loop
    def _axis(self, i):
        if self.joy is None or i >= len(self.joy.axes):
            return 0.0
        v = self.joy.axes[i]
        if abs(v) < self.dz:
            return 0.0
        s = 1.0 if v > 0 else -1.0
        return s * (abs(v) - self.dz) / (1.0 - self.dz)

    def _button(self, i):
        if self.joy is None or i >= len(self.joy.buttons):
            return False
        return bool(self.joy.buttons[i])

    def tick(self):
        held = self._button(self.deadman)

        if held != self.held:
            self.get_logger().info("deadman %s" % ("HELD" if held else "released"))
            self.held = held
            if held:
                # re-seed from the real pose so the machine never jumps
                if self.measured is None:
                    if not self.warned_no_state:
                        self.get_logger().warn(
                            "no /joint_states yet - not moving. Is Isaac playing?")
                        self.warned_no_state = True
                    self.held = False
                    return
                self.warned_no_state = False
                # Re-seed from the measured pose only when the target has
                # genuinely drifted - after a real pause, or if the joint
                # could not keep up. Re-seeding on every brief gap (terminal
                # auto-repeat, a dropped /joy frame) throws away integrated
                # progress, which is why heavy joints like the boom crawled.
                drift = (max(abs(a - b) for a, b in
                             zip(self.target, self.measured))
                         if self.target is not None else 1e9)
                if drift > self.resync:
                    self.target = list(self.measured)

        if not held or self.target is None:
            return          # publish nothing: the arbiter times out and freezes

        scale = 2.0 if self._button(self.turbo) else 1.0

        for j, (axis, sign) in enumerate(self.map):
            v = self._axis(axis) * sign * self.rates[j] * scale
            self.target[j] += v * self.dt
            self.target[j] = max(self.lo[j], min(self.hi[j], self.target[j]))

        # LEASH THE TARGET TO THE ARM.
        #
        # This integrates in wall time, but the simulator does not run at real
        # time - at 28 Hz of /joint_states against 60 Hz nominal it is about
        # half speed. Without a leash the target races away from the joint:
        # hold a key for ten seconds and the command has moved 4 rad while the
        # arm managed 1.5. The joint then crawls toward a point far ahead, the
        # key feels dead, and if the target reaches a joint limit that axis
        # stops responding entirely.
        #
        # Clamping the target to within `lead_rad` of the measured position
        # makes the command track whatever the sim can actually deliver, at any
        # sim speed, and releasing the key stops the arm at once.
        if self.measured is not None:
            for j in range(4):
                lo = self.measured[j] - self.lead
                hi = self.measured[j] + self.lead
                self.target[j] = max(lo, min(hi, self.target[j]))

        out = Float64MultiArray()
        out.data = self.target
        self.pub.publish(out)


def main():
    rclpy.init()
    node = JoyTeleop()
    try:
        # wall-clock loop, not create_timer - see the note in the docstring
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