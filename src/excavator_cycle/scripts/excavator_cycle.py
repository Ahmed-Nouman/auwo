#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from enum import Enum, auto

from excavator_models import default_model, load_profile

# Joint ranges for clamping (body is continuous so we won't clamp it).
# Replaced by the excavator model profile (ui_limits) in ExcavationCycleNode.__init__;
# the values below are the v1 ones.
BOOM_MIN, BOOM_MAX = -1.308, -0.087     # boom
STICK_MIN, STICK_MAX = -2.428, -0.085   # stick
BUCKET_MIN, BUCKET_MAX = -2.395, -0.357 # bucket


def _apply_limits(profile):
    global BOOM_MIN, BOOM_MAX, STICK_MIN, STICK_MAX, BUCKET_MIN, BUCKET_MAX
    lim = profile['ui_limits']
    BOOM_MIN, BOOM_MAX = (float(v) for v in lim['boom_rotation'])
    STICK_MIN, STICK_MAX = (float(v) for v in lim['stick_rotation'])
    BUCKET_MIN, BUCKET_MAX = (float(v) for v in lim['bucket_rotation'])


def clamp_arm_joints(q):
    # q = [body, boom, stick, bucket]
    body, boom, stick, bucket = q
    boom = max(BOOM_MIN, min(BOOM_MAX, boom))
    stick = max(STICK_MIN, min(STICK_MAX, stick))
    bucket = max(BUCKET_MIN, min(BUCKET_MAX, bucket))
    return [body, boom, stick, bucket]


class CycleState(Enum):
    PREPARE_DIG = auto()
    SCOOP_AND_SLEW = auto()
    LIFT_AND_ALIGN_DUMP = auto()
    DUMP = auto()
    RETURN_HOME = auto()


class ExcavationCycleNode(Node):
    def __init__(self):
        super().__init__('excavation_cycle_node')

        # Excavator model (v1/v2): cycle poses and joint ranges from its profile
        model = self.declare_parameter('excavator_model', default_model()).value
        profile = load_profile(model)
        _apply_limits(profile)
        self.poses = {k: [float(v) for v in q]
                      for k, q in profile.get('excavation_cycle', {}).items()}

        # Publisher to your joint group position controller
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/arm_position_controller/commands',
            10
        )

        # Control loop timer (10 Hz like teleop)
        self.timer = self.create_timer(0.1, self._tick)

        # High-level sequence of states
        self.sequence = [
            CycleState.PREPARE_DIG,
            CycleState.SCOOP_AND_SLEW,
            CycleState.LIFT_AND_ALIGN_DUMP,
            CycleState.DUMP,
            CycleState.RETURN_HOME,
        ]
        self.state_index = 0
        self.state_start_wall = time.monotonic()

        # How long we "hold" each state (seconds)
        self.state_hold_seconds = {
            CycleState.PREPARE_DIG:         3.0,
            CycleState.SCOOP_AND_SLEW:      3.0,
            CycleState.LIFT_AND_ALIGN_DUMP: 3.0,
            CycleState.DUMP:                2.5,
            CycleState.RETURN_HOME:         3.0,
        }

        # Current commanded joint vector
        self.q = [float(v) for v in profile['safe_pose']]
        self.target_q = self.q[:]

        # Max per-tick change (rad/tick) – larger so arm motion is visible
        self.max_step = 0.05

        # Wait for controller to be spawned before running state machine (spawn at 10s, 12s)
        self.warmup_done = False
        self.warmup_until_wall = time.monotonic() + 15.0

        self.get_logger().info(
            f"🟢 excavation_cycle_node (incremental stepping like teleop) started for model {profile['model']}")

    #
    # --- State machine helpers ---
    #
    def _advance_state_if_time(self):
        """Switch to next high-level pose after hold time (wall clock so cycle runs even without /clock)."""
        now = time.monotonic()
        current_state = self.sequence[self.state_index]
        if now - self.state_start_wall >= self.state_hold_seconds[current_state]:
            self.state_index = (self.state_index + 1) % len(self.sequence)
            self.state_start_wall = now
            self.get_logger().info(f"➡ state: {self.sequence[self.state_index].name}")

    def _desired_pose_for_state(self, state: CycleState):
        """Return the target joint pose [body, boom, stick, bucket] for the given state."""

        # Poses come from the model profile (excavation_cycle section). v1 poses:
        #   PREPARE_DIG         [0.0, -1.20, -2.20, -0.50]
        #   SCOOP_AND_SLEW      [0.5, -1.10, -1.80, -2.20]
        #   LIFT_AND_ALIGN_DUMP [1.0, -0.40, -0.90, -2.20]
        #   DUMP                [1.0, -0.40, -0.90, -0.50]
        #   RETURN_HOME         [0.0, -0.60, -1.20, -1.20]
        if state.name in self.poses:
            return list(self.poses[state.name])

        # fallback
        return self.q[:]

    def _update_target_from_state(self):
        """Refresh target_q based on current high-level state."""
        current_state = self.sequence[self.state_index]
        desired = self._desired_pose_for_state(current_state)
        desired = clamp_arm_joints(desired)
        self.target_q = desired

    def _step_toward_target(self):
        """Move q toward target_q by at most self.max_step per joint per tick."""
        new_q = list(self.q)
        for i in range(4):
            diff = self.target_q[i] - self.q[i]
            if abs(diff) <= self.max_step:
                new_q[i] = self.target_q[i]
            else:
                new_q[i] += self.max_step * (1.0 if diff > 0.0 else -1.0)
        new_q = clamp_arm_joints(new_q)
        self.q = new_q

    def _publish(self):
        msg = Float64MultiArray()
        msg.data = self.q
        self.pub.publish(msg)

    def _tick(self):
        """Timer callback: always publish so arm moves when sim runs; use wall clock for state machine."""
        now = time.monotonic()
        if now < self.warmup_until_wall:
            self._publish()
            return
        if not self.warmup_done:
            self.warmup_done = True
            self.get_logger().info("➡ excavation cycle state machine started (arm + body)")
        self._advance_state_if_time()
        self._update_target_from_state()
        self._step_toward_target()
        self._publish()


#
# --- Main entry point ---
#
def main(args=None):
    rclpy.init(args=args)
    node = ExcavationCycleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🟥 excavation_cycle_node interrupted — shutting down cleanly")
    finally:
        node.destroy_node()
        # Only call shutdown if the context is still active
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

