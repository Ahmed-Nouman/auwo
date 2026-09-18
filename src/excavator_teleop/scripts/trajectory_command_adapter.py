#!/usr/bin/env python3
"""
Converts position commands (Float64MultiArray) to smooth JointTrajectory so the
excavator moves smoothly instead of jerky step-wise. Subscribes to the same topic
used by the excavation cycle and RViz (arm_position_controller/commands), builds
a short trajectory from current state to target, and publishes to
arm_trajectory_controller/joint_trajectory.

Joint order: body_rotation, boom_rotation, stick_rotation, bucket_rotation.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from excavator_models import default_model, load_profile

JOINT_ORDER = ["body_rotation", "boom_rotation", "stick_rotation", "bucket_rotation"]
TRAJ_DURATION_S = 0.6  # time from current to target
NUM_POINTS = 15       # waypoints for smooth interpolation


class TrajectoryCommandAdapter(Node):
    def __init__(self):
        super().__init__("trajectory_command_adapter")
        model = self.declare_parameter("excavator_model", default_model()).value
        self.declare_parameter("trajectory_duration", TRAJ_DURATION_S)
        self.declare_parameter("num_points", NUM_POINTS)
        self.duration_s = self.get_parameter("trajectory_duration").value
        self.num_points = self.get_parameter("num_points").value

        # Start from the model's safe pose until /joint_states arrives (v1: [0, -0.5, -1, -1])
        self.current = [float(v) for v in load_profile(model)["safe_pose"]]
        self.last_target = None

        self.sub_cmd = self.create_subscription(
            Float64MultiArray,
            "/arm_position_controller/commands",
            self.on_command,
            10,
        )
        self.sub_joint = self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_states,
            10,
        )
        self.pub_traj = self.create_publisher(
            JointTrajectory,
            "/arm_trajectory_controller/joint_trajectory",
            10,
        )

        self.get_logger().info(
            "Trajectory adapter: position commands -> smooth trajectories (%.2f s, %d points)"
            % (self.duration_s, self.num_points)
        )

    def on_joint_states(self, msg: JointState):
        pos = list(self.current)
        for name, position in zip(msg.name, msg.position):
            if name in JOINT_ORDER:
                idx = JOINT_ORDER.index(name)
                pos[idx] = float(position)
        self.current = pos

    def on_command(self, msg: Float64MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warning("Expected 4 joints, got %d" % len(msg.data))
            return
        target = [float(x) for x in msg.data]
        self._publish_trajectory(self.current, target)

    def _publish_trajectory(self, start, goal):
        traj = JointTrajectory()
        traj.joint_names = JOINT_ORDER
        traj.header.stamp = self.get_clock().now().to_msg()
        n = max(2, self.num_points)
        for i in range(n):
            alpha = i / (n - 1) if n > 1 else 1.0
            t = alpha * self.duration_s
            # Smooth step (ease in-out) for less jerky motion
            s = alpha * alpha * (3.0 - 2.0 * alpha)
            point = JointTrajectoryPoint()
            point.positions = [
                start[j] + (goal[j] - start[j]) * s for j in range(4)
            ]
            point.time_from_start = Duration(
                sec=int(t), nanosec=int((t % 1) * 1e9)
            )
            traj.points.append(point)
        self.pub_traj.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryCommandAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
