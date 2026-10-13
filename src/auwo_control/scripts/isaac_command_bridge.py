#!/usr/bin/env python3
"""
Bridge RViz/cycle joint commands to Isaac Sim.

  /arm_position_controller/commands  (std_msgs/Float64MultiArray)
        -> /joint_command            (sensor_msgs/JointState)
              -> Isaac ROS2 SubscribeJointState -> ArticulationController

Array order matches joint_imarkers.py:
    [body_rotation, boom_rotation, stick_rotation, bucket_rotation]  (radians)

Isaac side: SubscriberJointState node topicName must be /joint_command,
not /joint_states (otherwise it echoes the robot's own published state).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    "body_rotation",
    "boom_rotation",
    "stick_rotation",
    "bucket_rotation",
]


class IsaacCommandBridge(Node):
    def __init__(self):
        super().__init__("isaac_command_bridge")

        self.declare_parameter("in_topic", "/arm_position_controller/commands")
        self.declare_parameter("out_topic", "/joint_command")

        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value

        self.pub = self.create_publisher(JointState, out_topic, 10)
        self.sub = self.create_subscription(
            Float64MultiArray, in_topic, self.cb, 10
        )
        self.get_logger().info(
            "Bridging %s (Float64MultiArray) -> %s (JointState)"
            % (in_topic, out_topic)
        )

    def cb(self, msg):
        n = min(len(msg.data), len(JOINT_NAMES))
        if n == 0:
            return
        js = JointState()
        #js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES[:n]
        js.position = [float(v) for v in msg.data[:n]]
        self.pub.publish(js)


def main():
    rclpy.init()
    node = IsaacCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
