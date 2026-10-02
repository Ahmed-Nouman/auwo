#!/usr/bin/env python3
"""Closes the kinematic loops of the v2 excavator (hydraulic cylinders + bucket linkage).

URDF cannot describe closed loops, so the cylinders and the bucket four-bar are extra
joints whose positions follow from the actuated joints. This node computes them.

Parameter "mode":
  controller (default)  ros2_control is running (Gazebo or mock hardware)
      sub  /joint_states                     actuated joints from joint_state_broadcaster
      pub  /linkage_controller/commands      std_msgs/Float64MultiArray (10 joints)
  mirror                joint states come from outside ros2_control (physical twin)
      sub  /joint_states                     actuated joints (e.g. physical_tf_follower)
      pub  /joint_states                     linkage joints (+ blade_rotation if missing)
  display               RViz-only viewing with sliders / demo pose
      sub  joint_commands                    actuated joints
      pub  joint_states                      all movable joints at 30 Hz
All modes publish
      pub  cylinder_lengths                  sensor_msgs/JointState, pin-to-pin length [m]

Parameter "excavator_model" selects the model; models without a linkage (v1) make the
node log a message and exit, so launch files can start it unconditionally.
"""
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from excavator_models import linkage_kinematics as K
from excavator_models.registry import default_model, load_profile

ARM = ['boom_rotation', 'stick_rotation', 'bucket_rotation']


class LinkageStatePublisher(Node):

    def __init__(self):
        super().__init__('linkage_state_publisher')
        model = self.declare_parameter('excavator_model', default_model()).value
        self.mode = self.declare_parameter('mode', 'controller').value
        self.profile = load_profile(model)
        link = self.profile['linkage']
        self.enabled = bool(link.get('enabled'))
        if not self.enabled:
            self.get_logger().info(
                f'Model "{self.profile["model"]}" has no hydraulic linkage - nothing to do.')
            return

        self.q = {name: 0.0 for name in K.ACTUATED}
        self.passive, self.lengths, _ = K.solve()
        self.len_pub = self.create_publisher(JointState, 'cylinder_lengths', 10)
        self._last_pub = None
        max_rate = float(self.declare_parameter('max_rate', 50.0).value)
        self._min_period_ns = int(1e9 / max_rate) if max_rate > 0 else 0

        if self.mode == 'controller':
            topic = f'{link.get("controller", "linkage_controller")}/commands'
            self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
            self.cmd_pub = self.create_publisher(Float64MultiArray, topic, 10)
            self.get_logger().info(f'controller mode: /joint_states -> {topic}')
        elif self.mode == 'mirror':
            self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
            self.js_pub = self.create_publisher(JointState, '/joint_states', 10)
            self.get_logger().info('mirror mode: /joint_states -> /joint_states (linkage joints)')
        elif self.mode == 'display':
            self.create_subscription(JointState, 'joint_commands', self.on_joints, 10)
            self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
            self.create_timer(1.0 / 30.0, self.publish_display)
            self.get_logger().info('display mode: joint_commands -> joint_states')
        else:
            raise ValueError(f'unknown mode "{self.mode}" (controller, mirror or display)')

    # ------------------------------------------------------------------
    def on_joints(self, msg):
        # In mirror mode our own messages come back on /joint_states: they contain
        # no arm joints, so they are ignored here.
        if not any(n in ARM for n in msg.name):
            return
        new = dict(self.q)
        for name, pos in zip(msg.name, msg.position):
            if name in new:
                new[name] = K.clamp(name, pos)
        passive, lengths, ok = K.solve(new['boom_rotation'], new['stick_rotation'],
                                       new['bucket_rotation'], new['blade_rotation'])
        if not ok:
            self.get_logger().warn('pose outside linkage range, keeping last valid pose',
                                   throttle_duration_sec=2.0)
            return
        self.q, self.passive, self.lengths = new, passive, lengths
        blade_in_msg = 'blade_rotation' in msg.name

        if self.mode == 'display':
            return
        now = self.get_clock().now()
        if self._last_pub is not None and \
                (now - self._last_pub).nanoseconds < self._min_period_ns:
            return
        self._last_pub = now

        if self.mode == 'controller':
            self.cmd_pub.publish(Float64MultiArray(
                data=[passive[j] for j in K.LINKAGE_JOINTS]))
        else:  # mirror
            js = JointState()
            stamp = getattr(msg.header, 'stamp', None)
            # keep the incoming stamp when it is set, so the mirrored joints carry
            # the same time as the source (bag playback / physical twin)
            js.header.stamp = stamp if (getattr(stamp, 'sec', 0) or
                                        getattr(stamp, 'nanosec', 0)) else now.to_msg()
            js.name = list(K.LINKAGE_JOINTS)
            js.position = [passive[j] for j in K.LINKAGE_JOINTS]
            if not blade_in_msg:
                js.name.append('blade_rotation')
                js.position.append(self.q['blade_rotation'])
            self.js_pub.publish(js)
        self.publish_lengths(now.to_msg())

    def publish_lengths(self, stamp):
        cl = JointState()
        cl.header.stamp = stamp
        cl.name = [f'{k}_cylinder' for k in self.lengths]
        cl.position = list(self.lengths.values())
        self.len_pub.publish(cl)

    def publish_display(self):
        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = list(self.q) + list(K.LINKAGE_JOINTS)
        js.position = list(self.q.values()) + [self.passive[j] for j in K.LINKAGE_JOINTS]
        self.js_pub.publish(js)
        self.publish_lengths(stamp)


def main(args=None):
    rclpy.init(args=args)
    node = LinkageStatePublisher()
    if not node.enabled:
        node.destroy_node()
        rclpy.try_shutdown()
        return 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
