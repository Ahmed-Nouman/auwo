#!/usr/bin/env python3
"""
physical_tf_follower_node.py (v12.2 — full joints restored + correct cab-only rotation)

v12.2: per-model calibration offsets (body_offset, stick_offset, bucket_offset;
boom_offset already existed). Defaults keep the v1 behaviour unchanged; the
launch file passes the values from the excavator model profile.
"""

import math
import rclpy
from rclpy.node import Node

import tf2_ros
from tf2_ros import TransformException, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import JointState


# -------------------------
# helpers
# -------------------------

def pitch_from_quat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(
        2.0 * (w * y - z * x),
        math.sqrt((1 - 2 * (y * y + z * z))**2 + (2 * (w * z + x * y))**2)
    )


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def wrap_zero_at_minus180(a):
    # maps -pi → 0, +pi → 2pi style continuity
    return normalize_angle(a + math.pi)


def continuous_angle(new, prev):
    if prev is None:
        return new
    return min(
        [new, new + 2*math.pi, new - 2*math.pi],
        key=lambda x: abs(x - prev)
    )


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)

    q = Quaternion()
    q.w = cr*cp*cy + sr*sp*sy
    q.x = sr*cp*cy - cr*sp*sy
    q.y = cr*sp*cy + sr*cp*sy
    q.z = cr*cp*sy - sr*sp*cy
    return q


# -------------------------
# node
# -------------------------

class PhysicalTfFollowerNode(Node):

    def __init__(self):
        super().__init__('physical_tf_follower_node')

        # params
        self.declare_parameter('mirror_to_joint_states', True)
        self.declare_parameter('publish_rate', 30.0)

        self.declare_parameter('frame_world_offset', 'world_offset')
        self.declare_parameter('frame_body', 'kinematic_1005')
        self.declare_parameter('frame_boom', 'kinematic_1003')
        self.declare_parameter('frame_stick', 'kinematic_1002')
        self.declare_parameter('frame_bucket', 'kinematic_2001')

        self.declare_parameter('body_sign', 1.0)
        self.declare_parameter('boom_sign', 1.0)
        self.declare_parameter('stick_sign', 1.0)
        self.declare_parameter('bucket_sign', 1.0)

        self.declare_parameter('boom_offset', -2.30)
        self.declare_parameter('body_offset', 0.0)
        self.declare_parameter('stick_offset', 0.0)
        self.declare_parameter('bucket_offset', 0.0)
        self.declare_parameter('base_link_roll_correction', 0.0)

        self._mirror = self.get_parameter('mirror_to_joint_states').value
        rate = self.get_parameter('publish_rate').value

        self._world = self.get_parameter('frame_world_offset').value
        self._body = self.get_parameter('frame_body').value
        self._boom = self.get_parameter('frame_boom').value
        self._stick = self.get_parameter('frame_stick').value
        self._bucket = self.get_parameter('frame_bucket').value

        self._s_body = self.get_parameter('body_sign').value
        self._s_boom = self.get_parameter('boom_sign').value
        self._s_stick = self.get_parameter('stick_sign').value
        self._s_bucket = self.get_parameter('bucket_sign').value

        self._boom_offset = self.get_parameter('boom_offset').value
        self._body_offset = float(self.get_parameter('body_offset').value)
        self._stick_offset = float(self.get_parameter('stick_offset').value)
        self._bucket_offset = float(self.get_parameter('bucket_offset').value)
        self._roll_corr = self.get_parameter('base_link_roll_correction').value

        self._prev_body = None
        self._prev_boom = None
        self._prev_stick = None
        self._prev_bucket = None

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._static = StaticTransformBroadcaster(self)

        if self._mirror:
            self._js_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.create_timer(1.0 / rate, self.update)

        self._publish_static_base()

        self.get_logger().info(
            "v12.2 TF follower started (offsets body=%.3f boom=%.3f stick=%.3f bucket=%.3f)"
            % (self._body_offset, self._boom_offset, self._stick_offset, self._bucket_offset))

    # -------------------------
    # static base link
    # -------------------------
    def _publish_static_base(self):
        t = TransformStamped()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.rotation.w = 1.0
        self._static.sendTransform(t)

    # -------------------------
    # lookup TF
    # -------------------------
    def lookup(self, p, c):
        try:
            return self._tf_buffer.lookup_transform(p, c, rclpy.time.Time())
        except TransformException:
            return None

    # -------------------------
    # BODY (cab only yaw fix)
    # -------------------------
    def compute_body(self):
        body = self.lookup(self._world, self._body)
        stick = self.lookup(self._world, self._stick)

        if not body or not stick:
            return None

        bx, by = body.transform.translation.x, body.transform.translation.y
        sx, sy = stick.transform.translation.x, stick.transform.translation.y

        yaw = math.atan2(sy - by, sx - bx)

        # fix -180 → 0 alignment
        yaw = wrap_zero_at_minus180(yaw)

        yaw *= self._s_body
        if self._body_offset:
            yaw = normalize_angle(yaw + self._body_offset)

        self._prev_body = continuous_angle(yaw, self._prev_body)
        return self._prev_body

    # -------------------------
    def update(self):
        now = self.get_clock().now().to_msg()

        body_yaw = self.compute_body()

        boom = self.lookup(self._body, self._boom)
        boom_a = None
        if boom:
            boom_a = self._s_boom * (-pitch_from_quat(boom.transform.rotation) + self._boom_offset)
            self._prev_boom = boom_a

        stick = self.lookup(self._body, self._stick)
        stick_a = None
        if stick:
            stick_a = self._s_stick * pitch_from_quat(stick.transform.rotation)
            if self._stick_offset:
                stick_a = normalize_angle(stick_a + self._stick_offset)
            self._prev_stick = stick_a

        bucket = self.lookup(self._body, self._bucket)
        bucket_a = None
        if bucket:
            bucket_a = self._s_bucket * pitch_from_quat(bucket.transform.rotation)
            if self._bucket_offset:
                bucket_a = normalize_angle(bucket_a + self._bucket_offset)
            self._prev_bucket = bucket_a

        # -------------------------
        # publish joints
        # -------------------------
        if self._mirror:
            js = JointState()
            js.header.stamp = now

            if body_yaw is not None:
                js.name.append("body_rotation")
                js.position.append(body_yaw)

            if boom_a is not None:
                js.name.append("boom_rotation")
                js.position.append(boom_a)

            if stick_a is not None:
                js.name.append("stick_rotation")
                js.position.append(stick_a)

            if bucket_a is not None:
                js.name.append("bucket_rotation")
                js.position.append(bucket_a)

            self._js_pub.publish(js)

            # -------------------------
            # FULL LOGGING RESTORED
            # -------------------------
            msg = []
            if body_yaw is not None:
                msg.append(f"body={math.degrees(body_yaw):.1f}°")
            if boom_a is not None:
                msg.append(f"boom={math.degrees(boom_a):.1f}°")
            if stick_a is not None:
                msg.append(f"stick={math.degrees(stick_a):.1f}°")
            if bucket_a is not None:
                msg.append(f"bucket={math.degrees(bucket_a):.1f}°")

            self.get_logger().info("  ".join(msg), throttle_duration_sec=1.0)


# -------------------------
# main
# -------------------------

def main():
    rclpy.init()
    node = PhysicalTfFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
