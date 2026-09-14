#!/usr/bin/env python3
"""
check_kinematics.py — is the IK consistent with TF, and is Isaac consistent
with the URDF?

Compares three things that should all agree and, when they do not, says which
pair disagrees:

  1. fk(measured joints)   the executor's analytic model
  2. TF base_link -> bucket   robot_state_publisher, driven by the URDF
  3. what you see in Isaac    checked by eye against RViz

If (1) and (2) agree, the analytic model matches the URDF and the IK is sound.
Any remaining error then lies between the URDF and the USD - the two describing
different machines - which no amount of IK work will fix.

    ros2 run auwo_task check_kinematics.py

Drive the arm around with the keyboard while it runs. The error should stay
near zero in every pose.
"""

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_ros
from sensor_msgs.msg import JointState

P_BOOM = np.array([0.933, -0.134, 0.302])
P_STICK = np.array([2.586, 0.0, -0.625])
P_BUCK = np.array([-1.362, 0.0, -0.154])
BODY_Z = 0.6
JOINTS = ["body_rotation", "boom_rotation", "stick_rotation", "bucket_rotation"]


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def fk_bucket(q):
    """Bucket JOINT origin in base_link, by the executor's model."""
    b, bo, st, _ = q
    p = np.array([0.0, 0.0, BODY_Z]) + Rz(b) @ P_BOOM
    R = Rz(b) @ Ry(bo)
    p = p + R @ P_STICK
    R = R @ Ry(st)
    p = p + R @ P_BUCK
    return np.array([-p[0], -p[1], p[2]])      # base -> base_link


class Check(Node):
    def __init__(self):
        super().__init__("check_kinematics")
        self.q = None
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.t = 0.0
        self.worst = 0.0
        self.get_logger().info(
            "comparing analytic FK against TF base_link -> bucket. "
            "Drive the arm around; the error should stay near zero.")

    def _on_js(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.q = np.array([pos[j] for j in JOINTS])

    def tick(self):
        now = time.monotonic()
        if now - self.t < 1.0 or self.q is None:
            return
        self.t = now
        try:
            tf = self.buf.lookup_transform("base_link", "bucket", Time(),
                                           timeout=Duration(seconds=0.3))
        except Exception as e:
            self.get_logger().warn("TF base_link -> bucket: %s" % e)
            return
        t = tf.transform.translation
        tf_p = np.array([t.x, t.y, t.z])
        fk_p = fk_bucket(self.q)
        d = fk_p - tf_p
        err = float(np.linalg.norm(d))
        self.worst = max(self.worst, err)

        verdict = ("MATCH" if err < 0.01 else
                   "MISMATCH - the analytic model disagrees with the URDF")
        self.get_logger().info(
            "joints %s\n"
            "        analytic fk  (%7.3f, %7.3f, %7.3f)\n"
            "        tf  -> bucket (%7.3f, %7.3f, %7.3f)\n"
            "        delta         (%7.3f, %7.3f, %7.3f)  |err| %.4f m  worst %.4f  %s"
            % (np.round(np.degrees(self.q), 1), *fk_p, *tf_p, *d,
               err, self.worst, verdict))


def main():
    rclpy.init()
    node = Check()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.tick()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()