#!/usr/bin/env python3
"""Merge two RGBD point clouds into one cloud in the excavator body frame.

Subscribes:  /points/camera_left, /points/camera_right (sensor_msgs/PointCloud2, xyzrgb)
Publishes:   /points/merged (PointCloud2, frame_id = target_frame)

The camera->body transforms are static (cameras are rigid to the body), so a
single TF lookup at startup is enough; we retry until TF is available.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from message_filters import Subscriber, ApproximateTimeSynchronizer
import tf2_ros


def cloud_to_xyzrgb(msg: PointCloud2, stride: int):
    """Return (N,3) float32 xyz and (N,) float32 rgb, NaNs removed."""
    offs = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({'names': ['x', 'y', 'z', 'rgb'],
                   'formats': ['<f4'] * 4,
                   'offsets': [offs['x'], offs['y'], offs['z'], offs['rgb']],
                   'itemsize': msg.point_step})
    pts = np.frombuffer(msg.data, dtype=dt)
    if stride > 1:
        pts = pts[::stride]
    xyz = np.stack([pts['x'], pts['y'], pts['z']], axis=-1)
    ok = np.isfinite(xyz).all(axis=1)
    return xyz[ok], pts['rgb'][ok]


class CloudMerger(Node):
    def __init__(self):
        super().__init__('cloud_merger')
        self.declare_parameter('target_frame', 'body')
        self.declare_parameter('stride', 4)         # keep every Nth point
        self.declare_parameter('max_range', 20.0)   # D455 envelope, meters
        self.target = self.get_parameter('target_frame').value
        self.stride = int(self.get_parameter('stride').value)
        self.max_range = float(self.get_parameter('max_range').value)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.T = {}  # frame_id -> (R 3x3, t 3)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        subs = [Subscriber(self, PointCloud2, '/points/camera_left', qos_profile=qos),
                Subscriber(self, PointCloud2, '/points/camera_right', qos_profile=qos)]
        self.sync = ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.10)
        self.sync.registerCallback(self.cb)
        self.pub = self.create_publisher(PointCloud2, '/points/merged', 5)
        self.get_logger().info(f'merging into frame "{self.target}"')

    def lookup(self, frame):
        if frame in self.T:
            return self.T[frame]
        try:
            tr = self.tf_buf.lookup_transform(self.target, frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF {self.target}<-{frame} not ready: {e}',
                                   throttle_duration_sec=2.0)
            return None
        q, t = tr.transform.rotation, tr.transform.translation
        w, x, y, z = q.w, q.x, q.y, q.z
        R = np.array([[1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
                      [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
                      [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)]], dtype=np.float32)
        self.T[frame] = (R, np.array([t.x, t.y, t.z], dtype=np.float32))
        return self.T[frame]

    def cb(self, left: PointCloud2, right: PointCloud2):
        chunks = []
        for msg in (left, right):
            tf = self.lookup(msg.header.frame_id)
            if tf is None:
                return
            R, t = tf
            xyz, rgb = cloud_to_xyzrgb(msg, self.stride)
            r = np.linalg.norm(xyz, axis=1)
            keep = r < self.max_range
            xyz, rgb = xyz[keep], rgb[keep]
            chunks.append((xyz @ R.T + t, rgb))

        xyz = np.concatenate([c[0] for c in chunks]).astype(np.float32)
        rgb = np.concatenate([c[1] for c in chunks]).astype(np.float32)
        n = xyz.shape[0]

        out = PointCloud2()
        out.header.stamp = left.header.stamp
        out.header.frame_id = self.target
        out.height, out.width = 1, n
        out.fields = [f for f in left.fields if f.name in ('x', 'y', 'z', 'rgb')]
        # repack tightly: x y z rgb = 16 bytes
        for f, off in zip(out.fields, (0, 4, 8, 12)):
            f.offset = off
        out.point_step, out.row_step = 16, 16 * n
        out.is_dense = True
        buf = np.empty((n, 4), dtype=np.float32)
        buf[:, :3], buf[:, 3] = xyz, rgb
        out.data = buf.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(CloudMerger())


if __name__ == '__main__':
    main()
