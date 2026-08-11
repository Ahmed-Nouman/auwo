#!/usr/bin/env python3
"""
map_accumulator.py — build a workspace map by accumulating lidar scans.

This is mapping-with-known-poses, NOT SLAM. The cab's slew angle comes from
the joint encoder via TF, so every scan is placed by known geometry -- no scan
matching, no IMU, no drift.

  /lidar_*/points  --TF--> fixed frame --> voxel grid --> /map_cloud

Params:
  input_topics (string array)  default ['/lidar_cab/points']
  fixed_frame  (string)        default 'base_link'
  voxel_size   (double)        default 0.10  (metres; 0 disables downsampling)
  publish_hz   (double)        default 2.0
  max_points   (int)           safety cap, default 4_000_000
  save_path    (string)        default '' -- set to write a .pcd on shutdown

Save at any time from another terminal:
  ros2 param set /map_accumulator save_now true

Example:
  ros2 run <pkg> map_accumulator.py --ros-args \
    -p input_topics:="['/lidar_cab/points']" \
    -p fixed_frame:=base_link -p voxel_size:=0.10 \
    -p save_path:=/home/nameless/Desktop/AUWO-testbed/maps/cfg1_cab_tilt30.pcd \
    -p use_sim_time:=true
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2


def quat_to_R(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def voxel_downsample(pts, voxel):
    """Keep one point per occupied voxel (first-hit). Pure numpy."""
    if voxel <= 0.0 or len(pts) == 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    # pack 3 int64 into a single view for uniqueness
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


class MapAccumulator(Node):
    def __init__(self):
        super().__init__("map_accumulator")

        self.declare_parameter("input_topics", ["/lidar_cab/points"])
        self.declare_parameter("fixed_frame", "base_link")
        self.declare_parameter("voxel_size", 0.10)
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("max_points", 4000000)
        self.declare_parameter("save_path", "")
        self.declare_parameter("save_now", False)

        self.frame = self.get_parameter("fixed_frame").value
        self.voxel = float(self.get_parameter("voxel_size").value)
        self.max_points = int(self.get_parameter("max_points").value)
        topics = list(self.get_parameter("input_topics").value)

        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

        self.acc = np.empty((0, 3), dtype=np.float32)
        self.n_scans = 0

        self.pub = self.create_publisher(PointCloud2, "/map_cloud", 1)
        for t in topics:
            self.create_subscription(PointCloud2, t, self.cb, 5)

        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / max(hz, 0.1), self.tick)

        self.get_logger().info(
            "Accumulating %s in '%s', voxel=%.3f m" % (topics, self.frame, self.voxel)
        )

    def cb(self, msg):
        try:
            tf = self.buf.lookup_transform(
                self.frame, msg.header.frame_id, Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as e:
            self.get_logger().warn("TF %s -> %s: %s" % (msg.header.frame_id,
                                                        self.frame, e),
                                   throttle_duration_sec=5.0)
            return

        pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"),
                                    skip_nans=True)
        if pts.size == 0:
            return

        tr, ro = tf.transform.translation, tf.transform.rotation
        R = quat_to_R(ro.x, ro.y, ro.z, ro.w)
        world = (pts @ R.T + np.array([tr.x, tr.y, tr.z])).astype(np.float32)

        self.acc = np.vstack([self.acc, world])
        self.n_scans += 1

        if len(self.acc) > self.max_points:
            self.acc = voxel_downsample(self.acc, max(self.voxel, 0.05))

        # self.acc = np.vstack([self.acc, world])
        # self.n_scans += 1

        # --- publish here instead of relying on the timer ---
        if self.n_scans % 5 == 0:
            self.acc = voxel_downsample(self.acc, self.voxel)
            self.publish(self.acc)
            self.get_logger().info("scans=%d points=%d" % (self.n_scans, len(self.acc)),
                                   throttle_duration_sec=5.0)

    def tick(self):
        if self.get_parameter("save_now").value:
            self.save()
            self.set_parameters(
                [rclpy.parameter.Parameter("save_now",
                                           rclpy.Parameter.Type.BOOL, False)])

        if len(self.acc) == 0:
            return

        self.acc = voxel_downsample(self.acc, self.voxel)
        self.publish(self.acc)
        self.get_logger().info("scans=%d  points=%d" % (self.n_scans, len(self.acc)),
                               throttle_duration_sec=5.0)

    def publish(self, pts):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.height = 1
        msg.width = pts.shape[0]
        msg.is_dense = True
        msg.is_bigendian = False
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.data = pts.astype(np.float32).tobytes()
        self.pub.publish(msg)

    def save(self):
        path = self.get_parameter("save_path").value
        if not path:
            self.get_logger().warn("save_now set but save_path is empty")
            return
        os.makedirs(os.path.dirname(path), exist_ok=True)
        pts = voxel_downsample(self.acc, self.voxel)
        with open(path, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write("WIDTH %d\nHEIGHT 1\n" % len(pts))
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write("POINTS %d\nDATA ascii\n" % len(pts))
            for p in pts:
                f.write("%.4f %.4f %.4f\n" % (p[0], p[1], p[2]))
        self.get_logger().info("saved %d points -> %s" % (len(pts), path))


def main():
    rclpy.init()
    node = MapAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.get_parameter("save_path").value:
            node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
