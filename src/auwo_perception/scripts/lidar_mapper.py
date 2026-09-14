#!/usr/bin/env python3
"""
lidar_mapper.py — self-filter + accumulate + publish + save, in one node.

Replaces the separate self_filter.py and map_accumulator.py.

  /lidar_*/points --> range crop --> link boxes --> TF to fixed frame
                  --> voxel grid --> /map_cloud   (+ /points_filtered)

Mapping is with KNOWN POSES, not SLAM: the cab slew angle comes from the joint
encoder via TF, so every scan is placed by known geometry.

Saves the accumulated cloud to `save_path` automatically on Ctrl+C.

Params:
  input_topics   default ['/lidar_cab/points']   (list; several lidars merge)
  fixed_frame    default 'base_link'
  filtered_topic default '/points_filtered'      ('' disables republish)
  min_range      default 2.0    metres from sensor (kills cab/roof ring)
  inflate        default 0.15   metres added to each link box face
  voxel_size     default 0.10   metres (0 disables downsampling)
  publish_every  default 5      publish once per N scans
  max_points     default 4000000
  save_path      default ''     written on shutdown

Example:
  ros2 run <pkg> lidar_mapper.py --ros-args \
    -p input_topics:="['/lidar_cab/points']" \
    -p min_range:=2.2 -p voxel_size:=0.10 -p use_sim_time:=true \
    -p save_path:=$HOME/Desktop/AUWO-testbed/maps/cfg1_cab_tilt30.pcd
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

# Link bounding boxes in each link's own frame, from the URDF collision meshes.
# (xmin, xmax, ymin, ymax, zmin, zmax) in metres.
LINK_BOXES = {
    "base":   (-0.94, 1.50, -0.85, 0.85,  0.03, 0.62),
    "body":   (-0.07, 0.30, -0.37, 0.38,  0.34, 1.06),
    "boom":   (-0.04, 2.63, -0.09, 0.09, -0.67, 0.43),
    "stick":  (-1.53, -1.16, -0.08, 0.08, -0.30, -0.01),
    "bucket": (-0.17, 0.59, -0.29, 0.29, -0.05, 0.54),
}

XYZ_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]


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
    if voxel <= 0.0 or len(pts) == 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


class LidarMapper(Node):
    def __init__(self):
        super().__init__("lidar_mapper")

        self.declare_parameter("input_topics", ["/lidar_cab/points"])
        self.declare_parameter("fixed_frame", "base_link")
        self.declare_parameter("filtered_topic", "/points_filtered")
        self.declare_parameter("min_range", 2.0)
        self.declare_parameter("inflate", 0.15)
        self.declare_parameter("voxel_size", 0.10)
        self.declare_parameter("publish_every", 5)
        self.declare_parameter("max_points", 4000000)
        self.declare_parameter("save_path", "")

        self.frame = self.get_parameter("fixed_frame").value
        self.min_range = float(self.get_parameter("min_range").value)
        self.inflate = float(self.get_parameter("inflate").value)
        self.voxel = float(self.get_parameter("voxel_size").value)
        self.pub_every = int(self.get_parameter("publish_every").value)
        self.max_points = int(self.get_parameter("max_points").value)
        topics = list(self.get_parameter("input_topics").value)

        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

        self.acc = np.empty((0, 3), dtype=np.float32)
        self.n_scans = 0
        self.n_in = 0
        self.n_kept = 0

        self.map_pub = self.create_publisher(PointCloud2, "/map_cloud", 1)

        ft = self.get_parameter("filtered_topic").value
        self.filt_pub = (self.create_publisher(PointCloud2, ft, 5) if ft else None)

        for t in topics:
            self.create_subscription(PointCloud2, t, self.cb, 5)

        self.get_logger().info(
            "lidar_mapper: %s -> /map_cloud in '%s'" % (topics, self.frame))
        self.get_logger().info(
            "  min_range=%.2f  inflate=%.2f  voxel=%.2f"
            % (self.min_range, self.inflate, self.voxel))

    # ------------------------------------------------------------------ filter
    def filter_scan(self, pts, sensor_frame, header):
        """pts are in the sensor frame. Returns points still in sensor frame."""
        if self.min_range > 0.0:
            r = np.linalg.norm(pts, axis=1)
            pts = pts[r >= self.min_range]
            if len(pts) == 0:
                return pts

        for link, box in LINK_BOXES.items():
            try:
                tf = self.buf.lookup_transform(
                    link, sensor_frame, Time(), timeout=Duration(seconds=0.1))
            except Exception:
                continue
            tr, ro = tf.transform.translation, tf.transform.rotation
            R = quat_to_R(ro.x, ro.y, ro.z, ro.w)
            local = pts @ R.T + np.array([tr.x, tr.y, tr.z])
            f = self.inflate
            inside = (
                (local[:, 0] > box[0] - f) & (local[:, 0] < box[1] + f) &
                (local[:, 1] > box[2] - f) & (local[:, 1] < box[3] + f) &
                (local[:, 2] > box[4] - f) & (local[:, 2] < box[5] + f)
            )
            pts = pts[~inside]
            if len(pts) == 0:
                break
        return pts

    # --------------------------------------------------------------- callback
    def cb(self, msg):
        pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"),
                                    skip_nans=True)
        if pts.size == 0:
            return
        n_in = len(pts)

        pts = self.filter_scan(pts, msg.header.frame_id, msg.header)
        if len(pts) == 0:
            return

        if self.filt_pub is not None:
            self.publish(self.filt_pub, pts.astype(np.float32),
                         msg.header.frame_id, msg.header.stamp)

        try:
            tf = self.buf.lookup_transform(
                self.frame, msg.header.frame_id, Time(),
                timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn("TF %s -> %s: %s"
                                   % (msg.header.frame_id, self.frame, e),
                                   throttle_duration_sec=5.0)
            return

        tr, ro = tf.transform.translation, tf.transform.rotation
        R = quat_to_R(ro.x, ro.y, ro.z, ro.w)
        world = (pts @ R.T + np.array([tr.x, tr.y, tr.z])).astype(np.float32)

        self.acc = np.vstack([self.acc, world])
        self.n_scans += 1
        self.n_in += n_in
        self.n_kept += len(pts)

        if len(self.acc) > self.max_points:
            self.acc = voxel_downsample(self.acc, max(self.voxel, 0.05))

        if self.n_scans % self.pub_every == 0:
            self.acc = voxel_downsample(self.acc, self.voxel)
            self.publish(self.map_pub, self.acc, self.frame,
                         self.get_clock().now().to_msg())
            self.get_logger().info(
                "scans=%d  map=%d pts  (kept %.0f%% of returns)"
                % (self.n_scans, len(self.acc),
                   100.0 * self.n_kept / max(self.n_in, 1)),
                throttle_duration_sec=5.0)

    # ---------------------------------------------------------------- outputs
    def publish(self, pub, pts, frame_id, stamp):
        m = PointCloud2()
        m.header.stamp = stamp
        m.header.frame_id = frame_id
        m.height = 1
        m.width = pts.shape[0]
        m.is_dense = True
        m.is_bigendian = False
        m.fields = XYZ_FIELDS
        m.point_step = 12
        m.row_step = 12 * m.width
        m.data = pts.astype(np.float32).tobytes()
        pub.publish(m)

    def save(self):
        path = self.get_parameter("save_path").value
        if not path:
            self.get_logger().info("no save_path set - not writing a file")
            return
        if len(self.acc) == 0:
            self.get_logger().warn("nothing accumulated - not writing a file")
            return

        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        pts = voxel_downsample(self.acc, self.voxel)
        with open(path, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write("WIDTH %d\nHEIGHT 1\n" % len(pts))
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write("POINTS %d\nDATA ascii\n" % len(pts))
            np.savetxt(f, pts, fmt="%.4f")
        self.get_logger().info("saved %d points -> %s" % (len(pts), path))


def main():
    rclpy.init()
    node = LidarMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("interrupted - saving map...")
    finally:
        try:
            node.save()
        except Exception as e:
            print("save failed:", e)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
