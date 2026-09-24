#!/usr/bin/env python3
"""
self_filter.py — remove lidar returns that land on the excavator itself.

Two mechanisms:

  1. RANGE CROP  — drop points closer than `min_range` to the sensor. Kills the
     inner "roof/cab" ring. With a 30 deg tilt the usable ground coverage
     starts around 2.4 m, so a 2.0-2.5 m crop costs no real terrain.

  2. LINK BOXES  — for each arm link, transform points into that link's frame
     (TF already carries the live joint angles) and drop anything inside an
     axis-aligned box. Kills the outer "arm" ring. Boxes come from the URDF
     collision mesh extents, inflated by `inflate`.

  /lidar_cab/points  -->  /lidar_cab/points_filtered

Params:
  input_topic   default '/lidar_cab/points'
  output_topic  default '/lidar_cab/points_filtered'
  min_range     default 2.0   (metres from sensor origin)
  inflate       default 0.15  (metres added to every box face)

Example:
  ros2 run <pkg> self_filter.py --ros-args \
      -p input_topic:=/lidar_cab/points -p min_range:=2.2 -p use_sim_time:=true
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

# Link bounding boxes in each link's own frame, from the URDF collision meshes
# (DAE vertices x 0.001 scale). (xmin, xmax, ymin, ymax, zmin, zmax) in metres.
LINK_BOXES = {
    "base":   (-0.94, 1.50, -0.85, 0.85,  0.03, 0.62),
    "body":   (-0.07, 0.30, -0.37, 0.38,  0.34, 1.06),
    "boom":   (-0.04, 2.63, -0.09, 0.09, -0.67, 0.43),
    "stick":  (-1.53, -1.16, -0.08, 0.08, -0.30, -0.01),
    "bucket": (-0.17, 0.59, -0.29, 0.29, -0.05, 0.54),
}


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


class SelfFilter(Node):
    def __init__(self):
        super().__init__("self_filter")

        self.declare_parameter("input_topic", "/lidar_cab/points")
        self.declare_parameter("output_topic", "/lidar_cab/points_filtered")
        self.declare_parameter("min_range", 2.0)
        self.declare_parameter("inflate", 0.15)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self.min_range = float(self.get_parameter("min_range").value)
        self.inflate = float(self.get_parameter("inflate").value)

        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

        self.pub = self.create_publisher(PointCloud2, out_topic, 5)
        self.create_subscription(PointCloud2, in_topic, self.cb, 5)

        self.n = 0
        self.get_logger().info(
            "self_filter: %s -> %s   min_range=%.2f m  inflate=%.2f m"
            % (in_topic, out_topic, self.min_range, self.inflate))

    def cb(self, msg):
        pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"),
                                    skip_nans=True)
        if pts.size == 0:
            return
        n_in = len(pts)

        # --- 1. range crop (points are in the sensor frame) ---
        if self.min_range > 0.0:
            r = np.linalg.norm(pts, axis=1)
            pts = pts[r >= self.min_range]
            if len(pts) == 0:
                return

        # --- 2. link boxes ---
        for link, box in LINK_BOXES.items():
            try:
                tf = self.buf.lookup_transform(
                    link, msg.header.frame_id, Time(),
                    timeout=Duration(seconds=0.1))
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
                return

        self.n += 1
        if self.n % 20 == 0:
            self.get_logger().info("kept %d / %d points" % (len(pts), n_in))

        self.publish(pts.astype(np.float32), msg.header)

    def publish(self, pts, header):
        out = PointCloud2()
        out.header = header
        out.height = 1
        out.width = pts.shape[0]
        out.is_dense = True
        out.is_bigendian = False
        out.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out.point_step = 12
        out.row_step = 12 * out.width
        out.data = pts.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()