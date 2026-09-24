#!/usr/bin/env python3
"""
dig_zone_monitor.py — live terrain + change detection in the dig zone.

Unlike lidar_mapper (which accumulates the union of all observations and so
cannot show change), this keeps a 2.5D elevation grid where each cell holds the
LATEST observed height. Digging lowers a cell; spoil raises it. Subtracting a
reference snapshot gives a difference map -- the representation ALICE uses to
drive excavation planning, and the thing that makes the merged two-lidar cloud
actually useful in real time.

  /lidar_*/points ─► self-filter ─► crop to dig zone ─► elevation grid
                                                          │
                              ┌───────────────────────────┼──────────────┐
                              ▼                           ▼              ▼
                   /dig_zone/elevation        /dig_zone/change    volume log
                   (PointCloud2, z=height)    (intensity = delta)  (m^3)

Topics
------
  /dig_zone/elevation  PointCloud2  current surface, one point per cell
  /dig_zone/change     PointCloud2  intensity = height - reference
                                    negative = material removed (dug)
                                    positive = material added (spoil)

Reference control
-----------------
  ros2 param set /dig_zone_monitor take_reference true    # snapshot "before"
  ros2 param set /dig_zone_monitor clear_grid true        # wipe and restart

Params
------
  input_topics   ['/lidar_fl/points', '/lidar_fr/points']
  fixed_frame    'base_link'
  cell_size      0.10   m, elevation grid resolution
  r_min / r_max  1.8 / 4.5  m, dig-zone annulus (from the work envelope)
  sector_deg     45.0   half-width about arm_yaw (None-like: use 180 for full)
  arm_yaw_deg    180.0  bearing of the arm (-X)
  z_min / z_max  -1.5 / 1.0  m, height crop (kills roof/stray outliers)
  min_hits       2      cell must be seen this many times to be published
  ema            0.4    0..1, weight of the newest height (1.0 = pure latest)
  min_range      2.2    m from sensor, self-hit crop
  inflate        0.15   m, link-box margin

Example
-------
  ros2 run excavator_interactive_rviz dig_zone_monitor.py --ros-args \
    -p input_topics:="['/lidar_fl/points','/lidar_fr/points']" \
    -p use_sim_time:=true
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.parameter import Parameter

import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float32

# link boxes for the self-filter (URDF collision mesh extents, metres)
LINK_BOXES = {
    "base":   (-0.94, 1.50, -0.85, 0.85,  0.03, 0.62),
    "body":   (-0.07, 0.30, -0.37, 0.38,  0.34, 1.06),
    "boom":   (-0.04, 2.63, -0.09, 0.09, -0.67, 0.43),
    "stick":  (-1.53, -1.16, -0.08, 0.08, -0.30, -0.01),
    "bucket": (-0.17, 0.59, -0.29, 0.29, -0.05, 0.54),
}

XYZI_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


class DigZoneMonitor(Node):
    def __init__(self):
        super().__init__("dig_zone_monitor")

        p = self.declare_parameter
        p("input_topics", ["/lidar_fl/points", "/lidar_fr/points"])
        p("fixed_frame", "base_link")
        p("cell_size", 0.10)
        p("r_min", 1.8)
        p("r_max", 4.5)
        p("sector_deg", 45.0)
        p("arm_yaw_deg", 180.0)
        p("z_min", -1.5)
        p("z_max", 1.0)
        p("min_hits", 2)
        p("ema", 0.4)
        p("min_range", 2.2)
        p("inflate", 0.15)
        p("take_reference", False)
        p("clear_grid", False)

        g = self.get_parameter
        self.frame = g("fixed_frame").value
        self.cell = float(g("cell_size").value)
        self.r_min = float(g("r_min").value)
        self.r_max = float(g("r_max").value)
        self.sector = float(g("sector_deg").value)
        self.arm_yaw = float(g("arm_yaw_deg").value)
        self.z_min = float(g("z_min").value)
        self.z_max = float(g("z_max").value)
        self.min_hits = int(g("min_hits").value)
        self.ema = float(g("ema").value)
        self.min_range = float(g("min_range").value)
        self.inflate = float(g("inflate").value)

        # grid indexed by (ix, iy) -> [height, hits]
        self.height = {}
        self.reference = None

        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

        self.pub_elev = self.create_publisher(PointCloud2, "/dig_zone/elevation", 1)
        self.pub_diff = self.create_publisher(PointCloud2, "/dig_zone/change", 1)
        self.pub_vol = self.create_publisher(Float32, "/dig_zone/volume_removed", 1)

        for t in g("input_topics").value:
            self.create_subscription(PointCloud2, t, self.cb, 5)

        self.n = 0
        self.get_logger().info(
            "dig_zone_monitor: %s -> /dig_zone/*  cell=%.2f m  r=%.1f-%.1f m"
            % (list(g("input_topics").value), self.cell, self.r_min, self.r_max))
        self.get_logger().info(
            "  snapshot reference: ros2 param set /dig_zone_monitor take_reference true")

    # ------------------------------------------------------------- filtering
    def self_filter(self, pts, sensor_frame):
        if self.min_range > 0.0:
            pts = pts[np.linalg.norm(pts, axis=1) >= self.min_range]
            if not len(pts):
                return pts
        for link, box in LINK_BOXES.items():
            try:
                tf = self.buf.lookup_transform(link, sensor_frame, Time(),
                                               timeout=Duration(seconds=0.1))
            except Exception:
                continue
            tr, ro = tf.transform.translation, tf.transform.rotation
            R = quat_to_R(ro.x, ro.y, ro.z, ro.w)
            loc = pts @ R.T + np.array([tr.x, tr.y, tr.z])
            f = self.inflate
            inside = (
                (loc[:, 0] > box[0] - f) & (loc[:, 0] < box[1] + f) &
                (loc[:, 1] > box[2] - f) & (loc[:, 1] < box[3] + f) &
                (loc[:, 2] > box[4] - f) & (loc[:, 2] < box[5] + f))
            pts = pts[~inside]
            if not len(pts):
                break
        return pts

    def arm_bearing(self):
        """Bearing of the arm in the fixed frame, from the live cab heading."""
        try:
            tf = self.buf.lookup_transform(self.frame, "body", Time(),
                                           timeout=Duration(seconds=0.1))
        except Exception:
            return self.arm_yaw
        ro = tf.transform.rotation
        R = quat_to_R(ro.x, ro.y, ro.z, ro.w)
        d = R @ np.array([1.0, 0.0, 0.0])   # arm is +X in the body frame
        return math.degrees(math.atan2(d[1], d[0]))

    def crop_zone(self, pts):
        r = np.hypot(pts[:, 0], pts[:, 1])
        m = ((r >= self.r_min) & (r <= self.r_max) &
             (pts[:, 2] >= self.z_min) & (pts[:, 2] <= self.z_max))
        if self.sector < 180.0:
            centre = self.arm_bearing()          # <-- live, not fixed
            yaw = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
            d = (yaw - centre + 180.0) % 360.0 - 180.0
            m &= np.abs(d) <= self.sector
        return pts[m]

    # -------------------------------------------------------------- callback
    def cb(self, msg):
        if self.get_parameter("clear_grid").value:
            self.height.clear()
            self.reference = None
            self.set_parameters([Parameter("clear_grid", Parameter.Type.BOOL, False)])
            self.get_logger().info("grid cleared")

        pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"),
                                    skip_nans=True)
        if pts.size == 0:
            return

        pts = self.self_filter(pts, msg.header.frame_id)
        if not len(pts):
            return

        try:
            tf = self.buf.lookup_transform(self.frame, msg.header.frame_id,
                                           Time(), timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn("TF: %s" % e, throttle_duration_sec=5.0)
            return

        tr, ro = tf.transform.translation, tf.transform.rotation
        R = quat_to_R(ro.x, ro.y, ro.z, ro.w)
        w = pts @ R.T + np.array([tr.x, tr.y, tr.z])

        w = self.crop_zone(w)
        if not len(w):
            return

        # --- latest-wins elevation update -------------------------------
        ix = np.floor(w[:, 0] / self.cell).astype(np.int64)
        iy = np.floor(w[:, 1] / self.cell).astype(np.int64)
        order = np.lexsort((iy, ix))
        ix, iy, z = ix[order], iy[order], w[order, 2]

        # highest return per cell in this scan (terrain surface, not the
        # underside of anything the beam clipped)
        keys = np.stack([ix, iy], axis=1)
        _, starts = np.unique(keys, axis=0, return_index=True)
        starts = np.sort(starts)
        ends = np.append(starts[1:], len(z))

        for s, e in zip(starts, ends):
            k = (int(ix[s]), int(iy[s]))
            z_new = float(np.max(z[s:e]))
            if k in self.height:
                h, hits = self.height[k]
                self.height[k] = (self.ema * z_new + (1 - self.ema) * h, hits + 1)
            else:
                self.height[k] = (z_new, 1)

        if self.get_parameter("take_reference").value:
            self.reference = {k: v[0] for k, v in self.height.items()
                              if v[1] >= self.min_hits}
            self.set_parameters(
                [Parameter("take_reference", Parameter.Type.BOOL, False)])
            self.get_logger().info("reference snapshot: %d cells"
                                   % len(self.reference))

        self.n += 1
        if self.n % 5 == 0:
            self.publish_all(msg.header.stamp)

    # --------------------------------------------------------------- output
    def publish_all(self, stamp):
        cells = [(k, v[0]) for k, v in self.height.items()
                 if v[1] >= self.min_hits]
        if not cells:
            return

        elev = np.array(
            [[(k[0] + 0.5) * self.cell, (k[1] + 0.5) * self.cell, h, h]
             for k, h in cells], dtype=np.float32)
        self.publish(self.pub_elev, elev, stamp)

        if self.reference is None:
            return

        diff, removed = [], 0.0
        for k, h in cells:
            if k not in self.reference:
                continue
            d = h - self.reference[k]
            diff.append([(k[0] + 0.5) * self.cell,
                         (k[1] + 0.5) * self.cell, h, d])
            if d < 0:
                removed += -d * self.cell * self.cell
        if not diff:
            return

        self.publish(self.pub_diff, np.array(diff, dtype=np.float32), stamp)

        m = Float32()
        m.data = float(removed)
        self.pub_vol.publish(m)
        self.get_logger().info("cells=%d  volume removed=%.3f m3"
                               % (len(diff), removed),
                               throttle_duration_sec=3.0)

    def publish(self, pub, arr, stamp):
        m = PointCloud2()
        m.header.stamp = stamp
        m.header.frame_id = self.frame
        m.height = 1
        m.width = arr.shape[0]
        m.is_dense = True
        m.is_bigendian = False
        m.fields = XYZI_FIELDS
        m.point_step = 16
        m.row_step = 16 * m.width
        m.data = arr.astype(np.float32).tobytes()
        pub.publish(m)


def main():
    rclpy.init()
    node = DigZoneMonitor()
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