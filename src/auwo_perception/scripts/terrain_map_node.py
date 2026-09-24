#!/usr/bin/env python3
"""
terrain_map_node.py — the operator's terrain map. Latest-wins, layered.

    /lidar_*/points ─► self-filter ─► TF ─► grid ─┬─► /auwo/map        (GridMap)
                                                  ├─► /auwo/map_cloud  (PointCloud2)
                                                  └─► /auwo/progress   (Float32MultiArray)

WHY THIS EXISTS ALONGSIDE lidar_mapper
lidar_mapper accumulates the union of every observation and never forgets. That
is CORRECT for the sensor placement study, where coverage means "was this ever
observed". It is wrong for an operator: move the bucket through the beam and it
leaves a permanent ghost, and digging a hole leaves the old surface in place.

This node keeps ONE height per cell, the most recent. Look at a cell again and
it updates. The ghost clears itself.

LAYERS
  elevation   latest surface height, NaN where never observed
  n_obs       observation count
  age         seconds since last observed  <- render stale cells faded
  variance    spread of recent observations, a confidence proxy
  target      desired surface, NaN until a dig region is set
  remaining   elevation - target, >0 means material still to remove
  reachable   1 inside the bucket-tip work envelope, else 0

TWO SENSORS AT DIFFERENT HEIGHTS - WHY OBSERVATIONS ARE WEIGHTED
The roof_plus_boom rig has a 16-beam lidar 0.40 m up and a wide-FOV lidar
2.87 m up. They see the same ground at very different grazing angles:

    distance    boom (0.40 m)    roof (2.87 m)
      3 m           7.6 deg         43.7 deg
      6 m           3.8 deg         25.6 deg
     10 m           2.3 deg         16.0 deg

A beam grazing at 2 deg has an enormous ground footprint, and range error turns
almost entirely into height error. Weighting every return equally would let the
boom lidar's far-field junk overwrite the roof lidar's good measurements in the
3 - 8 m band - which is the dump sector.

So each return is weighted by sin(grazing angle), normalised so 45 deg counts
fully, and returns below `min_grazing_deg` are discarded outright. The effect
is automatic and needs no per-sensor configuration: near the machine only the
boom lidar can see and it dominates; further out the roof lidar wins on
geometry. The `quality` layer records the best grazing angle a cell has ever
been measured at, so the operator and the planner can tell a well-measured
cell from a grazed guess.

STALENESS IS INFORMATION, NOT A REASON TO FORGET
Cells that stop being observed keep their last height and their age grows. That
is deliberate and differs from ALICE and HEAP, whose cells decay: their machines
drive, so the world moves relative to them. A slewing excavator is the only
agent changing that ground, so the last look is still the best estimate - but
the operator must be able to see how old it is. Hence the `age` layer.

DIG REGION
Subscribes to /auwo/dig_region (PolygonStamped) and /auwo/dig_depth (Float32).
When both are set it fills `target` and `remaining` inside the polygon and
publishes progress. Depth is measured DOWN from the surface at the moment the
region was set, so progress is measured against the operator's intent rather
than a moving reference.

Params
  input_topics   ['/lidar_boom/points']
  fixed_frame    'base_link'
  length_x/y     24.0    map extent, metres
  resolution     0.15    metres per cell
  min_range      0.5     self-hit crop, metres from the sensor
  inflate        0.15    link-box margin
  z_min / z_max  -3.0 / 3.0   height crop, kills stray returns
  min_hits       2       cell must be seen this often to be published
  ema            0.4     weight of the newest height (1.0 = pure latest)
  reach_min/max  1.86 / 4.36  bucket-tip work envelope
  map_hz         1.0     GridMap rate (it is a big message)
  cloud_hz       2.0     PointCloud2 rate

Example
  ros2 run auwo_perception terrain_map_node.py --ros-args \
      -p input_topics:="['/lidar_boom/points','/lidar_roof/points']"
"""

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import PolygonStamped

try:
    from grid_map_msgs.msg import GridMap, GridMapInfo
    HAVE_GRID_MAP = True
except ImportError:
    HAVE_GRID_MAP = False

# URDF collision-mesh extents per link, for the self-filter (metres)
LINK_BOXES = {
    "base":   (-0.94, 1.50, -0.85, 0.85,  0.03, 0.62),
    "body":   (-0.07, 0.30, -0.37, 0.38,  0.34, 1.06),
    "boom":   (-0.04, 2.63, -0.09, 0.09, -0.67, 0.43),
    "stick":  (-1.53, -1.16, -0.08, 0.08, -0.30, -0.01),
    "bucket": (-0.17, 0.59, -0.29, 0.29, -0.05, 0.54),
}

LAYERS = ["elevation", "n_obs", "age", "variance", "quality",
          "target", "remaining", "reachable"]


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


def point_in_poly(px, py, poly):
    """Ray casting. poly is an (N,2) array."""
    inside = np.zeros(px.shape, dtype=bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        cond = ((y1 > py) != (y2 > py))
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = (x2 - x1) * (py - y1) / (y2 - y1) + x1
        inside ^= cond & (px < xint)
    return inside


class TerrainMap(Node):
    def __init__(self):
        super().__init__("terrain_map_node")

        p = self.declare_parameter
        p("input_topics", ["/lidar_boom/points"])
        p("fixed_frame", "base_link")
        p("length_x", 24.0)
        p("length_y", 24.0)
        p("resolution", 0.15)
        p("min_range", 0.5)
        p("min_grazing_deg", 4.0)   # below this a return is discarded
        p("exclude_radius", 1.2)    # cylinder around the slew axis, see below
        p("inflate", 0.15)
        p("z_min", -3.0)
        p("z_max", 3.0)
        p("min_hits", 2)
        p("ema", 0.4)
        p("reach_min", 1.86)
        p("reach_max", 4.36)
        p("map_hz", 1.0)
        p("cloud_hz", 2.0)

        g = self.get_parameter
        self.frame = g("fixed_frame").value
        self.res = float(g("resolution").value)
        self.lx = float(g("length_x").value)
        self.ly = float(g("length_y").value)
        self.min_range = float(g("min_range").value)
        self.inflate = float(g("inflate").value)
        self.z_min = float(g("z_min").value)
        self.z_max = float(g("z_max").value)
        self.min_hits = int(g("min_hits").value)
        self.ema = float(g("ema").value)

        self.nr = int(round(self.lx / self.res))     # rows run along -x
        self.nc = int(round(self.ly / self.res))     # cols run along -y

        z = lambda: np.zeros((self.nr, self.nc), dtype=np.float32)
        nan = lambda: np.full((self.nr, self.nc), np.nan, dtype=np.float32)

        self.min_graze = math.radians(float(g("min_grazing_deg").value))
        self.excl_r = float(g("exclude_radius").value)
        self.ref_graze = math.sin(math.radians(45.0))   # 45 deg counts fully

        self.elev = nan()
        self.nobs = z()
        self.last_t = nan()
        self.var = z()
        self.quality = z()          # best grazing angle seen, degrees
        self.target = nan()

        # reachable is static: an annulus from the bucket-tip work envelope
        cx, cy = self._cell_centres()
        r = np.hypot(cx, cy)
        self.reach = ((r >= float(g("reach_min").value)) &
                      (r <= float(g("reach_max").value))).astype(np.float32)

        self.region = None          # (N,2) polygon in the fixed frame
        self.depth = None           # metres below the surface at set time

        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

        self.pub_cloud = self.create_publisher(PointCloud2, "/auwo/map_cloud", 1)
        self.pub_prog = self.create_publisher(Float32MultiArray, "/auwo/progress", 1)
        self.pub_map = None
        if HAVE_GRID_MAP:
            self.pub_map = self.create_publisher(GridMap, "/auwo/map", 1)
        else:
            self.get_logger().warn(
                "grid_map_msgs not found - publishing PointCloud2 only. "
                "Install with: sudo apt install ros-jazzy-grid-map-msgs")

        for t in g("input_topics").value:
            self.create_subscription(PointCloud2, t, self._on_cloud, 5)
        self.create_subscription(PolygonStamped, "/auwo/dig_region",
                                 self._on_region, 1)
        self.create_subscription(Float32, "/auwo/dig_depth", self._on_depth, 1)

        self.map_dt = 1.0 / max(float(g("map_hz").value), 0.1)
        self.cloud_dt = 1.0 / max(float(g("cloud_hz").value), 0.1)
        self.t_map = 0.0
        self.t_cloud = 0.0

        self.get_logger().info(
            "terrain map %.0f x %.0f m at %.2f m -> %d x %d cells"
            % (self.lx, self.ly, self.res, self.nr, self.nc))
        self.get_logger().info("   inputs %s in '%s'"
                               % (list(g("input_topics").value), self.frame))

    # ------------------------------------------------------------- geometry
    def _cell_centres(self):
        """x, y of every cell centre. Row 0 is max x, col 0 is max y."""
        xs = self.lx / 2.0 - (np.arange(self.nr) + 0.5) * self.res
        ys = self.ly / 2.0 - (np.arange(self.nc) + 0.5) * self.res
        return np.meshgrid(xs, ys, indexing="ij")

    def _index(self, x, y):
        i = ((self.lx / 2.0 - x) / self.res).astype(np.int64)
        j = ((self.ly / 2.0 - y) / self.res).astype(np.int64)
        ok = (i >= 0) & (i < self.nr) & (j >= 0) & (j < self.nc)
        return i, j, ok

    # ------------------------------------------------------------- callbacks
    def _on_region(self, msg):
        pts = [(p.x, p.y) for p in msg.polygon.points]
        if len(pts) < 3:
            self.region = None
            self.target[:] = np.nan
            self.get_logger().info("dig region cleared")
            return
        self.region = np.array(pts, dtype=np.float64)
        self.get_logger().info("dig region set: %d vertices" % len(pts))
        self._rebuild_target()

    def _on_depth(self, msg):
        self.depth = float(msg.data)
        self.get_logger().info("dig depth set: %.2f m" % self.depth)
        self._rebuild_target()

    def _rebuild_target(self):
        """target = surface at this moment, minus the requested depth."""
        self.target[:] = np.nan
        if self.region is None or self.depth is None:
            return
        cx, cy = self._cell_centres()
        inside = point_in_poly(cx, cy, self.region)
        seen = inside & np.isfinite(self.elev) & (self.nobs >= self.min_hits)
        self.target[seen] = self.elev[seen] - self.depth
        self.get_logger().info(
            "target set on %d cells (%d in region, %d unobserved)"
            % (int(seen.sum()), int(inside.sum()),
               int((inside & ~seen).sum())))

    def _self_filter(self, pts, sensor_frame):
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

    def _on_cloud(self, msg):
        pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"),
                                    skip_nans=True)
        if pts.size == 0:
            return
        pts = self._self_filter(pts, msg.header.frame_id)
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

        w = w[(w[:, 2] >= self.z_min) & (w[:, 2] <= self.z_max)]
        if not len(w):
            return

        # Everything within this radius of the slew axis is machine, not
        # terrain. The URDF link boxes cover base/body/boom/stick/bucket but
        # NOT the sensor mast, the camera housings at 1.85 m, or the lidar
        # bodies themselves - and the roof lidar looks straight down onto all
        # of them, producing points at head height with nothing there. No
        # usable ground is lost: the boom lidar's nearest ground return is
        # about 1.5 m from the axis and the roof lidar's is further still.
        if self.excl_r > 0.0:
            w = w[np.hypot(w[:, 0], w[:, 1]) >= self.excl_r]
            if not len(w):
                return

        # grazing angle of each return, measured from the sensor's own origin.
        # Shallow returns have huge footprints and turn range error into height
        # error, so they are discarded and the rest are down-weighted.
        origin = np.array([tr.x, tr.y, tr.z])
        v = w - origin
        horiz = np.hypot(v[:, 0], v[:, 1])
        graze = np.arctan2(np.abs(v[:, 2]), np.maximum(horiz, 1e-6))
        keep_g = graze >= self.min_graze
        w, graze = w[keep_g], graze[keep_g]
        if not len(w):
            return

        i, j, ok = self._index(w[:, 0], w[:, 1])
        i, j, zz, gz = i[ok], j[ok], w[ok, 2], graze[ok]
        if not len(zz):
            return

        # best-quality return per cell in this scan, not merely the highest:
        # a steeply-seen point beats a grazed one even if slightly lower
        order = np.lexsort((-gz, j, i))
        i, j, zz, gz = i[order], j[order], zz[order], gz[order]
        keep = np.ones(len(i), dtype=bool)
        keep[1:] = (i[1:] != i[:-1]) | (j[1:] != j[:-1])
        i, j, zz, gz = i[keep], j[keep], zz[keep], gz[keep]

        now = time.monotonic()
        old = self.elev[i, j]
        fresh = ~np.isfinite(old)

        # a return counts fully at 45 deg grazing and fades towards zero
        q = np.clip(np.sin(gz) / self.ref_graze, 0.0, 1.0)
        alpha = np.clip(self.ema * q, 0.02, 1.0)

        new = np.where(fresh, zz, alpha * zz + (1.0 - alpha) * old)
        d = np.where(fresh, 0.0, zz - old)

        self.elev[i, j] = new
        self.var[i, j] = np.where(fresh, 0.0,
                                  0.7 * self.var[i, j] + 0.3 * d * d)
        self.quality[i, j] = np.maximum(self.quality[i, j], np.degrees(gz))
        self.nobs[i, j] += 1.0
        self.last_t[i, j] = now

        self._maybe_publish(now)

    # ---------------------------------------------------------------- output
    def _maybe_publish(self, now):
        if now - self.t_cloud >= self.cloud_dt:
            self.t_cloud = now
            self._publish_cloud(now)
            self._publish_progress()
        if self.pub_map is not None and now - self.t_map >= self.map_dt:
            self.t_map = now
            self._publish_gridmap(now)

    def _remaining(self):
        with np.errstate(invalid="ignore"):
            return self.elev - self.target

    def _publish_cloud(self, now):
        good = np.isfinite(self.elev) & (self.nobs >= self.min_hits)
        if not good.any():
            return
        cx, cy = self._cell_centres()
        rem = self._remaining()
        age = now - self.last_t

        n = int(good.sum())
        arr = np.empty((n, 7), dtype=np.float32)
        arr[:, 0] = cx[good]
        arr[:, 1] = cy[good]
        arr[:, 2] = self.elev[good]
        arr[:, 3] = self.elev[good]                       # intensity
        arr[:, 4] = np.nan_to_num(age[good], nan=999.0)
        arr[:, 5] = np.nan_to_num(rem[good], nan=0.0)
        arr[:, 6] = self.quality[good]

        m = PointCloud2()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.height = 1
        m.width = n
        m.is_dense = True
        m.is_bigendian = False
        m.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="age", offset=16,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="remaining", offset=20,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name="quality", offset=24,
                       datatype=PointField.FLOAT32, count=1),
        ]
        m.point_step = 28
        m.row_step = 28 * n
        m.data = arr.tobytes()
        self.pub_cloud.publish(m)

    def _publish_gridmap(self, now):
        age = now - self.last_t
        rem = self._remaining()
        data = {
            "elevation": self.elev,
            "n_obs": self.nobs,
            "age": np.nan_to_num(age, nan=999.0).astype(np.float32),
            "variance": self.var,
            "quality": self.quality,
            "target": self.target,
            "remaining": rem.astype(np.float32),
            "reachable": self.reach,
        }

        m = GridMap()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.info = GridMapInfo()
        m.info.resolution = self.res
        m.info.length_x = self.lx
        m.info.length_y = self.ly
        m.info.pose.orientation.w = 1.0
        m.layers = LAYERS
        m.basic_layers = ["elevation"]
        m.outer_start_index = 0
        m.inner_start_index = 0

        for name in LAYERS:
            a = Float32MultiArray()
            d0 = MultiArrayDimension()
            d0.label = "column_index"
            d0.size = self.nc
            d0.stride = self.nr * self.nc
            d1 = MultiArrayDimension()
            d1.label = "row_index"
            d1.size = self.nr
            d1.stride = self.nr
            a.layout.dim = [d0, d1]
            # grid_map stores column-major
            a.data = data[name].astype(np.float32).flatten(order="F").tolist()
            m.data.append(a)

        self.pub_map.publish(m)

    def _publish_progress(self):
        if self.region is None or self.depth is None:
            return
        tgt = np.isfinite(self.target)
        if not tgt.any():
            return
        rem = self._remaining()
        cell = self.res * self.res
        to_remove = np.clip(rem[tgt], 0.0, None)
        total = float(self.depth) * int(tgt.sum()) * cell
        left = float(to_remove.sum()) * cell

        m = Float32MultiArray()
        m.data = [
            float(total),                       # m3 at the time the region was set
            float(max(total - left, 0.0)),      # m3 removed
            float(left),                        # m3 remaining
            # clamped: the surface drifts a few mm after the target is frozen,
            # which can make `left` marginally exceed `total`
            float(max(0.0, min(100.0, 100.0 * (1.0 - left / total))))
            if total > 1e-6 else 0.0,
            float(int(tgt.sum())),              # cells with a target
            float(int((to_remove <= 0.02).sum())),   # cells at target
        ]
        self.pub_prog.publish(m)


def main():
    rclpy.init()
    node = TerrainMap()
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