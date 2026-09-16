#!/usr/bin/env python3
"""
clicked_point_adapter.py — draw a dig region by clicking in RViz.

    RViz "Publish Point" tool ─► /clicked_point ─► this node ─┬─► /auwo/dig_region
                                                              ├─► /auwo/dig_depth
                                                              └─► /auwo/dig_region_markers

HOW TO USE IT
  1. In RViz pick the "Publish Point" tool from the toolbar.
  2. Click the corners of the area to excavate. Each click drops a marker and
     the outline grows.
  3. Close the shape by clicking near the FIRST point again (within
     close_radius), or by calling /auwo/finish_region.
  4. The region and depth are published latched, so a client that connects
     later still receives them.

  Start over:   ros2 service call /auwo/clear_region std_srvs/srv/Trigger
  Change depth: ros2 param set /auwo/clicked_point_adapter depth 0.8

WHY AN ADAPTER RATHER THAN A DASHBOARD
/clicked_point is RViz-specific. The rest of the system only knows about
/auwo/dig_region and /auwo/dig_depth, so RViz is one client among several. When
the AUWO dashboard arrives it publishes the same two topics and this node is
simply not launched - nothing downstream changes. Same reason the markers are
remapped off the machine command topic.

REACHABILITY IS A WARNING, NOT A VETO
Clicks outside the bucket-tip work envelope (1.86 - 4.36 m radial) are flagged
but still accepted: the machine may be repositioned later, and an operator
sketching a plan should not be blocked by the current stance. terrain_map_node
carries a `reachable` layer so the planner can make the real decision.

Params
  fixed_frame     base_link
  depth           0.5    metres to excavate below the current surface
  close_radius    0.6    click within this of the first point to close
  min_points      3
  reach_min/max   1.86 / 4.36   work envelope, for the warning only

Example
  ros2 run auwo_operator clicked_point_adapter.py
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import Point, Point32, PointStamped, PolygonStamped
from std_msgs.msg import ColorRGBA, Float32
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(depth=1,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE)


def rgba(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


class ClickedPointAdapter(Node):
    def __init__(self):
        super().__init__("clicked_point_adapter")

        p = self.declare_parameter
        p("fixed_frame", "base_link")
        p("depth", 0.5)
        p("close_radius", 0.6)
        p("min_points", 3)
        p("reach_min", 1.86)
        p("reach_max", 4.36)

        g = self.get_parameter
        self.frame = g("fixed_frame").value
        self.close_r = float(g("close_radius").value)
        self.min_pts = int(g("min_points").value)
        self.rmin = float(g("reach_min").value)
        self.rmax = float(g("reach_max").value)

        self.pts = []          # [(x, y)] in the fixed frame
        self.closed = False

        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

        self.pub_region = self.create_publisher(
            PolygonStamped, "/auwo/dig_region", LATCHED)
        self.pub_depth = self.create_publisher(
            Float32, "/auwo/dig_depth", LATCHED)
        self.pub_marks = self.create_publisher(
            MarkerArray, "/auwo/dig_region_markers", 1)

        self.create_subscription(PointStamped, "/clicked_point",
                                 self._on_click, 10)
        # Depth over a topic so the dashboard can set it before closing a
        # region, without reaching for a parameter dialog.
        self.create_subscription(Float32, "/auwo/set_depth", self._on_depth, 10)
        self.create_service(Trigger, "/auwo/clear_region", self._srv_clear)
        self.create_service(Trigger, "/auwo/finish_region", self._srv_finish)

        self.get_logger().info(
            "clicked_point_adapter ready. Use RViz 'Publish Point' to mark "
            "the corners; click near the first point to close.")
        self.get_logger().info("   depth %.2f m, close radius %.2f m"
                               % (float(g("depth").value), self.close_r))

    # --------------------------------------------------------------- clicks
    def _to_fixed(self, msg):
        if msg.header.frame_id in ("", self.frame):
            return msg.point.x, msg.point.y
        try:
            tf = self.buf.lookup_transform(
                self.frame, msg.header.frame_id, Time(),
                timeout=Duration(seconds=0.3))
        except Exception as e:
            self.get_logger().warn("cannot transform click from '%s': %s"
                                   % (msg.header.frame_id, e))
            return None
        t = tf.transform.translation
        # clicks land on the ground plane, so a translation-only transform is
        # adequate here; the map itself is built with full rotations
        return msg.point.x + t.x, msg.point.y + t.y

    def _on_click(self, msg):
        xy = self._to_fixed(msg)
        if xy is None:
            return
        x, y = xy

        if self.closed:
            self.get_logger().info("region was closed - starting a new one")
            self.pts, self.closed = [], False

        if self.pts and len(self.pts) >= self.min_pts:
            x0, y0 = self.pts[0]
            if math.hypot(x - x0, y - y0) <= self.close_r:
                self._finish()
                return

        r = math.hypot(x, y)
        self.pts.append((x, y))
        note = ""
        if r < self.rmin or r > self.rmax:
            note = "  (outside the %.2f - %.2f m work envelope)" % (self.rmin, self.rmax)
        self.get_logger().info("point %d: (%.2f, %.2f)  r=%.2f m%s"
                               % (len(self.pts), x, y, r, note))
        self._publish_markers()

    def _on_depth(self, msg):
        d = float(msg.data)
        if d <= 0.0:
            self.get_logger().warn("ignoring non-positive depth %.2f" % d)
            return
        self.set_parameters(
            [rclpy.parameter.Parameter("depth", rclpy.Parameter.Type.DOUBLE, d)])
        self.get_logger().info("depth set to %.2f m" % d)
        self._publish_markers()          # the label shows the new volume
        if self.closed:
            self._publish_region()       # re-issue so the target is rebuilt

    # -------------------------------------------------------------- services
    def _srv_clear(self, req, resp):
        self.pts, self.closed = [], False
        self._publish_region(clear=True)
        self._publish_markers()
        resp.success = True
        resp.message = "region cleared"
        self.get_logger().info("region cleared")
        return resp

    def _srv_finish(self, req, resp):
        if len(self.pts) < self.min_pts:
            resp.success = False
            resp.message = "need at least %d points, have %d" % (
                self.min_pts, len(self.pts))
            return resp
        self._finish()
        resp.success = True
        resp.message = "region published with %d points" % len(self.pts)
        return resp

    # --------------------------------------------------------------- publish
    def _finish(self):
        self.closed = True
        area = self._area()
        depth = float(self.get_parameter("depth").value)
        self.get_logger().info(
            "region closed: %d points, %.1f m2, depth %.2f m -> about %.2f m3"
            % (len(self.pts), area, depth, area * depth))
        self._publish_region()
        self._publish_markers()

    def _area(self):
        """Shoelace formula."""
        n = len(self.pts)
        if n < 3:
            return 0.0
        s = 0.0
        for i in range(n):
            x1, y1 = self.pts[i]
            x2, y2 = self.pts[(i + 1) % n]
            s += x1 * y2 - x2 * y1
        return abs(s) / 2.0

    def _publish_region(self, clear=False):
        m = PolygonStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        if not clear:
            for x, y in self.pts:
                pt = Point32()
                pt.x, pt.y, pt.z = float(x), float(y), 0.0
                m.polygon.points.append(pt)
        self.pub_region.publish(m)

        d = Float32()
        d.data = 0.0 if clear else float(self.get_parameter("depth").value)
        self.pub_depth.publish(d)

    def _publish_markers(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def base(mid, mtype, scale, colour):
            mk = Marker()
            mk.header.stamp = stamp
            mk.header.frame_id = self.frame
            mk.ns = "dig_region"
            mk.id = mid
            mk.type = mtype
            mk.action = Marker.ADD if self.pts else Marker.DELETE
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = scale
            mk.color = colour
            return mk

        # corner spheres
        sph = base(0, Marker.SPHERE_LIST, 0.25, rgba(1.0, 0.85, 0.2))
        for x, y in self.pts:
            sph.points.append(Point(x=float(x), y=float(y), z=0.15))
        arr.markers.append(sph)

        # outline: open while drawing, looped once closed
        line = base(1, Marker.LINE_STRIP, 0.08,
                    rgba(0.2, 0.9, 0.5) if self.closed else rgba(0.6, 0.6, 0.6))
        seq = list(self.pts) + ([self.pts[0]] if self.closed and self.pts else [])
        for x, y in seq:
            line.points.append(Point(x=float(x), y=float(y), z=0.12))
        arr.markers.append(line)

        # label with area and depth, once the shape means something
        txt = base(2, Marker.TEXT_VIEW_FACING, 0.45, rgba(1.0, 1.0, 1.0))
        if len(self.pts) >= self.min_pts:
            cx = sum(p[0] for p in self.pts) / len(self.pts)
            cy = sum(p[1] for p in self.pts) / len(self.pts)
            txt.pose.position.x = float(cx)
            txt.pose.position.y = float(cy)
            txt.pose.position.z = 1.0
            d = float(self.get_parameter("depth").value)
            a = self._area()
            txt.text = ("%.1f m2  x  %.2f m  =  %.2f m3%s"
                        % (a, d, a * d, "" if self.closed else "   (open)"))
        else:
            txt.action = Marker.DELETE
        arr.markers.append(txt)

        self.pub_marks.publish(arr)


def main():
    rclpy.init()
    node = ClickedPointAdapter()
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