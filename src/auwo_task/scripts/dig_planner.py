#!/usr/bin/env python3
"""
dig_planner.py — choose where to dig next. Mode M2.

    /auwo/map_cloud ─► pick a cell ─► /auwo/dig_target ─► excavation_executor

Reads the `remaining` channel of the operator terrain map, finds the cell with
the most material still above target, checks it is inside the work envelope,
and publishes it as the next dig pose.

DELIBERATELY DUMB TO BEGIN WITH
"Most material left, if reachable" is enough to close the loop. Refinement
never changes an interface, so it can wait. When it comes, ALICE's criteria are
the ones to adopt: prefer poses that will fill the bucket, cap the depth step so
material does not slide into holes already finished, and work the far positions
first because every dig drags material toward the machine. That last one is
already here as `far_bias`.

Re-planning after every cycle rather than pre-planning a sequence is also
ALICE's choice, and for the same reason: the surface changes unpredictably as
material slides and spills, so any pre-computed order is wrong by the second
bucket.

THIS IS THE NODE THAT BECOMES M3
Right now the operator draws the region and this picks a pose inside it. Mode
M3 - the machine proposing the region itself - is the same node choosing the
region too, writing the same topic. Nothing downstream changes.

Params
  reach_min/max   1.86 / 4.36   bucket-tip work envelope
  min_remaining   0.05   m, below this a cell counts as finished
  far_bias        0.15   m of bonus per metre of radius, works outward-in
  dig_depth_cap   0.30   m, deepest single bite
  replan_hz       1.0
  settle_s        2.0    ignore the map briefly after a target is issued

Example
  ros2 run auwo_task dig_planner.py
"""

import math
import time

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_srvs.srv import Trigger


class DigPlanner(Node):
    def __init__(self):
        super().__init__("dig_planner")

        p = self.declare_parameter
        p("map_topic", "/auwo/map_cloud")
        p("fixed_frame", "base_link")
        p("reach_min", 1.86)
        p("reach_max", 4.36)
        p("min_remaining", 0.05)
        p("far_bias", 0.15)
        p("dig_depth_cap", 0.30)
        p("replan_hz", 1.0)
        p("settle_s", 2.0)
        p("switch_margin", 0.06)

        g = self.get_parameter
        self.frame = g("fixed_frame").value
        self.rmin = float(g("reach_min").value)
        self.rmax = float(g("reach_max").value)
        self.min_rem = float(g("min_remaining").value)
        self.far_bias = float(g("far_bias").value)
        self.cap = float(g("dig_depth_cap").value)
        self.settle = float(g("settle_s").value)
        self.margin = float(g("switch_margin").value)
        self.current = None        # (x, y) of the target being worked

        self.cloud = None
        self.t_last = 0.0
        self.period = 1.0 / max(float(g("replan_hz").value), 0.1)
        self.reported = None

        self.pub = self.create_publisher(PoseStamped, "/auwo/dig_target", 1)
        self.create_subscription(PointCloud2, g("map_topic").value,
                                 self._on_map, 1)
        self.create_service(Trigger, "/auwo/next_target", self._srv_next)
        self.create_timer(self.period, self._tick)

        self.get_logger().info(
            "dig planner: %s -> /auwo/dig_target, envelope %.2f - %.2f m"
            % (g("map_topic").value, self.rmin, self.rmax))

    def _on_map(self, msg):
        self.cloud = msg

    # ---------------------------------------------------------------- choose
    def _choose(self):
        if self.cloud is None:
            return None, "no map yet"
        try:
            pts = pc2.read_points_numpy(
                self.cloud, field_names=("x", "y", "z", "remaining"),
                skip_nans=True)
        except Exception:
            return None, "map has no 'remaining' channel - is a region set?"
        if pts.size == 0:
            return None, "map is empty"

        x, y, z, rem = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
        r = np.hypot(x, y)

        ok = (rem > self.min_rem) & (r >= self.rmin) & (r <= self.rmax)
        if not ok.any():
            in_region = (rem > self.min_rem).sum()
            if in_region:
                return None, ("%d cells still have material but none are "
                              "inside the work envelope" % int(in_region))
            return None, "region complete"

        # most material first, with a nudge toward the far edge because every
        # dig drags material back toward the machine
        score = rem[ok] + self.far_bias * r[ok]
        xs, ys, zs, rems = x[ok], y[ok], z[ok], rem[ok]
        i = int(np.argmax(score))

        # HYSTERESIS. Without it the planner flip-flops: when many cells hold
        # nearly the same amount of material, millimetre noise in the map moves
        # the argmax every update and the machine wanders between them instead
        # of finishing one. Keep the cell being worked unless another beats it
        # by a real margin.
        if self.current is not None:
            d = np.hypot(xs - self.current[0], ys - self.current[1])
            j = int(np.argmin(d))
            if d[j] < 0.25 and score[j] >= score[i] - self.margin:
                i = j

        self.current = (float(xs[i]), float(ys[i]))
        bite = float(min(rems[i], self.cap))
        return (float(xs[i]), float(ys[i]),
                float(zs[i]) - bite, bite, int(ok.sum())), ""

    def _publish(self, pick):
        x, y, z, bite, n = pick
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z            # already the bottom of this bite
        m.pose.orientation.w = 1.0
        self.pub.publish(m)
        self.get_logger().info(
            "target (%.2f, %.2f, %.2f)  r=%.2f m  bite %.2f m  "
            "%d cells left" % (x, y, z, math.hypot(x, y), bite, n))

    def _tick(self):
        now = time.monotonic()
        if now - self.t_last < self.settle:
            return
        pick, why = self._choose()
        if pick is None:
            if why != self.reported:
                self.get_logger().info(why)
                self.reported = why
            self.current = None
            return
        self.reported = None
        self.t_last = now
        self._publish(pick)

    def _srv_next(self, req, resp):
        pick, why = self._choose()
        if pick is None:
            resp.success = False
            resp.message = why
            return resp
        self._publish(pick)
        self.t_last = time.monotonic()
        resp.success = True
        resp.message = "target at %.2f, %.2f, %.2f" % pick[:3]
        return resp


def main():
    rclpy.init()
    node = DigPlanner()
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