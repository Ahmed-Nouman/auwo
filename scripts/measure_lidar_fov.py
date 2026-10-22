#!/usr/bin/env python3
"""
measure_lidar_fov.py — measure a lidar's actual FOV from its published cloud.

    python3 measure_lidar_fov.py /avia/points
    python3 measure_lidar_fov.py /lidar_boom/points --frames 20

WHY NOT JUST LOOK AT RVIZ
You cannot read a FOV off a point cloud in RViz. What you see there is the
cloud's GROUND FOOTPRINT, which is the FOV after two transformations:

  - the mount tilt. 35.8 deg nose-down maps the vertical FOV onto a ground
    annulus. Steep rays land near the machine, shallow ones run to the
    horizon, and the angle between them is nowhere on screen.
  - perspective. A fan that is 70 deg at the sensor looks wider or narrower
    depending on the camera, and a ground plane exaggerates the near field.

So a footprint that "looks wrong" may be a correct sensor seen at an angle, or
a wrong sensor that happens to look plausible. This measures the angles in the
SENSOR's own frame, where the tilt and the camera do not enter.

WHAT IT CANNOT TELL YOU
Only rays that HIT something produce points, so this measures the RETURNED
FOV, never the emitted one. In particular, on a downward-tilted sensor over
open ground, every ray above the horizon flies off and returns nothing - the
measured upper elevation stops at the horizon, not at the sensor's true edge.

    measured elevation span <= emitted elevation span,   always

Azimuth is the trustworthy axis: ground returns span the full horizontal FOV,
so measured azimuth should match the spec closely. If it does not, the profile
is wrong.

To measure the full emitted FOV, give every ray something to hit - put the
sensor inside a box in the sim, or point it at a wall.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class FovMeasure(Node):
    def __init__(self, topic, want_frames):
        super().__init__("measure_lidar_fov")
        self.topic = topic
        self.want = want_frames
        self.seen = 0
        self.az, self.el, self.rng = [], [], []

        # Sensor drivers publish BEST_EFFORT; a RELIABLE subscriber silently
        # receives nothing from one, which looks exactly like a dead topic.
        qos = QoSProfile(depth=5)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self.create_subscription(PointCloud2, topic, self.on_cloud, qos)
        print("listening on %s for %d frame(s)..." % (topic, want_frames))

    def on_cloud(self, msg):
        if self.seen >= self.want:
            return
        self.seen += 1
        n = 0
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"),
                                          skip_nans=True):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            r = math.sqrt(x * x + y * y + z * z)
            if r < 1e-6:
                continue            # the origin is not a direction
            self.az.append(math.degrees(math.atan2(y, x)))
            self.el.append(math.degrees(math.atan2(z, math.hypot(x, y))))
            self.rng.append(r)
            n += 1
        print("   frame %d: %d points (frame_id %s)"
              % (self.seen, n, msg.header.frame_id))

    def done(self):
        return self.seen >= self.want


def pct(vals, q):
    s = sorted(vals)
    if not s:
        return float("nan")
    k = max(0, min(len(s) - 1, int(round(q / 100.0 * (len(s) - 1)))))
    return s[k]


def histogram(vals, lo, hi, bins=24, width=46):
    if not vals:
        return
    counts = [0] * bins
    step = (hi - lo) / bins if hi > lo else 1.0
    for v in vals:
        b = int((v - lo) / step)
        counts[min(bins - 1, max(0, b))] += 1
    peak = max(counts) or 1
    for i, c in enumerate(counts):
        a0 = lo + i * step
        bar = "#" * int(round(width * c / peak))
        print("   %7.1f  %-*s %d" % (a0, width, bar, c))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("topic")
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--expect-h", type=float, default=None,
                    help="expected horizontal FOV, deg (Avia: 70.4)")
    ap.add_argument("--expect-v", type=float, default=None,
                    help="expected vertical FOV, deg (Avia: 77.2)")
    args = ap.parse_args()

    rclpy.init()
    node = FovMeasure(args.topic, args.frames)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=2.0)
    except KeyboardInterrupt:
        pass

    az, el, rng = node.az, node.el, node.rng
    if not az:
        print("\nno points received on %s" % args.topic)
        print("Is Isaac playing? Does the topic publish? Check:")
        print("   ros2 topic hz %s" % args.topic)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print("\n%d points over %d frame(s)\n" % (len(az), node.seen))

    # 1st/99th percentile alongside min/max: a handful of stray returns at a
    # grazing angle can stretch min/max by degrees, and then a correct sensor
    # reads as out of spec. If the two disagree a lot, the tails are noise.
    print("%-12s %-22s %-22s" % ("", "full range", "1st - 99th pct"))
    for name, v in (("azimuth", az), ("elevation", el)):
        print("%-12s %7.2f .. %7.2f      %7.2f .. %7.2f   (span %.1f / %.1f)"
              % (name, min(v), max(v), pct(v, 1), pct(v, 99),
                 max(v) - min(v), pct(v, 99) - pct(v, 1)))
    print("%-12s %7.2f .. %7.2f m" % ("range", min(rng), max(rng)))

    off = [math.degrees(math.acos(max(-1.0, min(1.0,
           math.cos(math.radians(a)) * math.cos(math.radians(e))))))
           for a, e in zip(az, el)]
    print("%-12s %7.2f deg (99th pct %.2f)"
          % ("max off-axis", max(off), pct(off, 99)))

    print("\nAZIMUTH distribution")
    histogram(az, min(az), max(az))
    print("\nELEVATION distribution")
    histogram(el, min(el), max(el))

    if args.expect_h or args.expect_v:
        # FULL range, not the percentile span. The percentiles are there to
        # show whether the edges are sparse; comparing them to spec throws
        # away the outermost 2% of the cloud, which on a FOV measurement is
        # the only part that defines the answer. Doing that reported a sensor
        # measuring 70.1 of a specified 70.4 deg as "MISMATCH 64.1".
        print("\nAGAINST SPEC   (full range; percentiles are for the tails)")
        if args.expect_h:
            got = max(az) - min(az)
            print("   horizontal  %.1f measured vs %.1f expected   %s"
                  % (got, args.expect_h,
                     "ok" if abs(got - args.expect_h) < 1.5 else "<-- MISMATCH"))
            print("               (1-99 pct span %.1f - a much smaller number"
                  % (pct(az, 99) - pct(az, 1)))
            print("                here just means the edges are thinly "
                  "sampled)")
        if args.expect_v:
            got = max(el) - min(el)
            print("   vertical    %.1f measured vs %.1f expected   %s"
                  % (got, args.expect_v,
                     "ok" if abs(got - args.expect_v) < 1.5 else "(short)"))
            print("      A SHORTFALL HERE IS EXPECTED on a tilted sensor over")
            print("      open ground: rays above the horizon return nothing,")
            print("      so the top of the FOV is invisible. With a %.1f deg"
                  % args.expect_v)
            print("      FOV tilted 35.8 deg down, only the lower %.1f deg"
                  % min(args.expect_v, 35.8 + args.expect_v / 2))
            print("      can produce ground returns at all.")
            print("      AZIMUTH is the axis that should match outright.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()