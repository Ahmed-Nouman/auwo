#!/usr/bin/env python3
"""
clock_watch.py — watch /clock and /tf for time going backwards.

    ros2 run --prefix 'python3' ...   # or just:
    python3 clock_watch.py

TF_OLD_DATA means a transform arrived stamped earlier than one already in the
buffer. With a single publisher on /clock and a single one on /tf - which is
what `ros2 topic info --verbose` reports here - that cannot be two sources
disagreeing. Something is actually moving time backwards.

This prints every backward step it sees, on both the clock and each TF frame,
so the cause stops being a guess. Leave it running, then reproduce.

WHAT THE OUTPUT MEANS
  CLOCK WENT BACK      the simulator's own time jumped backwards. Stopping and
                       replaying the Isaac timeline does this (sim time resets
                       to zero), and so does scrubbing it. Everything stamped
                       against that clock is then "from the past" until sim
                       time climbs past where it was.
  FRAME <x> WENT BACK  one frame's transforms regressed while the clock did
                       not - that is a publisher problem, not a clock problem.
  (nothing)            time is monotonic now; the warnings in the log were
                       from an earlier stop/replay and have already cleared.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


def to_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Watch(Node):
    def __init__(self):
        # deliberately NOT use_sim_time: this node has to be able to see the
        # sim clock misbehave, so it must not run on it
        super().__init__("clock_watch")
        qos = QoSProfile(depth=50)
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.last_clock = None
        self.last_frame = {}
        self.n_clock = 0
        self.n_tf = 0
        self.back_clock = 0
        self.back_tf = 0

        self.create_subscription(Clock, "/clock", self.on_clock, qos)
        self.create_subscription(TFMessage, "/tf", self.on_tf, qos)
        self.create_timer(5.0, self.summary)

        self.get_logger().info("watching /clock and /tf for backward steps")
        self.get_logger().info("reproduce the problem now; Ctrl-C to stop")

    def on_clock(self, msg):
        t = to_s(msg.clock)
        self.n_clock += 1
        if self.last_clock is not None and t < self.last_clock - 1e-9:
            self.back_clock += 1
            print("CLOCK WENT BACK  %.4f -> %.4f   (%.4f s backwards)"
                  % (self.last_clock, t, self.last_clock - t))
        self.last_clock = t

    def on_tf(self, msg):
        for tr in msg.transforms:
            f = tr.child_frame_id
            t = to_s(tr.header.stamp)
            self.n_tf += 1
            prev = self.last_frame.get(f)
            if prev is not None and t < prev - 1e-9:
                self.back_tf += 1
                if self.back_tf <= 20:        # do not flood
                    print("FRAME %-22s WENT BACK  %.4f -> %.4f"
                          % (f, prev, t))
            self.last_frame[f] = t

    def summary(self):
        print("  [%d clock msgs, %d backward]   [%d tf msgs, %d backward]   "
              "sim time now %s"
              % (self.n_clock, self.back_clock, self.n_tf, self.back_tf,
                 "%.2f" % self.last_clock if self.last_clock else "?"))


def main():
    rclpy.init()
    n = Watch()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print("clock steps backwards : %d" % n.back_clock)
        print("tf steps backwards    : %d" % n.back_tf)
        if n.back_clock:
            print("\nThe simulator's clock is resetting. Stopping and replaying"
                  "\nthe Isaac timeline does exactly this. Start Isaac playing"
                  "\nFIRST, then launch the ROS stack, and do not stop the"
                  "\ntimeline while it runs.")
        elif n.back_tf:
            print("\nThe clock is fine but transforms regress - that is"
                  "\nrobot_state_publisher, not the simulator.")
        elif n.n_clock:
            print("\nTime is monotonic. The TF_OLD_DATA warnings came from an"
                  "\nearlier stop/replay and have cleared.")
        else:
            print("\nNo /clock messages at all - is Isaac playing?")
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
