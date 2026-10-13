#!/usr/bin/env python3
"""
slew_scan.py — rotate the excavator body for a mapping run (closed loop).

Advances the body_rotation command only once the measured joint angle has
caught up, so it works regardless of Isaac's real-time factor. The arm joints
are held at a fixed pose so scan geometry is repeatable between sensor
configurations.

Params:
  sweep_deg    total rotation                     (default 360)
  step_deg     command increment                  (default 2.0)
  tol_deg      "reached" tolerance                (default 1.0)
  settle_s     extra wall-time wait per step      (default 0.0)
  timeout_s    give up on a step after this long  (default 15.0)
  boom / stick / bucket  held arm pose in rad

Example:
  ros2 run <pkg> slew_scan.py --ros-args -p step_deg:=2.0 -p tol_deg:=1.0
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

BODY_JOINT = "body_rotation"


class SlewScan(Node):
    def __init__(self):
        super().__init__("slew_scan")

        self.declare_parameter("sweep_deg", 360.0)
        self.declare_parameter("step_deg", 2.0)
        self.declare_parameter("tol_deg", 1.0)
        self.declare_parameter("settle_s", 0.0)
        self.declare_parameter("timeout_s", 15.0)
        self.declare_parameter("boom", -0.6)
        self.declare_parameter("stick", -1.0)
        self.declare_parameter("bucket", 0.3)

        self.measured = None
        self.pub = self.create_publisher(
            Float64MultiArray, "/arm_position_controller/commands", 10)
        self.create_subscription(JointState, "/joint_states", self.js_cb, 10)

    def js_cb(self, msg):
        if BODY_JOINT in msg.name:
            self.measured = msg.position[msg.name.index(BODY_JOINT)]

    def send(self, ang_rad, arm):
        m = Float64MultiArray()
        m.data = [ang_rad] + arm
        self.pub.publish(m)

    def run(self):
        sweep = float(self.get_parameter("sweep_deg").value)
        step = float(self.get_parameter("step_deg").value)
        tol = math.radians(float(self.get_parameter("tol_deg").value))
        settle = float(self.get_parameter("settle_s").value)
        timeout = float(self.get_parameter("timeout_s").value)
        arm = [
            float(self.get_parameter("boom").value),
            float(self.get_parameter("stick").value),
            float(self.get_parameter("bucket").value),
        ]

        self.get_logger().info(
            "closed-loop slew: %.0f deg in %.1f deg steps (tol %.1f deg)"
            % (sweep, step, math.degrees(tol)))

        # wait for first joint state
        t0 = time.time()
        while rclpy.ok() and self.measured is None and time.time() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.measured is None:
            self.get_logger().error("no /joint_states received - is Isaac playing?")
            return

        start = self.measured
        self.get_logger().info("start angle %.3f rad" % start)

        # settle the arm into the held pose first
        self.get_logger().info("moving arm to hold pose...")
        for _ in range(30):
            self.send(start, arm)
            rclpy.spin_once(self, timeout_sec=0.1)

        target = start
        travelled = 0.0
        next_report = 0.0

        while rclpy.ok() and travelled < sweep:
            target += math.radians(step)
            travelled += step

            t_step = time.time()
            while rclpy.ok():
                self.send(target, arm)
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.measured is not None and abs(self.measured - target) <= tol:
                    break
                if time.time() - t_step > timeout:
                    self.get_logger().warn(
                        "step timeout at %.0f deg (measured %.3f, target %.3f)"
                        % (travelled, self.measured, target))
                    break

            if settle > 0.0:
                time.sleep(settle)

            if travelled >= next_report:
                self.get_logger().info(
                    "  %5.0f deg   measured %.3f rad" % (travelled, self.measured))
                next_report += 30.0

        self.get_logger().info("sweep complete: %.0f deg" % travelled)


def main():
    rclpy.init()
    node = SlewScan()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()