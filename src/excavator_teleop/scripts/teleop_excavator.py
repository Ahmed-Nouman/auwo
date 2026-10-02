#!/usr/bin/env python3
import sys, termios, tty, select, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from excavator_models import default_model, load_profile

HELP = "Keys: a/d body, w/s boom, i/k stick, j/l bucket, SPACE safe pose, q quit"
HELP_BLADE = "      u/o blade (models with a blade)"

# --- Joint limits: set from the excavator model profile in Teleop.__init__ ---
# (v1 values: boom [-1.308, -0.087], stick [-2.428, -0.085], bucket [-2.395, -0.357])
LIMS_MIN = [float('-inf'), -1.308, -2.428, -2.395]  # body is continuous
LIMS_MAX = [float('inf'),  -0.087, -0.085, -0.357]
SAFE_POSE = [0.0, -0.5, -1.0, -1.0]  # inside limits

def clamp(q):
    return [max(LIMS_MIN[i], min(LIMS_MAX[i], q[i])) for i in range(4)]


def _apply_profile(profile):
    """Replace the module-level limits and safe pose with the model's values."""
    global LIMS_MIN, LIMS_MAX, SAFE_POSE
    lim = profile['ui_limits']
    LIMS_MIN = [float('-inf')] + [float(lim[j][0]) for j in profile['arm_joints'][1:]]
    LIMS_MAX = [float('inf')] + [float(lim[j][1]) for j in profile['arm_joints'][1:]]
    SAFE_POSE = [float(v) for v in profile['safe_pose']]

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_excavator')

        # Excavator model (v1/v2) -> limits, safe pose, blade
        model = self.declare_parameter('excavator_model', default_model()).value
        profile = load_profile(model)
        _apply_profile(profile)
        blade_lim = profile.get('urdf_limits', {}).get('blade_rotation')
        self.has_blade = 'blade_controller' in profile.get('extra_controllers', []) and blade_lim
        self.blade_min, self.blade_max = (float(blade_lim[0]), float(blade_lim[1])) if self.has_blade else (0.0, 0.0)
        self.blade = 0.0
        self.blade_pub = self.create_publisher(
            JointTrajectory, '/blade_controller/joint_trajectory', 10) if self.has_blade else None

        # Built-in defaults so you don't need --ros-args
        default_topic = '/arm_position_controller/commands'
        default_step = 0.01       # <= your requested default
        default_rate_hz = 20.0

        # Still declare ROS params (optional override via params/YAML if you ever want)
        self.declare_parameter('topic', default_topic)
        self.declare_parameter('step', default_step)
        self.declare_parameter('rate_hz', default_rate_hz)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value or default_topic
        self.step  = float(self.get_parameter('step').get_parameter_value().double_value or default_step)
        rate_hz    = float(self.get_parameter('rate_hz').get_parameter_value().double_value or default_rate_hz)

        self.pub = self.create_publisher(Float64MultiArray, self.topic, 10)
        self.q = SAFE_POSE.copy()

        self._stop = threading.Event()

        # Timer publishes at a steady rate; guard against publishing after stop
        self.timer = self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(
            f"Model {profile['model']}: publishing to {self.topic} at {rate_hz} Hz (step={self.step})")
        self.get_logger().info(HELP)
        if self.has_blade:
            self.get_logger().info(HELP_BLADE)

        # Non-blocking keyboard reader in a thread
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

    def _publish(self):
        # Avoid "Destroyable ... destruction was requested" by not using node after stop
        if self._stop.is_set():
            return
        self.pub.publish(Float64MultiArray(data=self.q))

    def _publish_blade(self):
        if self._stop.is_set() or self.blade_pub is None:
            return
        traj = JointTrajectory()
        traj.joint_names = ['blade_rotation']
        pt = JointTrajectoryPoint()
        pt.positions = [self.blade]
        pt.time_from_start = Duration(sec=0, nanosec=200_000_000)
        traj.points.append(pt)
        self.blade_pub.publish(traj)

    def _keyboard_loop(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while not self._stop.is_set():
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not r:
                    continue
                c = sys.stdin.read(1)
                if c == 'q':
                    self.get_logger().info("Quit")
                    self._stop.set()
                    return
                elif c == ' ':
                    self.q = SAFE_POSE.copy()
                elif c == 'a':
                    self.q[0] += self.step
                elif c == 'd':
                    self.q[0] -= self.step
                elif c == 'w':
                    self.q[1] = clamp([self.q[0], self.q[1] + self.step, self.q[2], self.q[3]])[1]
                elif c == 's':
                    self.q[1] = clamp([self.q[0], self.q[1] - self.step, self.q[2], self.q[3]])[1]
                elif c == 'i':
                    self.q[2] = clamp([self.q[0], self.q[1], self.q[2] + self.step, self.q[3]])[2]
                elif c == 'k':
                    self.q[2] = clamp([self.q[0], self.q[1], self.q[2] - self.step, self.q[3]])[2]
                elif c == 'j':
                    self.q[3] = clamp([self.q[0], self.q[1], self.q[2], self.q[3] + self.step])[3]
                elif c == 'l':
                    self.q[3] = clamp([self.q[0], self.q[1], self.q[2], self.q[3] - self.step])[3]
                elif c in ('u', 'o') and self.has_blade:
                    # negative = blade up
                    delta = -self.step if c == 'u' else self.step
                    self.blade = max(self.blade_min, min(self.blade_max, self.blade + delta))
                    self._publish_blade()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    rclpy.init()
    node = Teleop()
    exec = SingleThreadedExecutor()
    exec.add_node(node)
    try:
        while rclpy.ok() and not node._stop.is_set():
            exec.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop publishing before destroying the node to avoid "Destroyable..." warnings
        try:
            node._stop.set()
            if node.timer is not None:
                node.timer.cancel()
        except Exception:
            pass
        try:
            if node._kb_thread.is_alive():
                node._kb_thread.join(timeout=0.5)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

