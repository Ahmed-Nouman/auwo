#!/usr/bin/env python3
"""
keyboard_to_joy.py — drive auwo_joy_teleop from the keyboard.

Publishes sensor_msgs/Joy, so the real teleop node runs completely unchanged.
When a gamepad arrives, stop this node, start joy_node, and nothing else moves.

    keyboard ─► /joy ─► auwo_joy_teleop ─► /auwo/cmd/teleop ─► arbiter ─► machine

KEYS  (WASD is the left stick, IJKL the right stick - same layout as the pad)

        W                    I               W / S   stick (dipper) out / in
      A   D                J   L             A / D   slew left / right
        S                    K               I / K   boom up / down
                                             J / L   bucket curl / dump

        B   turbo (double speed while held)
        Q   quit

THE DEADMAN IS "ARE YOU STILL TYPING"
A terminal gives no key-up event, so a key counts as held while its auto-repeat
keeps arriving, and is released HOLD_S after the repeats stop. The deadman
button is reported as pressed whenever any movement key is active. Stop
pressing keys and, within ~0.2 s, this node reports the deadman released, the
teleop node goes quiet, and the arbiter freezes the machine.

That is the same failure path as releasing a real deadman, unplugging the pad,
or losing the network - which is the point of testing it this way.

NOTE ON FEEL
Terminal auto-repeat has an initial delay of roughly half a second, so the
first moment of a press stutters. That is the terminal, not the control loop.
Reduce it with:   xset r rate 200 40

Run this node in a terminal WITH FOCUS - it reads that terminal's keystrokes.

Example
  ros2 run excavator_interactive_rviz keyboard_to_joy.py
  ros2 run excavator_interactive_rviz auwo_joy_teleop.py --ros-args -p use_sim_time:=true
  ros2 run excavator_interactive_rviz auwo_arbiter.py --ros-args -p start_mode:=teleop
"""

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

HOLD_S = 0.45      # a key counts as held this long after its last repeat.
                   # Must exceed the terminal's INITIAL auto-repeat delay
                   # (~0.5 s by default) or the deadman drops between the
                   # first keypress and the repeat stream, which makes the
                   # teleop node re-seed and lose progress.
                   # Shorten it after: xset r rate 200 40
RATE_HZ = 30.0

# key -> (axis index, value).  Indices match a standard gamepad:
#   0 left X, 1 left Y, 3 right X, 4 right Y
KEYMAP = {
    "a": (0, +1.0), "d": (0, -1.0),      # slew
    "w": (1, +1.0), "s": (1, -1.0),      # stick / dipper
    "j": (3, +1.0), "l": (3, -1.0),      # bucket
    "i": (4, +1.0), "k": (4, -1.0),      # boom
}
TURBO_KEY = "b"
DEADMAN_BUTTON = 4
TURBO_BUTTON = 5
N_AXES, N_BUTTONS = 6, 8


class KeyboardToJoy(Node):
    def __init__(self):
        super().__init__("keyboard_to_joy")
        self.pub = self.create_publisher(Joy, "/joy", 10)
        self.seen = {}                    # key -> time last observed
        self.quit = False
        self.was_active = False

        self.get_logger().info("keyboard -> /joy   WASD left stick, IJKL right stick")
        self.get_logger().info("   B turbo, Q quit. Keep THIS terminal focused.")

    def note_keys(self):
        now = time.time()
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1).lower()
            if ch == "q":
                self.quit = True
                return
            if ch == "\x03":              # ctrl-C
                self.quit = True
                return
            if ch in KEYMAP or ch == TURBO_KEY:
                self.seen[ch] = now

    def active(self, key):
        return (time.time() - self.seen.get(key, 0.0)) < HOLD_S

    def tick(self):
        self.note_keys()
        if self.quit:
            return

        axes = [0.0] * N_AXES
        buttons = [0] * N_BUTTONS
        moving = False

        for key, (axis, val) in KEYMAP.items():
            if self.active(key):
                axes[axis] += val
                moving = True

        for i in range(N_AXES):
            axes[i] = max(-1.0, min(1.0, axes[i]))

        buttons[DEADMAN_BUTTON] = 1 if moving else 0
        buttons[TURBO_BUTTON] = 1 if self.active(TURBO_KEY) else 0

        if moving != self.was_active:
            self.get_logger().info("deadman %s" % ("HELD" if moving else "released"))
            self.was_active = moving

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = axes
        msg.buttons = buttons
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = KeyboardToJoy()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        period = 1.0 / RATE_HZ
        while rclpy.ok() and not node.quit:
            node.tick()
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        # one last all-zero frame so nothing is left latched
        try:
            m = Joy()
            m.axes = [0.0] * N_AXES
            m.buttons = [0] * N_BUTTONS
            node.pub.publish(m)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\nkeyboard_to_joy stopped")


if __name__ == "__main__":
    main()