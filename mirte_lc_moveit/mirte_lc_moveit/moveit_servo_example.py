#!/usr/bin/env python3
"""
Keyboard teleop node for MoveIt Servo (Python version)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import termios
import tty
import signal

KEYCODE_UP    = "\x41"
KEYCODE_DOWN  = "\x42"
KEYCODE_RIGHT = "\x43"
KEYCODE_LEFT  = "\x44"
KEYCODE_W = "w"
KEYCODE_S = "s"
KEYCODE_Q = "q"

PLANNING_FRAME = "base_link"
STEP = 0.3


class KeyboardServo(Node):
    def __init__(self):
        super().__init__("servo_keyboard_input", parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])
        self.twist_pub = self.create_publisher(TwistStamped, "/delta_twist_cmds", 10)
        self.get_logger().info(
            "Keyboard Servo Node started.\n"
            "  Arrow UP/DOWN  → X axis\n"
            "  Arrow LEFT/RIGHT → Y axis\n"
            "  W/S            → Z axis\n"
            "  Q              → quit"
        )

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while True:
                c = sys.stdin.read(1)
                if not c:
                    continue
                if self.handle_key(c):
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def handle_key(self, c):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = PLANNING_FRAME

        if c == KEYCODE_UP:
            msg.twist.linear.x = STEP
        elif c == KEYCODE_DOWN:
            msg.twist.linear.x = -STEP
        elif c == KEYCODE_LEFT:
            msg.twist.linear.y = STEP
        elif c == KEYCODE_RIGHT:
            msg.twist.linear.y = -STEP
        elif c == KEYCODE_W:
            msg.twist.linear.z = STEP
        elif c == KEYCODE_S:
            msg.twist.linear.z = -STEP
        elif c.lower() == KEYCODE_Q:
            return True
        else:
            return False

        self.twist_pub.publish(msg)
        return False


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardServo()

    def signal_handler(sig, frame):
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    node.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()