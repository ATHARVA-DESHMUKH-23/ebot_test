#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class KeyboardEEVel(Node):

    def __init__(self):
        super().__init__('keyboard_ee_vel')
        self.pub = self.create_publisher(Twist, '/ee_velocity', 10)

        self.speed = 0.05
        self.cmd = Twist()   # <-- LATCHED COMMAND

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(
            "Keyboard control:\n"
            "  w/s : +Y / -Y\n"
            "  a/d : -X / +X\n"
            "  space: stop\n"
            "  Ctrl+C to quit"
        )

        self.timer = self.create_timer(0.05, self.loop)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1) if rlist else None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def loop(self):
        key = self.get_key()

        if key == 'w':
            self.cmd.linear.y = self.speed
            self.cmd.linear.x = 0.0
        elif key == 's':
            self.cmd.linear.y = -self.speed
            self.cmd.linear.x = 0.0
        elif key == 'a':
            self.cmd.linear.x = -self.speed
            self.cmd.linear.y = 0.0
        elif key == 'd':
            self.cmd.linear.x = self.speed
            self.cmd.linear.y = 0.0
        elif key == ' ':
            self.cmd = Twist()   # STOP
        elif key == '\x03':
            rclpy.shutdown()

        self.pub.publish(self.cmd)

def main():
    rclpy.init()
    node = KeyboardEEVel()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
