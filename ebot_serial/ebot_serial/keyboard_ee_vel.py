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

        # MUST match Cartesian servo
        self.pub = self.create_publisher(Twist, '/ee_velocity_arm', 10)

        self.speed_xy = 0.02   # m/s
        self.speed_z  = 0.02   # m/s
        self.speed_yaw = 0.02   # rad/s

        self.cmd = Twist()   # latched command

        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info(
            "\nKeyboard EE Velocity Control\n"
            "-----------------------------\n"
            "w / s : +X / -X\n"
            "a / d : +Y / -Y\n"
            "r / f : +Z / -Z\n"
            "q / e : +Yaw / -Yaw\n"
            "space : STOP\n"
            "Ctrl+C: Quit\n"
        )

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

    # ------------------------------------------------
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        key = sys.stdin.read(1) if rlist else None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    # ------------------------------------------------
    def loop(self):
        key = self.get_key()

        if key is None:
            self.pub.publish(self.cmd)
            return

        # Reset angular unless yaw key
        if key not in ['q', 'e']:
            self.cmd.angular.z = 0.0

        if key == 'w':
            self.cmd.linear.x = self.speed_xy
            self.cmd.linear.y = 0.0

        elif key == 's':
            self.cmd.linear.x = -self.speed_xy
            self.cmd.linear.y = 0.0

        elif key == 'a':
            self.cmd.linear.y = self.speed_xy
            self.cmd.linear.x = 0.0

        elif key == 'd':
            self.cmd.linear.y = -self.speed_xy
            self.cmd.linear.x = 0.0

        elif key == 'r':
            self.cmd.linear.z = self.speed_z

        elif key == 'f':
            self.cmd.linear.z = -self.speed_z

        elif key == 'q':
            self.cmd.angular.z = self.speed_yaw

        elif key == 'e':
            self.cmd.angular.z = -self.speed_yaw

        elif key == ' ':
            self.cmd = Twist()   # full stop

        elif key == '\x03':  # Ctrl+C
            self.destroy_node()
            rclpy.shutdown()
            return

        self.pub.publish(self.cmd)


def main():
    rclpy.init()
    node = KeyboardEEVel()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
