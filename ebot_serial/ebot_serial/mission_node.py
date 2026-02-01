#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from ebot_serial.srv import SetGoal   # ✅ SERVICE, not msg

class Mission(Node):
    def __init__(self):
        super().__init__('mission_node')

        self.cli = self.create_client(SetGoal, '/ebot/set_goal')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ebot_nav service...')

        self.send_goal(1.0, 0.0, 0.0)

    def send_goal(self, x, y, yaw):
        req = SetGoal.Request()
        req.x = x
        req.y = y
        req.yaw = yaw
        self.cli.call_async(req)


def main():
    rclpy.init()
    node = Mission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
