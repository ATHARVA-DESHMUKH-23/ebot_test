#!/usr/bin/env python3

import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from geometry_msgs.msg import PoseStamped
from ebot_navigation_interfaces.action import NavigateToPoseOdom
from rclpy.executors import MultiThreadedExecutor


class OdomNavAction(Node):

    def __init__(self):
        super().__init__('odom_nav_action_server')

        self.goal_done = False

        self.goal_pub = self.create_publisher(
            PoseStamped, '/odom_nav/goal', 10
        )

        self.done_sub = self.create_subscription(
            PoseStamped, '/odom_nav/done', self.done_cb, 10
        )

        self.server = ActionServer(
            self,
            NavigateToPoseOdom,
            'navigate_to_pose_odom',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb
        )

        self.get_logger().info("✅ Odom Action Server ready (stable)")

    # -------- ACTION CALLBACKS --------

    def goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def done_cb(self, msg):
        self.goal_done = True

    def execute_cb(self, goal_handle):
        self.goal_done = False

        # Publish goal to controller
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "odom"
        goal_msg.pose.position.x = goal_handle.request.x
        goal_msg.pose.position.y = goal_handle.request.y

        goal_msg.pose.orientation.w = math.cos(goal_handle.request.yaw / 2)
        goal_msg.pose.orientation.z = math.sin(goal_handle.request.yaw / 2)

        self.goal_pub.publish(goal_msg)
        self.get_logger().info("📤 Goal sent to controller")

        # BLOCKING WAIT (SAFE HERE)
        while rclpy.ok() and not self.goal_done:
            time.sleep(0.05)

        goal_handle.succeed()

        result = NavigateToPoseOdom.Result()
        result.success = True
        return result


def main():
    rclpy.init()
    node = OdomNavAction()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
