#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
import math

# ---------------- PARAMETERS ----------------
CONTROL_HZ = 20.0
POS_TOLERANCE = 0.4
YAW_TOLERANCE = math.radians(15)

KP_LINEAR = 0.4
KP_ANGULAR = 1.5

MAX_LINEAR = 0.5
MAX_ANGULAR = 1.0
# -------------------------------------------


def normalize(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class OdomController(Node):

    def __init__(self):
        super().__init__('odom_controller')

        self.pose_x = None
        self.pose_y = None
        self.pose_yaw = None

        self.goal = None
        self.goal_reached = False

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/odom_nav/goal', self.goal_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.done_pub = self.create_publisher(PoseStamped, '/odom_nav/done', 10)

        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.control_loop)

        self.get_logger().info("✅ Odom Controller running (deterministic)")

    # ---------- Callbacks ----------

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.pose_x = p.x
        self.pose_y = p.y

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.pose_yaw = math.atan2(siny, cosy)

    def goal_cb(self, msg: PoseStamped):
        self.goal = msg
        self.goal_reached = False
        self.get_logger().info("🎯 New goal received by controller")

    # ---------- Control ----------

    def control_loop(self):
        if self.goal is None or self.pose_x is None:
            return

        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y

        # yaw from quaternion (goal)
        q = self.goal.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        gyaw = math.atan2(siny, cosy)

        dx = gx - self.pose_x
        dy = gy - self.pose_y
        dist = math.hypot(dx, dy)

        angle_to_goal = math.atan2(dy, dx)
        heading_error = normalize(angle_to_goal - self.pose_yaw)
        yaw_error = normalize(gyaw - self.pose_yaw)

        cmd = Twist()

        if dist > POS_TOLERANCE:
            if abs(heading_error) > 0.15:
                cmd.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, KP_ANGULAR * heading_error))
            else:
                cmd.linear.x = min(MAX_LINEAR, KP_LINEAR * dist)
                cmd.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, KP_ANGULAR * heading_error))
        else:
            if abs(yaw_error) > YAW_TOLERANCE:
                cmd.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, KP_ANGULAR * yaw_error))
            else:
                self.cmd_pub.publish(Twist())
                self.done_pub.publish(self.goal)
                self.goal = None
                return

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = OdomController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
