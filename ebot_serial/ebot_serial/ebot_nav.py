#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time

# ------------------ Parameters ------------------

POS_TOLERANCE = 0.3
YAW_TOLERANCE = math.radians(15)

CONTROL_HZ = 20.0

KP_LINEAR  = 0.4
KP_ANGULAR = 1.5

MAX_LINEAR  = 0.5
MAX_ANGULAR = 1.0

# ------------------------------------------------

def normalize(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Ebot_Navigator(Node):
    """
    Minimal odometry-based navigator
    Call set_goal(x, y, yaw) from anywhere
    """

    def __init__(self):
        super().__init__('odom_navigator')

        self.x = None
        self.y = None
        self.yaw = None

        self.goal = None
        self.active = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.timer = self.create_timer(
            1.0 / CONTROL_HZ, self.control_loop
        )

        self.get_logger().info("Odometry Navigator ready")

    # ---------- API (Nav2-like) ----------

    def set_goal(self, x, y, yaw):
        self.goal = (x, y, yaw)
        self.active = True
        self.get_logger().info(
            f"New goal: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°"
        )

    def goal_reached(self):
        return not self.active

    # ---------- Callbacks ----------

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.x = p.x
        self.y = p.y

        siny = 2.0 * (q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    # ---------- Control Loop ----------

    def control_loop(self):
        if not self.active or self.x is None:
            return

        gx, gy, gyaw = self.goal

        dx = gx - self.x
        dy = gy - self.y

        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)

        heading_error = normalize(angle_to_goal - self.yaw)
        yaw_error     = normalize(gyaw - self.yaw)

        cmd = Twist()

        # ---- Position ----
        if dist > POS_TOLERANCE:
            cmd.angular.z = max(
                -MAX_ANGULAR,
                min(MAX_ANGULAR, KP_ANGULAR * heading_error)
            )

            if abs(heading_error) < 0.2:
                cmd.linear.x = min(MAX_LINEAR, KP_LINEAR * dist)

        # ---- Final yaw ----
        elif abs(yaw_error) > YAW_TOLERANCE:
            cmd.angular.z = max(
                -MAX_ANGULAR,
                min(MAX_ANGULAR, KP_ANGULAR * yaw_error)
            )

        # ---- Goal reached ----
        else:
            self.cmd_pub.publish(Twist())
            self.active = False
            self.get_logger().info("Goal reached")
            return

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)

    node = Ebot_Navigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
