#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time


class EncoderOdometry(Node):
    def __init__(self):
        super().__init__('encoder_odometry')

        self.R = 0.0625
        self.L = 0.37
        self.TICKS_PER_REV = 1024

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_left = None
        self.last_right = None
        self.last_time = time.time()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(
            Float32MultiArray, '/fake_encoders', self.enc_cb, 10
        )

    def enc_cb(self, msg):
        left, right = msg.data
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if self.last_left is None:
            self.last_left = left
            self.last_right = right
            return

        dl = (left - self.last_left) * (2 * math.pi * self.R) / self.TICKS_PER_REV
        dr = (right - self.last_right) * (2 * math.pi * self.R) / self.TICKS_PER_REV

        self.last_left = left
        self.last_right = right

        dc = (dr + dl) / 2.0
        dtheta = (dr - dl) / self.L

        self.theta += dtheta
        self.x += dc * math.cos(self.theta)
        self.y += dc * math.sin(self.theta)

        self.publish(now)

    def publish(self, now):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'ebot_base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'ebot_base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)

        self.tf_pub.sendTransform(t)


def main():
    rclpy.init()
    node = EncoderOdometry()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
