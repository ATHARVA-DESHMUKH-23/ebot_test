#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import math
import time


class EncoderOdometry(Node):
    def __init__(self):
        super().__init__('encoder_odometry')

        self.R = 0.0625
        self.L = 0.44
        self.TICKS_PER_REV = 1024

        self.last_left = None
        self.last_right = None
        self.last_time = time.time()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(
            Float32MultiArray, '/fake_encoders', self.enc_cb, 10
        )

    def enc_cb(self, msg):
        left, right = msg.data
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if self.last_left is None or dt <= 0.0:
            self.last_left = left
            self.last_right = right
            return

        # Tick difference → distance
        dl = (left - self.last_left) * (2 * math.pi * self.R) / self.TICKS_PER_REV
        dr = (right - self.last_right) * (2 * math.pi * self.R) / self.TICKS_PER_REV

        self.last_left = left
        self.last_right = right

        # Differential drive
        dc = (dr + dl) / 2.0
        dtheta = (dr - dl) / self.L

        v = dc / dt
        w = dtheta / dt

        self.publish(v, w)

    def publish(self, v, w):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'ebot_base_link'

        # EKF will compute pose → leave pose identity
        odom.pose.pose.orientation.w = 1.0

        # Provide only velocities
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        # Covariance: yaw is unreliable
        odom.twist.covariance[0] = 0.05    # vx
        odom.twist.covariance[35] = 5.0    # yaw rate

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = EncoderOdometry()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
