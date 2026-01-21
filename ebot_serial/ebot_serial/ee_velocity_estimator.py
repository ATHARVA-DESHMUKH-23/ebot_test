#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import tf2_ros
import numpy as np


class EEVelocityEstimator(Node):

    def __init__(self):
        super().__init__('ee_velocity_estimator')

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher
        self.pub = self.create_publisher(
            TwistStamped,
            '/ee_velocity_actual',
            10
        )

        # State
        self.prev_pos = None
        self.prev_time = None

        # Filtering (MoveIt-style)
        self.vel_filt = np.zeros(3)
        self.alpha = 0.3              # smoothing factor
        self.min_disp = 1e-4          # 0.1 mm threshold

        # Timer (match TF ~20–30 Hz)
        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info("EE velocity estimator started")

    def update(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom',
                'ee_link',
                rclpy.time.Time()
            )
        except Exception:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        pos = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z
        ])

        if self.prev_pos is None:
            self.prev_pos = pos
            self.prev_time = now
            return

        dt = now - self.prev_time
        if dt <= 0:
            return

        disp = pos - self.prev_pos

        # 🔴 Ignore tiny TF jitter
        if np.linalg.norm(disp) < self.min_disp:
            return

        vel = disp / dt

        # 🔵 Low-pass filter
        self.vel_filt = self.alpha * vel + (1.0 - self.alpha) * self.vel_filt

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ebot_base'
        msg.twist.linear.x = float(self.vel_filt[0])
        msg.twist.linear.y = float(self.vel_filt[1])
        msg.twist.linear.z = float(self.vel_filt[2])

        self.pub.publish(msg)

        self.prev_pos = pos
        self.prev_time = now


def main():
    rclpy.init()
    node = EEVelocityEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
