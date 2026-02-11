#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import math
import numpy as np

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, Float64MultiArray

import tf_transformations


class ScanTo3D(Node):

    def __init__(self):
        super().__init__('scan_to_3d')

        # Subscribers
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10)

        self.create_subscription(
            Float64MultiArray, '/imu/rpy_deg', self.rpy_cb, 10)

        # Publisher
        self.pc_pub = self.create_publisher(
            PointCloud2, '/scan_3d/cloud', 10)

        self.roll = 0.0
        self.pitch = 0.0

        self.get_logger().info("ScanTo3D started (IMU pitch enabled)")

    def rpy_cb(self, msg):
        # msg.data = [roll, pitch, yaw] in degrees
        self.roll  = math.radians(msg.data[0])
        self.pitch = -math.radians(msg.data[1])

    def scan_cb(self, scan):
        angle = scan.angle_min
        points = []

        # Pitch rotation (around Y axis)
        R_pitch = tf_transformations.rotation_matrix(
            self.pitch, (0, 1, 0)
        )

        for r in scan.ranges:
            if not (scan.range_min < r < scan.range_max):
                angle += scan.angle_increment
                continue

            # 2D LiDAR point
            p = np.array([
                r * math.cos(angle),
                r * math.sin(angle),
                0.0,
                1.0
            ])

            # Apply pitch
            p_rot = R_pitch @ p
            points.append(p_rot[:3].tolist())

            angle += scan.angle_increment

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "laser"

        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(cloud)


def main():
    rclpy.init()
    node = ScanTo3D()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
