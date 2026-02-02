#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import tf2_ros
import tf_transformations


class ScanTo3D(Node):

    def __init__(self):
        super().__init__('scan_to_3d')

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pc_pub = self.create_publisher(PointCloud2, '/scan_3d/slice', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("ScanTo3D (TF-based accumulation) started")

    def scan_cb(self, scan):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                scan.header.frame_id,
                rclpy.time.Time()
            )
        except Exception:
            return

        # Build full transform matrix (rotation + translation)
        T = tf_transformations.quaternion_matrix([
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w
        ])
        T[0, 3] = tf.transform.translation.x
        T[1, 3] = tf.transform.translation.y
        T[2, 3] = tf.transform.translation.z

        points = []
        angle = scan.angle_min

        for r in scan.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += scan.angle_increment
                continue

            # Laser point (homogeneous)
            p_laser = np.array([
                r * math.cos(angle),
                r * math.sin(angle),
                0.0,
                1.0
            ])

            # Transform into map frame
            p_map = T @ p_laser
            points.append(p_map[:3].tolist())

            angle += scan.angle_increment

        self.publish_cloud(points)

    def publish_cloud(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

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
