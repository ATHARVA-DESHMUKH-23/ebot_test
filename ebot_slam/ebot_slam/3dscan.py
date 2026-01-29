#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import math
import numpy as np
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from collections import deque

class ScanTo3D(Node):

    def __init__(self):
        super().__init__('scan_to_3d')

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10)

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_cb, 10)

        # Publisher
        self.pc_pub = self.create_publisher(
            PointCloud2, '/phase1/points', 10)

        # IMU state
        # self.roll = 0.0
        self.pitch = 0.0
        
        # Motion gating thresholds
        self.GYRO_THRESHOLD = 0.02   # rad/s
        self.ACC_THRESHOLD  = 0.2    # m/s^2

        # Last stable orientation
        # self.last_roll = 0.0
        self.last_pitch = 0.0
        
        self.initialized = False
        # self.roll_offset = 0.0
        self.pitch_offset = 0.0


        # Simple complementary filter
        self.alpha = 0.98
        self.prev_time = None

        # Accumulated points
        self.points = deque(maxlen=20000)

        self.get_logger().info("Phase-1 Scan → 3D node started")

    # ---------------- IMU CALLBACK ----------------
    def imu_cb(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_time is None:
            self.prev_time = now
            return

        dt = now - self.prev_time
        self.prev_time = now

        pitch_acc = math.atan2(-az, ax)

        if not self.initialized:
            self.pitch_offset = pitch_acc
            self.initialized = True

        pitch_acc -= self.pitch_offset

        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * pitch_acc
        self.last_pitch = self.pitch



    # ---------------- LASER CALLBACK ----------------
    def scan_cb(self, msg):
        self.points.clear()

        angle = msg.angle_min

        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = 0.0   # ALWAYS ZERO IN PHASE-1

            self.points.append([x, y, z])
            angle += msg.angle_increment

        self.publish_cloud()




    # ---------------- POINTCLOUD ----------------
    def publish_cloud(self):

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world_local"

        cloud = point_cloud2.create_cloud_xyz32(
            header, self.points)

        self.pc_pub.publish(cloud)


def main():
    rclpy.init()
    node = ScanTo3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
