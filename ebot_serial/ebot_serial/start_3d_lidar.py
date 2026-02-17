#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial
import time


class LidarControl(Node):

    def __init__(self):
        super().__init__('lidar_control_node')

        # ===== PARAMETERS =====
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 9600)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value

        # ===== SERIAL CONNECTION =====
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # allow Arduino reset
            self.get_logger().info(f"Connected to Arduino on {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        # ===== ROS SUBSCRIBER =====
        self.subscription = self.create_subscription(
            Bool,
            '/lidar_3d_enable',
            self.lidar_callback,
            10
        )

        self.get_logger().info("Lidar control node ready")

    # ===== CALLBACK =====
    def lidar_callback(self, msg: Bool):

        if self.ser is None:
            self.get_logger().error("Serial not connected")
            return

        try:
            if msg.data:
                self.ser.write(b"start_3d_lidar\n")
                self.get_logger().info("➡ 3D LiDAR START sent")
            else:
                self.ser.write(b"stop_3d_lidar\n")
                self.get_logger().info("➡ 3D LiDAR STOP sent")

        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    # ===== CLEAN SHUTDOWN =====
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial connection closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
