#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import math
import time

MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43
ACCEL_XOUT_H = 0x3B

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x80)
        time.sleep(0.1)

        # Wake up and use PLL
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x01)
        time.sleep(0.1)

        self.pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.timer = self.create_timer(0.01, self.publish_imu)
        self.gyro_bias_z = 0.0
        self.calibrated = False
        self.samples = []


        self.get_logger().info("IMU node started")

    def read_word(self, reg):
        high = self.bus.read_byte_data(MPU_ADDR, reg)
        low = self.bus.read_byte_data(MPU_ADDR, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    def publish_imu(self):
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"

        ax = self.read_word(ACCEL_XOUT_H) / 16384.0
        ay = self.read_word(ACCEL_XOUT_H + 2) / 16384.0
        az = self.read_word(ACCEL_XOUT_H + 4) / 16384.0

        gx_raw = self.read_word(GYRO_XOUT_H) / 131.0 * math.pi / 180.0
        gy_raw = self.read_word(GYRO_XOUT_H + 2) / 131.0 * math.pi / 180.0
        gz_raw = self.read_word(GYRO_XOUT_H + 4) / 131.0 * math.pi / 180.0
            # ---- Calibration phase ----
        if not self.calibrated:
            self.samples.append(gz_raw)
            if len(self.samples) >= 300:  # ~3 seconds at 100 Hz
                self.gyro_bias_z = sum(self.samples) / len(self.samples)
                self.calibrated = True
                self.get_logger().info(f"Gyro Z bias calibrated: {self.gyro_bias_z}")
            return

        # ---- Bias removal ----
        gz = gz_raw - self.gyro_bias_z  
            
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        imu.angular_velocity.x = gx_raw
        imu.angular_velocity.y = gy_raw
        imu.angular_velocity.z = gz_raw - self.gyro_bias_z
                
                # Orientation not provided (we are not computing quaternion)
        imu.orientation_covariance[0] = -1

        # Gyro covariance (tells SLAM gyro is usable)
        imu.angular_velocity_covariance[0] = 0.02

        # Accel covariance
        imu.linear_acceleration_covariance[0] = 0.04

        self.pub.publish(imu)

def main():
    rclpy.init()
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
