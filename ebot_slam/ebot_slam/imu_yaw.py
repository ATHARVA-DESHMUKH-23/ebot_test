#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import math
import time

from tf_transformations import quaternion_from_euler
from ebot_slam.kalman import KalmanAngle

MPU_ADDR = 0x68
ACCEL_XOUT = 0x3B
GYRO_XOUT = 0x43
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B

ACCEL_SCALE = 16384.0      # ±2g
GYRO_SCALE = 131.0         # ±250 deg/s


class MPU6050EKF(Node):

    def __init__(self):
        super().__init__('imu_ekf_node')

        self.bus = smbus.SMBus(1)

        # Wake MPU6050
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # Set gyro range ±250°/s
        self.bus.write_byte_data(MPU_ADDR, GYRO_CONFIG, 0x00)

        self.roll_kf = KalmanAngle()
        self.pitch_kf = KalmanAngle()

        self.last_time = self.get_time()
        self.yaw = 0.0

        self.gz_bias = 0.0
        self.calibrate_gyro()

        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

    # --------------------------------------------------
    def get_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    # --------------------------------------------------
    def calibrate_gyro(self):
        self.get_logger().info("Calibrating gyro... KEEP ROBOT STILL")

        samples = 400
        total = 0.0

        for _ in range(samples):
            gz = self.read_word(GYRO_XOUT + 4) / GYRO_SCALE
            total += gz
            time.sleep(0.005)

        self.gz_bias = total / samples
        self.get_logger().info(f"Gyro Z bias: {self.gz_bias:.6f} deg/s")

    # --------------------------------------------------
    def read_word(self, reg):
        h = self.bus.read_byte_data(MPU_ADDR, reg)
        l = self.bus.read_byte_data(MPU_ADDR, reg + 1)
        val = (h << 8) | l
        return val - 65536 if val > 32767 else val

    # --------------------------------------------------
    def update(self):

        now = self.get_time()
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0 or dt > 0.1:
            return

        # -------- Accelerometer --------
        ax_raw = self.read_word(ACCEL_XOUT) / ACCEL_SCALE
        ay_raw = self.read_word(ACCEL_XOUT + 2) / ACCEL_SCALE
        az_raw = self.read_word(ACCEL_XOUT + 4) / ACCEL_SCALE

        # axis alignment (based on your mounting)
        ax = az_raw
        ay = ay_raw
        az = -ax_raw

        # -------- Gyroscope --------
        gx = self.read_word(GYRO_XOUT) / GYRO_SCALE
        gy = self.read_word(GYRO_XOUT + 2) / GYRO_SCALE
        gz = self.read_word(GYRO_XOUT + 4) / GYRO_SCALE

        gz_corrected = gz - self.gz_bias

        # -------- Roll & Pitch from accel --------
        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(
            math.atan2(-ax, math.sqrt(ay * ay + az * az))
        )

        roll = self.roll_kf.update(roll_acc, gx, dt)
        pitch = self.pitch_kf.update(pitch_acc, gy, dt)

        # -------- Yaw from gyro integration --------
        self.yaw += gz_corrected * dt
        self.yaw = (self.yaw + 180) % 360 - 180

        # -------- Convert to quaternion --------
        qx, qy, qz, qw = quaternion_from_euler(
            math.radians(roll),
            math.radians(pitch),
            math.radians(self.yaw)
        )

        # -------- Create IMU message --------
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Orientation
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw

        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = math.radians(gx)
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz_corrected)

        # Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = ax * 9.81
        imu_msg.linear_acceleration.y = ay * 9.81
        imu_msg.linear_acceleration.z = az * 9.81

        # Covariances (tune later if needed)
        imu_msg.orientation_covariance = [
                                            0.03, 0, 0,
                                            0, 0.03, 0,
                                            0, 0, 999.0   # ignore yaw orientation
                                        ]

        imu_msg.angular_velocity_covariance = [0.02, 0, 0,
                                               0, 0.02, 0,
                                               0, 0, 0.01]

        imu_msg.linear_acceleration_covariance = [0.1, 0, 0,
                                                  0, 0.1, 0,
                                                  0, 0, 0.1]

        self.pub.publish(imu_msg)

        self.get_logger().info(
            f"Roll={roll:.2f}  Pitch={pitch:.2f}  Yaw={self.yaw:.2f}"
        )


def main():
    rclpy.init()
    node = MPU6050EKF()
    rclpy.spin(node)
    rclpy.shutdown()
