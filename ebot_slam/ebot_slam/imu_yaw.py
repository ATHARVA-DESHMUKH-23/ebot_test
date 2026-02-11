#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import smbus
import math
import time

from ebot_slam.kalman import KalmanAngle  

MPU_ADDR = 0x68
ACCEL_XOUT = 0x3B
GYRO_XOUT = 0x43
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B

ACCEL_SCALE = 16384.0          # ±2g
GYRO_SCALE = 131.0             # ±250 deg/s


class MPU6050EKF(Node):
    def __init__(self):
        super().__init__('imu_ekf')

        self.bus = smbus.SMBus(1)

        # Wake up MPU
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # Force gyro to ±250°/s
        self.bus.write_byte_data(MPU_ADDR, GYRO_CONFIG, 0x00)

        self.roll_kf = KalmanAngle()
        self.pitch_kf = KalmanAngle()

        self.last_time = self.get_time()
        self.yaw = 0.0

        self.gz_bias = 0.0
        self.calibrate_gyro()

        self.pub = self.create_publisher(Float64MultiArray, '/imu/rpy_deg', 10)

        # True 50 Hz
        self.timer = self.create_timer(0.02, self.update)

    # -----------------------------
    def get_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    # -----------------------------
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

    # -----------------------------
    def read_word(self, reg):
        h = self.bus.read_byte_data(MPU_ADDR, reg)
        l = self.bus.read_byte_data(MPU_ADDR, reg + 1)
        val = (h << 8) | l
        return val - 65536 if val > 32767 else val

    # -----------------------------
    def update(self):
        now = self.get_time()
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0 or dt > 0.1:
            return

        # ---- Accelerometer ----
        ax_raw = self.read_word(ACCEL_XOUT) / ACCEL_SCALE
        ay_raw = self.read_word(ACCEL_XOUT + 2) / ACCEL_SCALE
        az_raw = self.read_word(ACCEL_XOUT + 4) / ACCEL_SCALE

        ax = az_raw
        ay = ay_raw
        az = -ax_raw

        # ---- Gyroscope ----
        gx = self.read_word(GYRO_XOUT) / GYRO_SCALE
        gy = self.read_word(GYRO_XOUT + 2) / GYRO_SCALE
        gz = self.read_word(GYRO_XOUT + 4) / GYRO_SCALE

        gz_corrected = gz - self.gz_bias

        # ---- Roll & Pitch ----
        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(
            math.atan2(-ax, math.sqrt(ay * ay + az * az))
        )

        roll = self.roll_kf.update(roll_acc, gx, dt)
        pitch = self.pitch_kf.update(pitch_acc, gy, dt)

        # ---- Yaw Integration ----
        self.yaw += gz_corrected * dt

        # Wrap yaw to [-180, 180]
        self.yaw = (self.yaw + 180) % 360 - 180

        # ---- Publish ----
        msg = Float64MultiArray()
        msg.data = [roll, pitch, self.yaw]
        self.pub.publish(msg)

        self.get_logger().info(
            f"ROLL={roll:.2f}°, "
            f"PITCH={pitch:.2f}°, "
            f"YAW={self.yaw:.2f}°"
        )


def main():
    rclpy.init()
    node = MPU6050EKF()
    rclpy.spin(node)
    rclpy.shutdown()
