#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import smbus
import math
import time
import csv
import os


from ebot_slam.kalman import KalmanAngle  

MPU_ADDR = 0x68
ACCEL_XOUT = 0x3B
GYRO_XOUT = 0x43
PWR_MGMT_1 = 0x6B

ACCEL_SCALE = 16384.0
GYRO_SCALE = 131.0

class MPU6050EKF(Node):
    def __init__(self):
        super().__init__('imu_ekf')

        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        self.roll_kf = KalmanAngle()
        self.pitch_kf = KalmanAngle()

        self.last_time = time.time()

        self.yaw = 0.0

        self.pub = self.create_publisher(Float64MultiArray, '/lidar_imu/rpy_deg', 10)
        self.timer = self.create_timer(0.5, self.update)  # 50 Hz

    def read_word(self, reg):
        h = self.bus.read_byte_data(MPU_ADDR, reg)
        l = self.bus.read_byte_data(MPU_ADDR, reg+1)
        val = (h << 8) | l
        return val - 65536 if val > 32767 else val

    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        ax_raw = self.read_word(ACCEL_XOUT) / ACCEL_SCALE
        ay_raw = self.read_word(ACCEL_XOUT+2) / ACCEL_SCALE
        az_raw = self.read_word(ACCEL_XOUT+4) / ACCEL_SCALE
        
        ax = az_raw
        ay = ay_raw
        az = -ax_raw


        gx = self.read_word(GYRO_XOUT) / GYRO_SCALE
        gy = self.read_word(GYRO_XOUT+2) / GYRO_SCALE
        gz = self.read_word(GYRO_XOUT+4) / GYRO_SCALE

        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

        roll = self.roll_kf.update(roll_acc, gx, dt)
        pitch = self.pitch_kf.update(pitch_acc, gy, dt)




        msg = Float64MultiArray()
        msg.data = [roll, pitch, self.yaw]
        self.pub.publish(msg)

        self.get_logger().info(
            f"ROLL={roll:.2f}°, PITCH={pitch:.2f}°"
        )

def main():
    rclpy.init()
    node = MPU6050EKF()
    rclpy.spin(node)
    rclpy.shutdown()
    

