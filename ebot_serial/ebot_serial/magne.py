#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
import smbus2
import time
import math

class BMM150YawNode(Node):
    def __init__(self):
        super().__init__('bmm150_yaw_node')

        # ================= ROS =================
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.1, self.update)  # 10 Hz

        # ================= I2C =================
        self.BMM150_ADDR = 0x13
        self.REG_POWER   = 0x4B
        self.REG_OPMODE  = 0x4C
        self.REG_DATA    = 0x42
        self.REG_REP_XY  = 0x51
        self.REG_REP_Z   = 0x52

        self.bus = smbus2.SMBus(0)

        # ================= CALIBRATION =================
        self.OFFSET_X = 0.85
        self.OFFSET_Y = 14.85

        self.SCALE_X  = 1.008
        self.SCALE_Y  = 0.992



        # Magnetic declination (adjust for your location)
        self.MAG_DECLINATION = math.radians(1.2)

        # ================= FILTER =================
        self.ALPHA = 0.45
        self.yaw_sin = 0.0
        self.yaw_cos = 1.0

        # ================= STATE =================
        self.last_yaw = None
        self.last_time = time.time()

        self.init_bmm150()
        self.get_logger().info("BMM150 calibrated yaw EKF node running")

    # -------------------------------------------------
    def init_bmm150(self):
        self.bus.write_byte_data(self.BMM150_ADDR, self.REG_POWER, 0x00)
        time.sleep(0.01)
        self.bus.write_byte_data(self.BMM150_ADDR, self.REG_POWER, 0x01)
        time.sleep(0.01)

        self.bus.write_byte_data(self.BMM150_ADDR, self.REG_OPMODE, 0x00)
        time.sleep(0.01)

        self.bus.write_byte_data(self.BMM150_ADDR, self.REG_REP_XY, 0x04)
        self.bus.write_byte_data(self.BMM150_ADDR, self.REG_REP_Z,  0x0F)

    # -------------------------------------------------
    def read_mag(self):
        data = self.bus.read_i2c_block_data(self.BMM150_ADDR, self.REG_DATA, 8)

        x = ((data[1] << 8) | data[0]) >> 3
        y = ((data[3] << 8) | data[2]) >> 3

        if x > 4095: x -= 8192
        if y > 4095: y -= 8192

        # Hard + soft iron correction
        x = (x - self.OFFSET_X) * self.SCALE_X
        y = (y - self.OFFSET_Y) * self.SCALE_Y

        return x, y

    # -------------------------------------------------
    def filter_yaw(self, yaw):
        self.yaw_sin = self.ALPHA * self.yaw_sin + (1 - self.ALPHA) * math.sin(yaw)
        self.yaw_cos = self.ALPHA * self.yaw_cos + (1 - self.ALPHA) * math.cos(yaw)
        return math.atan2(self.yaw_sin, self.yaw_cos)

    # -------------------------------------------------
    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        mx, my = self.read_mag()

        yaw_raw = math.atan2(my, mx)
        yaw_filt = self.filter_yaw(yaw_raw)
        yaw_filt += self.MAG_DECLINATION

        # Normalize
        yaw_filt = math.atan2(math.sin(yaw_filt), math.cos(yaw_filt))

        # ---------- YAW RATE ----------
        yaw_rate = 0.0
        if self.last_yaw is not None and dt > 0.0:
            dyaw = yaw_filt - self.last_yaw
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
            yaw_rate = dyaw / dt

        self.last_yaw = yaw_filt

        # ---------- IMU MESSAGE ----------
        q = quaternion_from_euler(0.0, 0.0, yaw_filt)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'

        # Orientation (yaw only)
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]

        imu.orientation_covariance = [
            999.0, 0.0,   0.0,
            0.0,   999.0, 0.0,
            0.0,   0.0,   0.02
        ]


        # Angular velocity (yaw rate)
        imu.angular_velocity.z = yaw_rate
        imu.angular_velocity_covariance = [
            -1.0, 0.0, 0.0,
             0.0, -1.0, 0.0,
             0.0, 0.0, 0.05
        ]

        # Disable linear acceleration
        imu.linear_acceleration_covariance[0] = -1.0
        # print(f"Yaw: {math.degrees(yaw_filt):.2f} deg, Yaw Rate: {math.degrees(yaw_rate):.2f} deg/s")

        self.pub.publish(imu)

# -------------------------------------------------
def main():
    rclpy.init()
    node = BMM150YawNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
