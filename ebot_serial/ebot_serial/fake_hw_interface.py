#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import serial
import math
import time


class FakeHWInterface(Node):
    def __init__(self):
        super().__init__('fake_hw_interface')

        # ---------------- PARAMETERS ----------------
        self.WHEEL_RADIUS = 0.0625      # meters
        self.WHEEL_BASE = 0.37          # meters
        self.TICKS_PER_REV = 1024
        self.RPM_PER_UNIT = 1.47        # calibration
        self.ACC_STEP = 5               # same as Arduino
        self.SERIAL_TIMEOUT = 0.3       # seconds

        # ---------------- STATE ----------------
        self.v_cmd = 0.0
        self.w_cmd = 0.0

        self.v_current = 0.0
        self.w_current = 0.0

        self.last_serial_time = time.time()
        self.serial_v = 0.0
        self.serial_w = 0.0
        self.serial_mode = 0            # 0=RC, 1=ROS

        self.left_ticks = 0.0
        self.right_ticks = 0.0
        self.last_time = time.time()

        # ---------------- ROS ----------------
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.enc_pub = self.create_publisher(
            Float32MultiArray, '/fake_encoders', 10
        )

        # ---------------- SERIAL ----------------
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.01)
            self.get_logger().info('Serial connected')
        except:
            self.ser = None
            self.get_logger().warn('No serial connection')

        self.timer = self.create_timer(0.02, self.update)

    # ------------------------------------------------
    def cmd_cb(self, msg):
        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z

    # ------------------------------------------------
    def read_serial(self):
        if not self.ser:
            return False

        try:
            line = self.ser.readline().decode().strip()
            if not line:
                return False

            # expected: v,w,mode
            parts = line.split(',')
            if len(parts) != 3:
                return False

            self.serial_v = float(parts[0])
            self.serial_w = float(parts[1])
            self.serial_mode = int(parts[2])

            self.last_serial_time = time.time()
            return True

        except:
            return False

    # ------------------------------------------------
    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        serial_active = self.read_serial() or \
            (now - self.last_serial_time < self.SERIAL_TIMEOUT)

        # -------- SELECT SOURCE --------
        if serial_active and self.serial_mode == 0:
            v = -self.serial_v
            w = -self.serial_w
        else:
            v = self.v_cmd
            w = self.w_cmd

        # -------- ACC LIMIT (same as Arduino) --------
        self.v_current += max(min(v - self.v_current, self.ACC_STEP), -self.ACC_STEP)
        self.w_current += max(min(w - self.w_current, self.ACC_STEP), -self.ACC_STEP)

        # -------- DIFF DRIVE --------
        left_unit = self.v_current - self.w_current
        right_unit = self.v_current + self.w_current

        # -------- MOTOR UNIT → RPM --------
        left_rpm = left_unit * self.RPM_PER_UNIT
        right_rpm = right_unit * self.RPM_PER_UNIT

        # -------- RPM → LINEAR SPEED --------
        left_v = (2 * math.pi * self.WHEEL_RADIUS * left_rpm) / 60.0
        right_v = (2 * math.pi * self.WHEEL_RADIUS * right_rpm) / 60.0

        # -------- SPEED → TICKS --------
        left_ticks_delta = (left_v / (2 * math.pi * self.WHEEL_RADIUS)) \
                           * self.TICKS_PER_REV * dt
        right_ticks_delta = (right_v / (2 * math.pi * self.WHEEL_RADIUS)) \
                            * self.TICKS_PER_REV * dt

        self.left_ticks += left_ticks_delta
        self.right_ticks += right_ticks_delta

        msg = Float32MultiArray()
        msg.data = [self.left_ticks, self.right_ticks]
        self.enc_pub.publish(msg)


def main():
    rclpy.init()
    node = FakeHWInterface()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
