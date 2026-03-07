#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
import serial
import math
import time


class EbotSerialDriver(Node):

    def __init__(self):
        super().__init__('ebot_serial_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # ---------------- SERIAL ----------------
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Serial connected on {port} @ {baudrate}")
        except:
            self.ser = None
            self.get_logger().warn("Serial connection failed")

        # ---------------- CMD DATA ----------------
        self.v = 0.0
        self.w = 0.0
        self.bot_yaw = 0.0

        # ---------------- PARAMETERS ----------------
        self.WHEEL_RADIUS = 0.0625
        self.WHEEL_BASE = 0.37
        self.TICKS_PER_REV = 1024
        self.RPM_PER_UNIT = 1.3
        self.ACC_STEP = 20
        self.SERIAL_TIMEOUT = 0.3

        # ---------------- STATE ----------------
        self.v_cmd = 0.0
        self.w_cmd = 0.0

        self.v_current = 0.0
        self.w_current = 0.0

        self.last_serial_time = time.time()
        self.serial_v = 0.0
        self.serial_w = 0.0
        self.serial_mode = 0

        self.left_ticks = 0.0
        self.right_ticks = 0.0
        self.last_time = time.time()

        self.MAX_LINEAR_SPEED = 1.5
        self.MAX_ANGULAR_SPEED = (2.0 * self.MAX_LINEAR_SPEED) / self.WHEEL_BASE

        # ---------------- ROS INTERFACE ----------------
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.create_subscription(Float32, '/ebot_info/yaw', self.yaw_callback, 10)

        self.enc_pub = self.create_publisher(
            Float32MultiArray,
            '/fake_encoders',
            10
        )

        # Timers
        self.tx_timer = self.create_timer(0.05, self.send_serial)  # 20Hz
        self.update_timer = self.create_timer(0.02, self.update)   # 50Hz

        self.get_logger().info("EBot Serial Driver Started")

    # ------------------------------------------------
    # ROS CALLBACKS
    # ------------------------------------------------

    def cmd_callback(self, msg):
        self.v = -msg.linear.x
        self.w = -msg.angular.z

    def yaw_callback(self, msg):
        self.bot_yaw = msg.data

    # ------------------------------------------------
    # SERIAL WRITE (same as first node)
    # ------------------------------------------------

    def send_serial(self):

        if not self.ser:
            return

        data = f"{self.v:.2f},{self.w:.2f},{self.bot_yaw:.2f}\n"

        # print(f"Sending to Arduino: {data.strip()}")

        try:
            self.ser.write(data.encode())
        except:
            self.get_logger().warn("Serial write failed")

    # ------------------------------------------------
    # SERIAL READ (same as second node)
    # ------------------------------------------------

    def read_serial(self):

        if not self.ser:
            return False

        try:
            line = self.ser.readline().decode().strip()
            print("RX:", line)
            if not line:
                return False

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
    # MAIN UPDATE LOOP (same logic)
    # ------------------------------------------------

    def update(self):

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        serial_active = self.read_serial() or \
            (now - self.last_serial_time < self.SERIAL_TIMEOUT)

        if serial_active and self.serial_mode == 0:
            v = -self.serial_v
            w = -self.serial_w
        else:
            v = -self.serial_v
            w = -self.serial_w

        # ACC LIMIT
        self.v_current += max(min(v - self.v_current, self.ACC_STEP), -self.ACC_STEP)
        self.w_current += max(min(w - self.w_current, self.ACC_STEP), -self.ACC_STEP)

        # DIFF DRIVE
        left_unit = self.v_current - self.w_current
        right_unit = self.v_current + self.w_current

        # MOTOR UNIT → RPM
        left_rpm = left_unit * self.RPM_PER_UNIT
        right_rpm = right_unit * self.RPM_PER_UNIT

        # RPM → LINEAR SPEED
        left_v = (2 * math.pi * self.WHEEL_RADIUS * left_rpm) / 60.0
        right_v = (2 * math.pi * self.WHEEL_RADIUS * right_rpm) / 60.0

        # SPEED → TICKS
        left_ticks_delta = (
            left_v / (2 * math.pi * self.WHEEL_RADIUS)
        ) * self.TICKS_PER_REV * dt

        right_ticks_delta = (
            right_v / (2 * math.pi * self.WHEEL_RADIUS)
        ) * self.TICKS_PER_REV * dt

        self.left_ticks += left_ticks_delta
        self.right_ticks += right_ticks_delta

        msg = Float32MultiArray()
        msg.data = [self.left_ticks, self.right_ticks]

        self.enc_pub.publish(msg)


def main():

    rclpy.init()

    node = EbotSerialDriver()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()