#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np


class EEStatePrinter(Node):

    def __init__(self):
        super().__init__('ee_state_printer')

        # ---- Arm parameters (MATCH YOUR MODEL) ----
        self.L1 = 0.30
        self.L2 = 0.25

        # ---- State ----
        self.q = np.zeros(3)
        self.ee_vel = Twist()
        self.received_js = False

        # ---- Subscribers ----
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        self.create_subscription(
            Twist,
            '/ee_velocity',
            self.ee_vel_cb,
            10
        )

        # ---- Timer (slow on purpose) ----
        self.timer = self.create_timer(0.2, self.print_state)  # 5 Hz

        self.get_logger().info("EE State Printer started")

    def joint_state_cb(self, msg):
        try:
            self.q[0] = msg.position[msg.name.index('shoulder_joint')]
            self.q[1] = msg.position[msg.name.index('elbow_joint')]
            self.q[2] = msg.position[msg.name.index('wrist_joint')]
            self.received_js = True
        except ValueError:
            pass

    def ee_vel_cb(self, msg):
        self.ee_vel = msg

    def forward_kinematics(self, q):
        q1, q2, _ = q
        x = self.L1 * np.cos(q1) + self.L2 * np.cos(q1 + q2)
        y = self.L1 * np.sin(q1) + self.L2 * np.sin(q1 + q2)
        return np.array([x, y])

    def print_state(self):
        if not self.received_js:
            return

        ee_pos = self.forward_kinematics(self.q)

        self.get_logger().info(
            f"EE Pos [x y] = [{ee_pos[0]:+.3f}, {ee_pos[1]:+.3f}] | "
            f"Cmd vel [vx vy] = [{self.ee_vel.linear.x:+.2f}, "
            f"{self.ee_vel.linear.y:+.2f}]"
        )


def main():
    rclpy.init()
    node = EEStatePrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
