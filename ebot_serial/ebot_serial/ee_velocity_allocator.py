#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np


class EEVelocityAllocator(Node):

    def __init__(self):
        super().__init__('ee_velocity_allocator')

        # ---- Arm geometry ----
        self.L1 = 0.30
        self.L2 = 0.25

        # ---- Workspace limits ----
        self.r_max = 0.50          # hard limit
        self.soft_band = 0.08
        self.r_soft_start = self.r_max - self.soft_band

        # ---- State ----
        self.q = np.zeros(3)
        self.ee_cmd = Twist()
        self.received_js = False

        # ---- Subscribers ----
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10
        )

        self.create_subscription(
            Twist, '/ee_velocity', self.ee_cmd_cb, 10
        )

        # ---- Publishers ----
        self.ee_arm_pub = self.create_publisher(
            Twist, '/ee_velocity_arm', 10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info("EE Velocity Allocator started")

    # --------------------------------------------------

    def joint_state_cb(self, msg):
        try:
            self.q[0] = msg.position[msg.name.index('shoulder_joint')]
            self.q[1] = msg.position[msg.name.index('elbow_joint')]
            self.q[2] = msg.position[msg.name.index('wrist_joint')]
            self.received_js = True
        except ValueError:
            pass

    def ee_cmd_cb(self, msg):
        self.ee_cmd = msg

    # --------------------------------------------------

    def forward_kinematics(self):
        q1, q2, _ = self.q
        x = self.L1*np.cos(q1) + self.L2*np.cos(q1+q2)
        y = self.L1*np.sin(q1) + self.L2*np.sin(q1+q2)
        return np.array([x, y])

    # --------------------------------------------------

    def compute_scale(self, r):
        """
        Returns scale ∈ [0,1] for arm contribution
        """
        if r <= self.r_soft_start:
            return 1.0
        elif r >= self.r_max:
            return 0.0
        else:
            return (self.r_max - r) / (self.r_max - self.r_soft_start)

    # --------------------------------------------------

    def update(self):
        if not self.received_js:
            return

        ee_pos = self.forward_kinematics()
        r = np.linalg.norm(ee_pos)

        scale = self.compute_scale(r)

        # ---- Desired EE velocity (odom frame) ----
        vx = self.ee_cmd.linear.x
        vy = self.ee_cmd.linear.y
        vz = self.ee_cmd.linear.z
        wz = self.ee_cmd.angular.z

        # ---- Split velocity ----
        ee_arm = Twist()
        base = Twist()

        ee_arm.linear.x = scale * vx
        ee_arm.linear.y = scale * vy
        ee_arm.linear.z = scale * vz
        ee_arm.angular.z = scale * wz

        base.linear.y = (1.0 - scale) * vx
        base.linear.x = ((1.0 - scale) * vy) *10
        base.angular.z = (1.0 - scale) * wz

        # ---- Publish ----
        self.ee_arm_pub.publish(ee_arm)
        self.cmd_vel_pub.publish(base)

        # ---- Debug (slow rate OK) ----
        # self.get_logger().info(
        #     f"r={r:.3f} | scale={scale:.2f} | "
        #     f"arm=({ee_arm.linear.y:.2f}) base=({base.linear.y:.2f})"
        # )


def main():
    rclpy.init()
    node = EEVelocityAllocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
