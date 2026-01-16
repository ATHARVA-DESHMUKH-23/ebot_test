#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np


class CartesianServo(Node):

    def __init__(self):
        super().__init__('cartesian_servo')

        # Link lengths
        self.L1 = 0.30
        self.L2 = 0.25

        self.dt = 0.01  # 100 Hz

        # Joint state
        self.q = np.zeros(3)
        self.received_js = False

        # Desired EE velocity [ẋ, ẏ, θ̇]
        self.xdot = np.zeros(3)
        self.last_cmd_time = self.get_clock().now()

        # Limits
        self.max_qdot = 1.0  # rad/s
        self.lambda_dls = 0.02

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(Twist, '/ee_velocity', self.ee_vel_cb, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_position_controller/commands',
            10
        )

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Cartesian Servo (3-DOF planar) started")

    def joint_state_cb(self, msg):
        try:
            self.q[0] = msg.position[msg.name.index('shoulder_joint')]
            self.q[1] = msg.position[msg.name.index('elbow_joint')]
            self.q[2] = msg.position[msg.name.index('wrist_joint')]
            self.received_js = True
        except ValueError:
            pass

    def ee_vel_cb(self, msg):
        self.xdot[0] = msg.linear.x
        self.xdot[1] = msg.linear.y
        self.xdot[2] = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def jacobian(self, q):
        q1, q2, q3 = q
        return np.array([
            [-self.L1*np.sin(q1) - self.L2*np.sin(q1+q2),
             -self.L2*np.sin(q1+q2),
             0.0],

            [ self.L1*np.cos(q1) + self.L2*np.cos(q1+q2),
              self.L2*np.cos(q1+q2),
              0.0],

            [1.0, 1.0, 1.0]
        ])

    def control_loop(self):
        if not self.received_js:
            return

        # Dead-man timeout (200 ms)
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds > 200_000_000:
            self.xdot[:] = 0.0

        J = self.jacobian(self.q)

        # Damped least squares
        JT = J.T
        qdot = JT @ np.linalg.inv(J @ JT + self.lambda_dls * np.eye(3)) @ self.xdot

        # Velocity limit
        qdot = np.clip(qdot, -self.max_qdot, self.max_qdot)

        # Integrate
        q_cmd = self.q + qdot * self.dt

        msg = Float64MultiArray()
        msg.data = q_cmd.tolist()
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = CartesianServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
