#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from geometry_msgs.msg import Twist


class CartesianServo(Node):

    def __init__(self):
        super().__init__('cartesian_servo')

        # ---- Robot parameters ----
        self.L1 = 0.30
        self.L2 = 0.25
        self.dt = 0.01   # 100 Hz

        # Joint state storage
        self.q = np.zeros(2)
        self.received_js = False

        # Subscribers / Publishers
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_position_controller/commands',
            10
        )
        # Desired end-effector velocity
        self.xdot = np.zeros(2)

        self.create_subscription(
            Twist,
            '/ee_velocity',
            self.ee_vel_cb,
            10
        )

        # Control loop
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Cartesian Servo Node Started")

    def ee_vel_cb(self, msg):
        self.xdot[0] = msg.linear.x
        self.xdot[1] = msg.linear.y


    def joint_state_cb(self, msg):
        try:
            i1 = msg.name.index('shoulder_joint')
            i2 = msg.name.index('elbow_joint')
            self.q[0] = msg.position[i1]
            self.q[1] = msg.position[i2]
            self.received_js = True
        except ValueError:
            pass

    def jacobian(self, q1, q2):
        return np.array([
            [-self.L1*np.sin(q1) - self.L2*np.sin(q1+q2),  -self.L2*np.sin(q1+q2)],
            [ self.L1*np.cos(q1) + self.L2*np.cos(q1+q2),   self.L2*np.cos(q1+q2)]
        ])

    def control_loop(self):
        if not self.received_js:
            return

        q1, q2 = self.q
        J = self.jacobian(q1, q2)

        # Damped least squares
        lam = 0.01
        JT = J.T
        qdot = JT @ np.linalg.inv(J @ JT + lam*np.eye(2)) @ self.xdot

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
