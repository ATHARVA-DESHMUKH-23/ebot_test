#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np


class CartesianServo(Node):

    def __init__(self):
        super().__init__('cartesian_servo')

        # Link lengths
        self.L1 = 0.30
        self.L2 = 0.25

        self.dt = 0.01  # 100 Hz

        # Joint vector: [base, shoulder, elbow, wrist]
        self.q = np.zeros(4)
        self.received_js = False

        # Desired Cartesian velocity [x, y, z, yaw]
        self.xdot = np.zeros(4)

        self.max_qdot = 1.2
        self.lambda_dls = 0.02

        # Joint jog
        self.jog_qdot = np.zeros(4)
        self.jog_active = False
        self.jog_timeout = 5.0  # seconds
        self.last_jog_time = self.get_clock().now()


        # INIT
        self.mode = "INIT"
        self.init_sent = False
        self.start_q = np.array([0.0, -0.6, 1.8, 0.0])
        self.pos_tol = 0.02

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(Twist, '/ee_velocity_arm', self.ee_vel_cb, 10)
        self.create_subscription(JointState,'/joint_jog_arm',self.joint_jog_cb,10)


        # Publisher
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("3D Cartesian Servo started")

    # ----------------------------
    # Callbacks
    # ----------------------------
    def joint_jog_cb(self, msg):
        # # Reset jog velocity
        # self.jog_qdot[:] = 0.0

        for i, name in enumerate(msg.name):
            if name == 'base_rotation_joint':
                self.jog_qdot[0] = msg.velocity[i]
            elif name == 'shoulder_joint':
                self.jog_qdot[1] = msg.velocity[i]
            elif name == 'elbow_joint':
                self.jog_qdot[2] = msg.velocity[i]
            elif name == 'wrist_joint':
                self.jog_qdot[3] = msg.velocity[i]

        self.jog_active = True
        self.last_jog_time = self.get_clock().now()

    def joint_state_cb(self, msg):
        try:
            self.q[0] = msg.position[msg.name.index('base_rotation_joint')]
            self.q[1] = msg.position[msg.name.index('shoulder_joint')]
            self.q[2] = msg.position[msg.name.index('elbow_joint')]
            self.q[3] = msg.position[msg.name.index('wrist_joint')]
            self.received_js = True
        except ValueError:
            pass

    def ee_vel_cb(self, msg):
        self.xdot[0] = msg.linear.x *50
        self.xdot[1] = msg.linear.y *50
        self.xdot[2] = msg.linear.z *50
        self.xdot[3] = msg.angular.z *50

    # ----------------------------
    # Forward Kinematics (3D)
    # ----------------------------
    def forward_kinematics(self, q):
        q0, q1, q2, _ = q

        r = self.L1*np.cos(q1) + self.L2*np.cos(q1 + q2)

        x = np.cos(q0) * r
        y = np.sin(q0) * r
        z = self.L1*np.sin(q1) + self.L2*np.sin(q1 + q2)

        return np.array([x, y, z])

    # ----------------------------
    # Jacobian (4x4)
    # ----------------------------
    def jacobian(self, q):
        q0, q1, q2, _ = q

        r = self.L1*np.cos(q1) + self.L2*np.cos(q1 + q2)
        dr1 = -self.L1*np.sin(q1) - self.L2*np.sin(q1 + q2)
        dr2 = -self.L2*np.sin(q1 + q2)

        J = np.array([
            [-np.sin(q0)*r,  np.cos(q0)*dr1,  np.cos(q0)*dr2, 0.0],
            [ np.cos(q0)*r,  np.sin(q0)*dr1,  np.sin(q0)*dr2, 0.0],
            [          0.0,  self.L1*np.cos(q1) + self.L2*np.cos(q1 + q2),
                                   self.L2*np.cos(q1 + q2), 0.0],
            [          1.0,                0.0,                0.0, 1.0]
        ])

        return J

    # ----------------------------
    # Trajectory
    # ----------------------------
    def send_trajectory(self, q_cmd, duration):
        traj = JointTrajectory()
        traj.joint_names = [
            'base_rotation_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_joint'
        ]

        pt = JointTrajectoryPoint()
        pt.positions = q_cmd.tolist()
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration % 1.0) * 1e9)

        traj.points.append(pt)
        self.traj_pub.publish(traj)

    # ----------------------------
    # Control Loop
    # ----------------------------
    def control_loop(self):
        if not self.received_js:
            return

        if self.mode == "INIT":
            if not self.init_sent:
                self.send_trajectory(self.start_q, 2.0)
                self.init_sent = True
                return

            if np.linalg.norm(self.start_q - self.q) < self.pos_tol:
                self.mode = "SERVO"
                self.get_logger().info("Entered SERVO mode")
            return
        
        # ----------------------------
        # SERVO / JOG arbitration
        # ----------------------------

        # Check jog timeout
        dt_jog = (self.get_clock().now() - self.last_jog_time).nanoseconds * 1e-9
        if dt_jog > self.jog_timeout:
            self.jog_active = False

        # ----------------------------
        # JOINT JOG HAS PRIORITY
        # ----------------------------
        if self.jog_active:
            qdot = np.clip(self.jog_qdot, -self.max_qdot, self.max_qdot)
            q_cmd = self.q + qdot * self.dt
            self.send_trajectory(q_cmd, self.dt)
            return

        # SERVO
        J = self.jacobian(self.q)

        qdot = J.T @ np.linalg.inv(
            J @ J.T + self.lambda_dls * np.eye(4)
        ) @ self.xdot

        qdot = np.clip(qdot, -self.max_qdot, self.max_qdot)

        q_cmd = self.q + qdot * self.dt
        self.send_trajectory(q_cmd, self.dt)


def main():
    rclpy.init()
    node = CartesianServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
