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

        
        # HARD workspace boundary (meters)
        self.r_max = 0.50          # hard limit
        self.soft_band = 0.08       # 8 cm soft zone
        self.r_soft_start = self.r_max - self.soft_band

        self.dt = 0.01  # 100 Hz

        # Joint state
        self.q = np.zeros(3)
        self.received_js = False

        # Desired EE velocity
        self.xdot = np.zeros(3)

        # Limits
        self.max_qdot = 1.0
        self.lambda_dls = 0.02

        # Init pose
        self.mode = "INIT"
        self.init_sent = False
        self.start_q = np.array([-0.62, 2.0, 0.4])
        self.pos_tol = 0.02

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(Twist, '/ee_velocity_arm', self.ee_vel_cb, 10)

        # Trajectory publisher
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Cartesian Servo started (HARD boundary mode)")

    def joint_state_cb(self, msg):
        try:
            self.q[0] = msg.position[msg.name.index('shoulder_joint')]
            self.q[1] = msg.position[msg.name.index('elbow_joint')]
            self.q[2] = msg.position[msg.name.index('wrist_joint')]
            self.received_js = True
        except ValueError:
            pass

    def ee_vel_cb(self, msg):
        self.xdot[0] = msg.linear.x * 50
        self.xdot[1] = msg.linear.y * 50
        self.xdot[2] = msg.angular.z * 50

    def forward_kinematics(self, q):
        q1, q2, _ = q
        x = self.L1 * np.cos(q1) + self.L2 * np.cos(q1 + q2)
        y = self.L1 * np.sin(q1) + self.L2 * np.sin(q1 + q2)
        return np.array([x, y])

    def jacobian(self, q):
        q1, q2, _ = q
        return np.array([
            [-self.L1*np.sin(q1) - self.L2*np.sin(q1+q2),
             -self.L2*np.sin(q1+q2),
             0.0],
            [ self.L1*np.cos(q1) + self.L2*np.cos(q1+q2),
              self.L2*np.cos(q1+q2),
              0.0],
            [1.0, 1.0, 1.0]
        ])

    def apply_soft_workspace_limit(self, xdot, ee_pos):
        r = np.linalg.norm(ee_pos)
        if r < 1e-6:
            return xdot

        # Predict next EE position
        ee_pos_next = ee_pos + xdot[:2] * self.dt
        r_next = np.linalg.norm(ee_pos_next)

        # Only act if moving OUTWARD
        if r_next <= r:
            return xdot   # retreating or sliding → allow full speed

        # Now we KNOW we are pushing outward
        if r_next > self.r_soft_start:

            if r >= self.r_max:
                scale = 0.0
            else:
                scale = (self.r_max - r) / (self.r_max - self.r_soft_start)
                scale = np.clip(scale, 0.0, 1.0)

            xdot[:2] *= scale

        return xdot




    def send_trajectory(self, q_cmd, duration):
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_joint', 'elbow_joint', 'wrist_joint']

        point = JointTrajectoryPoint()
        point.positions = q_cmd.tolist()
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)

        traj.points.append(point)
        self.traj_pub.publish(traj)

    def control_loop(self):
        if not self.received_js:
            return

        # ---------- INIT ----------
        if self.mode == "INIT":
            if not self.init_sent:
                self.send_trajectory(self.start_q, 2.0)
                self.init_sent = True
                return

            if np.linalg.norm(self.start_q - self.q) < self.pos_tol:
                self.mode = "SERVO"
                self.get_logger().info("Entered SERVO mode")
            return

        # ---------- SERVO ----------
        # ---------- SERVO ----------
        ee_pos = self.forward_kinematics(self.q)
        r = np.linalg.norm(ee_pos)

        xdot_safe =  self.apply_soft_workspace_limit(self.xdot.copy(),ee_pos)

        if r >= self.r_max:
            # unit radial direction
            radial_dir = ee_pos / r

            # radial component of velocity
            radial_vel = np.dot(xdot_safe[:2], radial_dir)

            if radial_vel > 0.0:
                # block ONLY outward motion
                xdot_safe[:2] -= radial_vel * radial_dir
                self.get_logger().warn("At boundary → blocking outward motion")


        J = self.jacobian(self.q)
        qdot = J.T @ np.linalg.inv(J @ J.T + self.lambda_dls*np.eye(3)) @ xdot_safe
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
