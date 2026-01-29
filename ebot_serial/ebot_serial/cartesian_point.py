#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import PyKDL as kdl

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class CartesianPointController(Node):

    def __init__(self):
        super().__init__('cartesian_point_controller')

        # ---- Control params ----
        self.dt = 0.02
        self.lambda_dls = 0.02
        self.max_qdot = 1.0
        self.pos_tol = 0.01      # meters
        self.kp = 2.0            # Cartesian gain ⭐

        # ---- State ----
        self.q = np.zeros(4)
        self.q_received = False
        self.target = None

        # ---- KDL ----
        self.chain = self.build_kdl_chain()
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)

        # ---- ROS ----
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(Point, '/arm/cartesian_target', self.target_cb, 10)

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("Cartesian Point Controller ready")

    # ------------------------------------------------
    # KDL chain
    # ------------------------------------------------
    def build_kdl_chain(self):
        l1, l2, l3, l4 = 0.10, 0.30, 0.25, 0.05

        chain = kdl.Chain()
        chain.addSegment(kdl.Segment(
            "link1",
            kdl.Joint("base_rotation_joint", kdl.Joint.RotZ),
            kdl.Frame(kdl.Vector(0, 0, 0))
        ))
        chain.addSegment(kdl.Segment(
            "link2",
            kdl.Joint("shoulder_joint", kdl.Joint.RotY),
            kdl.Frame(kdl.Vector(0, 0, l1))
        ))
        chain.addSegment(kdl.Segment(
            "link3",
            kdl.Joint("elbow_joint", kdl.Joint.RotY),
            kdl.Frame(kdl.Vector(0, 0, l2))
        ))
        chain.addSegment(kdl.Segment(
            "link4",
            kdl.Joint("wrist_joint", kdl.Joint.RotY),
            kdl.Frame(kdl.Vector(0, 0, l3))
        ))
        chain.addSegment(kdl.Segment(
            "tool0",
            kdl.Joint(),
            kdl.Frame(kdl.Vector(0, 0, l4))
        ))
        return chain

    # ------------------------------------------------
    # Callbacks
    # ------------------------------------------------
    def joint_state_cb(self, msg):
        try:
            self.q[0] = msg.position[msg.name.index('base_rotation_joint')]
            self.q[1] = msg.position[msg.name.index('shoulder_joint')]
            self.q[2] = msg.position[msg.name.index('elbow_joint')]
            self.q[3] = msg.position[msg.name.index('wrist_joint')]
            self.q_received = True
        except ValueError:
            pass

    def target_cb(self, msg):
        self.target = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f"New target received: {self.target}")

    # ------------------------------------------------
    # Helpers
    # ------------------------------------------------
    def np_to_kdl(self, q):
        arr = kdl.JntArray(len(q))
        for i in range(len(q)):
            arr[i] = q[i]
        return arr

    def kdl_jac_to_np(self, jac):
        J = np.zeros((6, jac.columns()))
        for i in range(6):
            for j in range(jac.columns()):
                J[i, j] = jac[i, j]
        return J

    # ------------------------------------------------
    # Control loop
    # ------------------------------------------------
    def control_loop(self):
        if not self.q_received or self.target is None:
            return

        # ---- FK ----
        q_kdl = self.np_to_kdl(self.q)
        ee_frame = kdl.Frame()
        self.fk_solver.JntToCart(q_kdl, ee_frame)

        x_curr = np.array([
            ee_frame.p.x(),
            ee_frame.p.y(),
            ee_frame.p.z()
        ])
        
        self.get_logger().info(
            f"FK position: x={x_curr[0]:.3f}, y={x_curr[1]:.3f}, z={x_curr[2]:.3f}",
            throttle_duration_sec=1.0
        )



        error = self.kp * (self.target - x_curr)

        if np.linalg.norm(error) < self.pos_tol:
            return  # Target reached

        # ---- Jacobian ----
        jac_kdl = kdl.Jacobian(self.chain.getNrOfJoints())
        self.jac_solver.JntToJac(q_kdl, jac_kdl)
        J = self.kdl_jac_to_np(jac_kdl)[0:3, :]  # linear part

        # ---- DLS IK ----
        qdot = J.T @ np.linalg.inv(
            J @ J.T + self.lambda_dls * np.eye(3)
        ) @ error

        qdot = np.clip(qdot, -self.max_qdot, self.max_qdot)

        q_cmd = self.q + qdot * self.dt

        self.send_trajectory(q_cmd)

    # ------------------------------------------------
    def send_trajectory(self, q):
        traj = JointTrajectory()
        traj.joint_names = [
            'base_rotation_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_joint'
        ]

        pt = JointTrajectoryPoint()
        pt.positions = q.tolist()
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(self.dt * 1e9)

        traj.points.append(pt)
        self.traj_pub.publish(traj)


def main():
    rclpy.init()
    node = CartesianPointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
