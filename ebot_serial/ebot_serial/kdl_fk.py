#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL as kdl


class KDLFKTest(Node):

    def __init__(self):
        super().__init__('kdl_fk_test')

        # ------------------------------------------------
        # Declare + fetch robot_description
        # ------------------------------------------------
        self.declare_parameter('robot_description', '')

        robot_desc = (
            self.get_parameter('robot_description')
            .get_parameter_value()
            .string_value
        )

        if not robot_desc:
            self.get_logger().error("❌ robot_description empty")
            return

        self.get_logger().info("✅ robot_description received")

        # ------------------------------------------------
        # Parse URDF → KDL tree
        # ------------------------------------------------
        robot = URDF.from_xml_string(robot_desc)

        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            self.get_logger().error("❌ Failed to build KDL tree")
            return

        # IMPORTANT: must match TF frames exactly
        base_link = 'arm_base_link'
        tip_link = 'tool0'

        if not tree.getNrOfSegments():
            self.get_logger().error("❌ Empty KDL tree")
            return

        chain = tree.getChain(base_link, tip_link)

        self.get_logger().info(
            f"✅ KDL chain created: {base_link} -> {tip_link}"
        )
        self.get_logger().info(
            f"✅ Number of joints: {chain.getNrOfJoints()}"
        )

        # ------------------------------------------------
        # FK sanity check
        # ------------------------------------------------
        self.test_fk(chain)

    # ------------------------------------------------
    # FK test
    # ------------------------------------------------
    def test_fk(self, chain):
        fk_solver = kdl.ChainFkSolverPos_recursive(chain)

        q = kdl.JntArray(chain.getNrOfJoints())
        for i in range(chain.getNrOfJoints()):
            q[i] = 0.0   # zero configuration

        frame = kdl.Frame()
        fk_solver.JntToCart(q, frame)

        self.get_logger().info(
            f"🧮 KDL FK result:"
        )
        self.get_logger().info(
            f"    x = {frame.p.x():.3f}"
        )
        self.get_logger().info(
            f"    y = {frame.p.y():.3f}"
        )
        self.get_logger().info(
            f"    z = {frame.p.z():.3f}"
        )


def main():
    rclpy.init()
    node = KDLFKTest()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
