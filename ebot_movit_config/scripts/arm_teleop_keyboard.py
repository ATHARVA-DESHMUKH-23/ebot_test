#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from moveit_commander import MoveGroupCommander, RobotCommander


class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop_keyboard')

        self.robot = RobotCommander()
        self.arm_group = MoveGroupCommander("arm")

        self.get_logger().info("Arm teleop node started")
        self.print_current_pose()

    def print_current_pose(self):
        pose = self.arm_group.get_current_pose().pose
        self.get_logger().info(
            f"Current EE Pose:\n"
            f"  Position: x={pose.position.x:.3f}, "
            f"y={pose.position.y:.3f}, "
            f"z={pose.position.z:.3f}\n"
            f"  Orientation (quat): "
            f"x={pose.orientation.x:.3f}, "
            f"y={pose.orientation.y:.3f}, "
            f"z={pose.orientation.z:.3f}, "
            f"w={pose.orientation.w:.3f}"
        )


def main():
    rclpy.init()
    node = ArmTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

