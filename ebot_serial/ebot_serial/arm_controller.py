#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_ros_planning_interface.move_group_interface import MoveGroupInterface

from tf_transformations import quaternion_from_euler


class ArmPoseCommander(Node):

    def __init__(self):
        super().__init__('arm_pose_commander')

        self.get_logger().info("Initializing MoveIt interface...")

        self.move_group = MoveGroupInterface(
            node=self,
            joint_model_group="arm"
        )

        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_max_acceleration_scaling_factor(0.3)

        self.send_pose_goal()

    def send_pose_goal(self):

        target_pose = PoseStamped()
        target_pose.header.frame_id = "ebot_base_link"
        target_pose.header.stamp = self.get_clock().now().to_msg()

        # -------- XYZ (meters) --------
        target_pose.pose.position.x = 0.35
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.30

        # -------- RPY (radians) --------
        roll  = 0.0
        pitch = 1.57
        yaw   = 0.0

        q = quaternion_from_euler(roll, pitch, yaw)

        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        self.get_logger().info("Sending pose goal...")
        success = self.move_group.move_to_pose(
            target_pose,
            end_effector_link="arm_end_effector_link"
        )

        if success:
            self.get_logger().info("Motion completed successfully.")
        else:
            self.get_logger().error("Motion failed.")


def main():
    rclpy.init()
    node = ArmPoseCommander()
    rclpy.spin_once(node, timeout_sec=5.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
