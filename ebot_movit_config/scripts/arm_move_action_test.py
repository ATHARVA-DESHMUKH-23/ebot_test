#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from geometry_msgs.msg import PoseStamped


class ArmPoseGoalClient(Node):

    def __init__(self):
        super().__init__("arm_pose_goal_client")

        self.client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for MoveGroup action server...")
        self.client.wait_for_server()
        self.get_logger().info("MoveGroup action server available!")

    def send_pose_goal(self):

        goal = MoveGroup.Goal()
        req = goal.request

        # ---------------- Planning config ----------------
        req.group_name = "arm"
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2

        # 🚨 IMPORTANT: let MoveIt use CURRENT state
        req.start_state.is_diff = True

        # ---------------- Target pose ----------------
        pose = PoseStamped()
        pose.header.frame_id = "ebot_base_link"

        # SMALL DELTA (teleop friendly)
        pose.pose.position.x = 0.27
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.46

        pose.pose.orientation.w = 1.0

        # ---------------- Convert pose to goal constraint ----------------
        from moveit_msgs.msg import PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive

        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = "arm_end_effector_link"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]

        pc.constraint_region.primitives.append(box)
        pc.constraint_region.primitive_poses.append(pose.pose)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = "arm_end_effector_link"
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 3.14
        oc.absolute_y_axis_tolerance = 3.14
        oc.absolute_z_axis_tolerance = 3.14
        oc.weight = 0.1

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)

        req.goal_constraints.clear()
        req.goal_constraints.append(constraints)

        self.client.send_goal_async(goal)
        self.get_logger().info("Pose goal sent")


def main():
    rclpy.init()
    node = ArmPoseGoalClient()
    node.send_pose_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
