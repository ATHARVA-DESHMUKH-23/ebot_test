#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from geometry_msgs.msg import PoseStamped
import threading
import time

STEP_POS = 0.01  # meters


class EndEffectorTeleop(Node):
    def __init__(self):
        super().__init__("ee_teleop")

        # ---- MoveIt2 Interface ----
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
            ],
            base_link_name="base_link",
            end_effector_name="ee_link",
            group_name="arm",
        )

        # ---- Spin in background thread ----
        self.executor_thread = threading.Thread(
            target=rclpy.spin,
            args=(self,),
            daemon=True
        )
        self.executor_thread.start()

        self.get_logger().info("End-effector teleop ready")

    def move_delta_x(self, dx):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()

        # Simple forward motion
        pose.pose.position.x = 0.3 + dx
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.3

        # Neutral orientation (valid quaternion)
        pose.pose.orientation.w = 1.0

        self.moveit2.move_to_pose(pose.pose)
        self.moveit2.wait_until_executed()


def main():
    rclpy.init()
    node = EndEffectorTeleop()

    time.sleep(1.0)  # let MoveIt connect
    node.move_delta_x(STEP_POS)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
