#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from geometry_msgs.msg import TwistStamped
import math
import time


class BaseController(Node):

    def __init__(self):
        super().__init__('base_controller')

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'base_controller/follow_joint_trajectory',
            self.execute_callback
        )

        # limits (tune later)
        self.max_linear = 0.3
        self.max_angular = 1.0

        self.get_logger().info("✅ Base controller ready")

    async def execute_callback(self, goal_handle):
        self.get_logger().info("🚀 Received base trajectory")

        traj = goal_handle.request.trajectory

        self.get_logger().info(
            f"Joint names: {traj.joint_names}"
        )

        for i, p in enumerate(traj.points):
            self.get_logger().info(
                f"Point {i}: "
                f"positions={list(p.positions)} "
                f"time={p.time_from_start.sec + p.time_from_start.nanosec*1e-9:.3f}"
            )

        if len(p.velocities) > 0:
            self.get_logger().info(
                f"velocities={list(p.velocities)}"
            )

        for i in range(len(traj.points) - 1):

            p1 = traj.points[i]
            p2 = traj.points[i + 1]

            # time difference
            dt = (p2.time_from_start.sec - p1.time_from_start.sec) + \
                 (p2.time_from_start.nanosec - p1.time_from_start.nanosec) * 1e-9

            if dt <= 0:
                continue

            # global differences
            
            dtheta = p2.positions[0] - p1.positions[0]
            dx = p2.positions[1] - p1.positions[1]
            dy = p2.positions[2] - p1.positions[2]

            # current orientation
            theta = p1.positions[0]
            self.get_logger().info(
                f"Segment {i} → {i+1}: "
                f"dx={dx:.3f} m, dy={dy:.3f} m, dtheta={dtheta:.3f} rad, dt={dt:.3f} s"
            )

            # 🔥 convert world → robot frame
            vx = (dx * math.cos(theta) + dy * math.sin(theta)) / dt
            wz = dtheta / dt

            # 🔥 clamp velocities
            vx = max(min(vx, self.max_linear), -self.max_linear)
            wz = max(min(wz, self.max_angular), -self.max_angular)
            self.get_logger().info(
                f"Publishing cmd_vel: vx={vx:.3f} m/s, wz={wz:.3f} rad/s"
            )
            # message
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = " "

            msg.twist.linear.x = vx
            msg.twist.angular.z = wz

            self.cmd_pub.publish(msg)

            time.sleep(dt)

        # stop robot
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = " "
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.angular.z = 0.0

        self.cmd_pub.publish(stop_msg)

        goal_handle.succeed()
        return FollowJointTrajectory.Result()


def main():
    rclpy.init()
    node = BaseController()
    rclpy.spin(node)


if __name__ == '__main__':
    main()