#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import tf_transformations
import math

class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update)

        # Pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Speeds
        self.linear_speed = 0.10      # m/s (slow)
        self.angular_speed = 0.30     # rad/s (slow turn)

        # State machine
        self.state = 'FORWARD'
        self.state_time = 0.0

        # Timings (seconds)
        self.forward_time = 4.0
        self.rotate_time = 3.0
        self.pause_time = 1.0

    def update(self):
        self.state_time += self.dt

        # -------- STATE LOGIC --------
        v = 0.0
        w = 0.0

        if self.state == 'FORWARD':
            v = self.linear_speed
            if self.state_time >= self.forward_time:
                self._next_state('PAUSE1')

        elif self.state == 'PAUSE1':
            if self.state_time >= self.pause_time:
                self._next_state('ROTATE')

        elif self.state == 'ROTATE':
            w = self.angular_speed
            if self.state_time >= self.rotate_time:
                self._next_state('PAUSE2')

        elif self.state == 'PAUSE2':
            if self.state_time >= self.pause_time:
                self._next_state('FORWARD')

        # -------- INTEGRATE MOTION --------
        self.x += v * math.cos(self.yaw) * self.dt
        self.y += v * math.sin(self.yaw) * self.dt
        self.yaw += w * self.dt

        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)

        # -------- TF --------
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )
        self.tf_broadcaster.sendTransform(t)

        # -------- ODOM MSG --------
        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation

        self.odom_pub.publish(odom)

    def _next_state(self, new_state):
        self.state = new_state
        self.state_time = 0.0

def main():
    rclpy.init()
    node = FakeOdom()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
