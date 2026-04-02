import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math


class YawPublisher(Node):

    def __init__(self):
        super().__init__('yaw_publisher')

        # Subscriber
        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # Publisher
        self.yaw_pub = self.create_publisher(
            Float32,
            '/ebot_info/yaw',
            10
        )

    def odom_callback(self, msg):

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Quaternion → Yaw
        siny = 2 * (qw * qz + qx * qy)
        cosy = 1 - 2 * (qy * qy + qz * qz)

        yaw_rad = math.atan2(siny, cosy)

        # Convert to degrees
        yaw_deg = math.degrees(yaw_rad)

        # Print nicely
        # self.get_logger().info(f"Yaw: {yaw_deg:.2f}°")

        yaw_msg = Float32()
        yaw_msg.data = yaw_deg   # publish degrees instead of radians

        self.yaw_pub.publish(yaw_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YawPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()