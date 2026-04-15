import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class SimpleScanFilter(Node):
    def __init__(self):
        super().__init__('simple_scan_filter')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',   # input topic
            self.scan_callback,
            10
        )

        self.publisher = self.create_publisher(
            LaserScan,
            '/scan_filtered',   # output topic
            10
        )

        self.threshold = 0.2  # meters

    def scan_callback(self, msg):
        filtered_msg = LaserScan()

        # Copy metadata
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max

        # Filter ranges
        filtered_ranges = []
        for r in msg.ranges:
            if r < self.threshold:
                filtered_ranges.append(float('inf'))
            else:
                filtered_ranges.append(r)

        filtered_msg.ranges = filtered_ranges
        filtered_msg.intensities = msg.intensities  # keep same

        self.publisher.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()