import rclpy
from rclpy.node import Node
from enum import Enum
import tf2_ros
from geometry_msgs.msg import Twist
import math


class State(Enum):
    IDLE = 0
    DETECTING = 1
    APPROACHING = 2
    DONE = 3


class ArucoFollower(Node):

    def __init__(self):
        super().__init__('aruco_follower')

        self.state = State.IDLE

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Velocity publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer loop
        self.timer = self.create_timer(0.1, self.run)

        self.marker_frame = "1039_obj"
        self.base_frame = "ebot_base_link"

    # ---------------------------
    # TF
    # ---------------------------

    def get_marker_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.base_frame,
                self.marker_frame,
                rclpy.time.Time()
            )
        except:
            return None

    # ---------------------------
    # Control
    # ---------------------------

    def compute_control(self, transform):
        twist = Twist()

        dx = transform.transform.translation.x
        dy = transform.transform.translation.y

        distance = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)

        # Gains
        k_ang = 2.0
        k_lin = 0.5

        # If angle large → rotate only
        if abs(angle) > 0.2:
            twist.angular.z = k_ang * angle
            twist.linear.x = 0.0
        else:
            twist.angular.z = k_ang * angle
            twist.linear.x = k_lin * distance

        return twist, distance, angle

    # ---------------------------
    # State Machine
    # ---------------------------

    def run(self):

        # ---- IDLE ----
        if self.state == State.IDLE:
            self.get_logger().info("Waiting for marker...")
            self.state = State.DETECTING

        # ---- DETECTING ----
        elif self.state == State.DETECTING:
            trans = self.get_marker_transform()

            if trans:
                self.get_logger().info("Marker detected → approaching")
                self.state = State.APPROACHING

        # ---- APPROACHING ----
        elif self.state == State.APPROACHING:
            trans = self.get_marker_transform()

            if not trans:
                self.get_logger().warn("Marker lost!")
                return

            twist, dist, angle = self.compute_control(trans)

            self.get_logger().info(f"Dist: {dist:.2f}, Angle: {angle:.2f}")

            # Stop condition
            if dist <= 0.4:
                self.get_logger().info("Reached target distance")

                stop = Twist()
                self.cmd_pub.publish(stop)

                self.state = State.DONE
                return

            self.cmd_pub.publish(twist)

        # ---- DONE ----
        elif self.state == State.DONE:
            self.get_logger().info("Task completed")
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    node = ArucoFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()