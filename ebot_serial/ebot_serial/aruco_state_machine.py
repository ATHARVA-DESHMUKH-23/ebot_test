import rclpy
from rclpy.node import Node
from enum import Enum
import tf2_ros
from geometry_msgs.msg import TwistStamped, PoseStamped
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

        # Publishers
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(PoseStamped, '/arm_target_pose', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.run)

        self.marker_frame = "1039_obj"
        self.base_frame = "ebot_base_link"
        self.arm_frame = "moveo_base_link"

        self.moveit_sent = False
        self.stop_time = None

    # ---------------------------
    # TF
    # ---------------------------

    def get_transform(self, target_frame):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                self.marker_frame,
                rclpy.time.Time()
            )
        except:
            return None

    # ---------------------------
    # Control
    # ---------------------------

    def compute_control(self, transform):
        dx = transform.transform.translation.x
        dy = transform.transform.translation.y

        distance = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.base_frame

        # Gains
        k_ang = 1.0
        k_lin = 0.4

        # Rotate first if needed
        if abs(angle) > 0.2:
            twist.twist.angular.z = k_ang * angle
            twist.twist.linear.x = 0.0
        else:
            twist.twist.angular.z = k_ang * angle
            twist.twist.linear.x = min(k_lin * distance, 0.3)

        return twist, distance, angle

    def stop_robot(self):
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = self.base_frame
        self.cmd_pub.publish(stop)

    # ---------------------------
    # MoveIt Target
    # ---------------------------

    def send_to_moveit(self):
        trans = self.get_transform(self.arm_frame)

        if not trans:
            self.get_logger().error("Transform to arm frame not available!")
            return

        pose = PoseStamped()
        pose.header.frame_id = self.arm_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        pose.pose.position.x = trans.transform.translation.y - 0.15  # offset
        pose.pose.position.y = -trans.transform.translation.x 
        pose.pose.position.z = trans.transform.translation.z + 0.0  # offset

        # Orientation (simple for now)
        pose.pose.orientation.w = 1.0

        self.arm_pub.publish(pose)
        self.get_logger().info(
            f"Sent to MoveIt: x={pose.pose.position.x:.2f}, "
            f"y={pose.pose.position.y:.2f}, "
            f"z={pose.pose.position.z:.2f}"
        )

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
            trans = self.get_transform(self.base_frame)

            if trans:
                self.get_logger().info("Marker detected → approaching")
                self.state = State.APPROACHING

        # ---- APPROACHING ----
        elif self.state == State.APPROACHING:
            trans = self.get_transform(self.base_frame)

            if not trans:
                self.get_logger().warn("Marker lost!")
                self.stop_robot()
                return

            twist, dist, angle = self.compute_control(trans)

            self.get_logger().info(f"Dist: {dist:.2f}, Angle: {angle:.2f}")

            if dist <= 0.6:
                self.get_logger().info("Reached target distance")

                self.stop_robot()

                self.stop_time = self.get_clock().now()
                self.state = State.DONE
                return

            self.cmd_pub.publish(twist)

        # ---- DONE ----
        elif self.state == State.DONE:

            # Wait 0.5 sec before sending to MoveIt
            if not self.moveit_sent:
                now = self.get_clock().now()
                if (now - self.stop_time).nanoseconds > 5e8:
                    self.get_logger().info("Sending target to MoveIt")
                    self.send_to_moveit()
                    self.moveit_sent = True

            else:
                self.get_logger().info("Task completed")
                self.stop_robot()
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