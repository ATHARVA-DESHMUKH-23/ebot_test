import rclpy
from rclpy.node import Node
from enum import Enum
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
import math
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler


class State(Enum):
    IDLE = 0
    NAVIGATING = 1
    DETECTING = 2
    APPROACHING = 3
    MANIPULATION = 4
    DONE = 5

class ArucoFollower(Node):

    def __init__(self):
        super().__init__('aruco_follower')

        self.state = State.IDLE

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.declare_parameter("marker_id", 0)
        self.declare_parameter("command", "pick")

        self.declare_parameter("approx_x", 0.0)
        self.declare_parameter("approx_y", 0.0)
        self.declare_parameter("approx_yaw", 0.0)
        self.declare_parameter("stamped", False)
        
        self.marker_id = self.get_parameter("marker_id").value
        self.command = self.get_parameter("command").value

        self.approx_x = self.get_parameter("approx_x").value
        self.approx_y = self.get_parameter("approx_y").value
        self.approx_yaw = self.get_parameter("approx_yaw").value
        
        self.use_stamped = self.get_parameter("stamped").value

        self.marker_frame = f"aruco_{self.marker_id}"

        # Timer
        self.timer = self.create_timer(0.1, self.run)

        # self.marker_frame = "aruco_0"
        self.base_frame = "ebot_base_link"
        self.arm_frame = "moveo_base_link"

        # Publishers
        if self.use_stamped:
            self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        else:
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(PoseStamped, '/arm_target_pose', 10)

        self.moveit_sent = False
        self.stop_time = None

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_goal_sent = False
        self.nav_cancel_done = False

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
    # Navigation
    # ---------------------------
    def send_nav_goal(self):
        goal_msg = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = self.approx_x
        pose.pose.position.y = self.approx_y

        q = quaternion_from_euler(0, 0, self.approx_yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal_msg.pose = pose

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        self.get_logger().info("Nav2 goal sent")
        
    def goal_response_callback(self, future):
        self.nav_goal_handle = future.result()

        if not self.nav_goal_handle.accepted:
            self.get_logger().error("❌ Nav2 goal REJECTED")
            return

        self.get_logger().info("✅ Nav2 goal ACCEPTED")

        result_future = self.nav_goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("🎯 Nav2 goal reached")

    def cancel_done_callback(self, future):
        self.get_logger().info("🛑 Nav2 goal CANCELLED")
        self.nav_cancel_done = True
    # ---------------------------
    # Control
    # ---------------------------

    def compute_control(self, transform):
        dx = transform.transform.translation.x
        dy = transform.transform.translation.y

        distance = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx)

        twist = Twist()
        # twist.header.stamp = self.get_clock().now().to_msg()
        # twist.header.frame_id = self.base_frame

        # Gains
        k_ang = 1.0
        k_lin = 0.4

        # Rotate first if needed
        if abs(angle) > 0.2:
           twist.angular.z = k_ang * angle
           twist.linear.x = 0.0
        else:
           twist.angular.z = k_ang * angle
           twist.linear.x = min(k_lin * distance, 0.3)

        if self.use_stamped:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = " "
            twist_msg.twist = twist
        else:
            twist_msg = twist

        return twist_msg, distance, angle

    def stop_robot(self):
        if self.use_stamped:
            stop = TwistStamped()
            stop.header.stamp = self.get_clock().now().to_msg()
            stop.header.frame_id = self.base_frame
            stop.twist.linear.x = 0.0
            stop.twist.angular.z = 0.0
        else:
            stop = Twist()

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
        pose.pose.position.x = trans.transform.translation.y - 0.0  # offset
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
            self.get_logger().info("Starting navigation")
            self.send_nav_goal()
            self.nav_cancel_done = False
            self.state = State.NAVIGATING

        elif self.state == State.NAVIGATING:

            trans = self.get_transform(self.base_frame)

            if trans:
                if hasattr(self, 'nav_goal_handle') and self.nav_goal_handle:

                    cancel_future = self.nav_goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(self.cancel_done_callback)

                    self.get_logger().info("⏳ Cancelling Nav2...")

                    self.state = State.DETECTING
                    return
            
        # ---- DETECTING ----
        elif self.state == State.DETECTING:
            # WAIT until Nav2 is actually cancelled
            if not self.nav_cancel_done:
                self.get_logger().info("Waiting for Nav2 cancel...")
                return
            
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

            if dist <= 0.55:
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