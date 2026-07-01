#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>

geometry_msgs::msg::Twist current_vel;
std::mutex vel_mutex;

// ============================================================
// Velocity Callback
// ============================================================

void velocityCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(vel_mutex);

  current_vel = *msg;
}

// ============================================================
// MAIN
// ============================================================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "fake_servo_node",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
  );

  bool use_base = true;
  

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node);

  std::thread([&executor]()
  {
    executor.spin();
  }).detach();

  // ==========================================================
  // MoveIt
  // ==========================================================

  moveit::planning_interface::MoveGroupInterface move_group(
    node,
    "arm");

  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(1.0);
  while (!move_group.getCurrentState(1.0))
  {
    RCLCPP_WARN(
      node->get_logger(),
      "Waiting for current robot state...");
  }

  // ==========================================================
  // Subscriber
  // ==========================================================

  auto vel_sub =
    node->create_subscription<geometry_msgs::msg::Twist>(
      "/ee_velocity",
      10,
      velocityCallback);
  
  auto cmd_vel_pub =
    node->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10);
  // ==========================================================
  // Trajectory Publisher
  // ==========================================================

  auto traj_pub =
    node->create_publisher<
      trajectory_msgs::msg::JointTrajectory>(
        "/arm_controller/joint_trajectory",
        10);

  // ==========================================================
  // Servo Loop
  // ==========================================================

  const double dt = 0.03;

  rclcpp::Rate rate(1.0 / dt);

  RCLCPP_INFO(node->get_logger(), "Fake Servo Started");

  while (rclcpp::ok())
  {

    geometry_msgs::msg::Twist vel;

    {
      std::lock_guard<std::mutex> lock(vel_mutex);
      vel = current_vel;
      RCLCPP_INFO_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        500,

        "VEL | "
        "LIN[x=%.3f y=%.3f z=%.3f] "
        "ANG[x=%.3f y=%.3f z=%.3f]",

        vel.linear.x,
        vel.linear.y,
        vel.linear.z,

        vel.angular.x,
        vel.angular.y,
        vel.angular.z);
        
      bool no_motion =
        std::abs(vel.linear.x) < 0.0001 &&
        std::abs(vel.linear.y) < 0.0001 &&
        std::abs(vel.linear.z) < 0.0001 &&
        std::abs(vel.angular.x) < 0.0001 &&
        std::abs(vel.angular.y) < 0.0001 &&
        std::abs(vel.angular.z) < 0.0001;

      if (no_motion)
      {
        geometry_msgs::msg::Twist stop_cmd;

        cmd_vel_pub->publish(stop_cmd);

        RCLCPP_INFO_THROTTLE(
          node->get_logger(),
          *node->get_clock(),
          1000,
          "Publishing base stop");

        // rclcpp::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
    }

    // --------------------------------------------------------
    // Current Pose
    // --------------------------------------------------------

    geometry_msgs::msg::Pose current_pose =
      move_group.getCurrentPose().pose;

    geometry_msgs::msg::Pose target_pose =
      current_pose;

    // --------------------------------------------------------
    // Integrate Velocity
    // --------------------------------------------------------

    target_pose.position.x += vel.linear.x * dt;
    target_pose.position.y += vel.linear.y * dt;
    target_pose.position.z += vel.linear.z * dt;

    // --------------------------------------------------------
    // Orientation Integration
    // --------------------------------------------------------

    tf2::Quaternion q_current;

    tf2::fromMsg(
      current_pose.orientation,
      q_current);

    // Small rotation from angular velocity

    tf2::Quaternion q_delta;

    q_delta.setRPY(
      vel.angular.x * dt,
      vel.angular.y * dt,
      vel.angular.z * dt);

    // Apply rotation increment

    tf2::Quaternion q_target =
      q_delta * q_current;

    q_target.normalize();

    target_pose.orientation =
      tf2::toMsg(q_target);

    // --------------------------------------------------------
    // Current State
    // --------------------------------------------------------

    moveit::core::RobotStatePtr current_state =
      move_group.getCurrentState(1.0);

    if (!current_state)
    {
      RCLCPP_WARN(node->get_logger(), "No current state");
      rate.sleep();
      continue;
    }

    const moveit::core::JointModelGroup*
      joint_model_group =
        current_state->getJointModelGroup("arm");

    // --------------------------------------------------------
    // IK Solve
    // --------------------------------------------------------
    
    bool found_ik =
      current_state->setFromIK(
        joint_model_group,
        target_pose,
        0.02);

    if (!found_ik)
    {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        1000,
        "IK Failed");

      // ======================================================
      // Base fallback
      // ======================================================

      bool pure_x_motion =
        std::abs(vel.linear.x) > 0.001 &&
        std::abs(vel.linear.y) < 0.001 &&
        std::abs(vel.linear.z) < 0.001 &&
        std::abs(vel.angular.x) < 0.001 &&
        std::abs(vel.angular.y) < 0.001 &&
        std::abs(vel.angular.z) < 0.001;

      if (use_base && pure_x_motion)
      {
        geometry_msgs::msg::Twist base_cmd;

        base_cmd.linear.x = vel.linear.x / 2.0;

        cmd_vel_pub->publish(base_cmd);

        RCLCPP_INFO_THROTTLE(
          node->get_logger(),
          *node->get_clock(),
          1000,
          "Using mobile base fallback");
      }

      rate.sleep();
      continue;
    }

    // --------------------------------------------------------
    // Joint Values
    // --------------------------------------------------------

    std::vector<double> joint_values;

    current_state->copyJointGroupPositions(
      joint_model_group,
      joint_values);

    std::stringstream ss;

    ss << "JOINTS | ";

    for (double j : joint_values)
    {
      ss << j << " ";
    }

    RCLCPP_INFO_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      500,
      "%s",
      ss.str().c_str());

    // --------------------------------------------------------
    // Build Trajectory
    // --------------------------------------------------------

    trajectory_msgs::msg::JointTrajectory traj;

    traj.header.stamp = node->now();

    traj.joint_names =
      joint_model_group->getVariableNames();

    trajectory_msgs::msg::JointTrajectoryPoint point;

    point.positions = joint_values;

    point.time_from_start =
      rclcpp::Duration::from_seconds(dt);

    traj.points.push_back(point);

    // --------------------------------------------------------
    // Publish
    // --------------------------------------------------------

    traj_pub->publish(traj);

    rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}