
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <thread>
#include <rclcpp/executors/single_threaded_executor.hpp>
/**
 * @brief The main function that starts our program.
 *
 * This function sets up our ROS 2 environment and prepares it for robot control.
 *
 * @param argc The number of input arguments our program receives.
 * @param argv The list of input arguments our program receives.
 * @return int A number indicating if our program finished successfully (0) or not.
 */
int main(int argc, char * argv[])
{
  // Start up ROS 2
  rclcpp::init(argc, argv);

  // Creates a node named "hello_moveit". The node is set up to automatically
  // handle any settings (parameters) we might want to change later without editing the code.
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  
  // 🔥 ADD THIS BLOCK (RIGHT HERE)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Creates a "logger" that we can use to print out information or error messages
  // as our program runs.
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interfaces
  // These interfaces are used to plan and execute movements, set target poses,
  // and perform other motion-related tasks for each respective part of the robot.
  // The use of auto allows the compiler to automatically deduce the type of variable.
  // Source: https://github.com/moveit/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "arm");

  // Specify a planning pipeline to be used for further planning
  arm_group_interface.setPlanningPipelineId("ompl");

  // Specify a planner to be used for further planning
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");

  // Specify the maximum amount of time in seconds to use when planning
  arm_group_interface.setStartStateToCurrentState();
  arm_group_interface.setPlanningTime(5.0);
  arm_group_interface.setNumPlanningAttempts(10);

  // Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1].
  arm_group_interface.setMaxVelocityScalingFactor(1.0);

  //  Set a scaling factor for optionally reducing the maximum joint acceleration. Allowed values are in (0,1].
  arm_group_interface.setMaxAccelerationScalingFactor(1.0);

  // Display helpful logging messages on the terminal
  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());
  while (!arm_group_interface.getCurrentState(1.0)) {
    RCLCPP_WARN(logger, "Waiting for current robot state...");
  }
  auto pose = arm_group_interface.getCurrentPose();
  RCLCPP_INFO(logger, "Current: x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f w=%.3f",
    pose.pose.position.x,
    pose.pose.position.y,
    pose.pose.position.z,
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w);
  auto current_pose = arm_group_interface.getCurrentPose();
  // Set a target pose for the end effector of the arm
  // auto const arm_target_pose = [&node]{
  //   geometry_msgs::msg::PoseStamped msg;
  //   msg.header.frame_id = "moveo_base_link";
  //   msg.header.stamp = node->now();
  //   msg.pose.position.x = 0.061;
  //   msg.pose.position.y = -0.176;
  //   msg.pose.position.z = 0.168;
  //   msg.pose.orientation.x = 1.0;
  //   msg.pose.orientation.y = 0.0;
  //   msg.pose.orientation.z = 0.0;
  //   msg.pose.orientation.w = 0.0;
  //   return msg;
  // }();
  double x, y, z;

  node->get_parameter("target_x", x);
  node->get_parameter("target_y", y);
  node->get_parameter("target_z", z);

  // Use the current pose as a starting point and modify it to create the target pose
  geometry_msgs::msg::Pose target_pose = current_pose.pose;

  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;
  // target_pose.orientation.x = -0.455;
  // target_pose.orientation.y = 0.456;
  // target_pose.orientation.z = 0.541;
  // target_pose.orientation.w = 0.541;
  // target_pose.orientation.x = -0.631;
  // target_pose.orientation.y = 0.764;
  // target_pose.orientation.z = -0.041;
  // target_pose.orientation.w = -0.126;
  RCLCPP_INFO(logger, "Target received: x=%.3f y=%.3f z=%.3f", x, y, z);
  // arm_group_interface.setPositionTarget(
  //   x, y, z
  // );
  arm_group_interface.setGoalOrientationTolerance(0.14159); // ~8 degrees
  arm_group_interface.setPoseTarget(target_pose);
// x=0.019 y=0.001 z=0.638

  // Create a plan to that target pose
  // This will give us two things:
  // 1. Whether the planning was successful (stored in 'success')
  // 2. The actual motion plan (stored in 'plan')
  auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Try to execute the movement plan if it was created successfully
  // If the plan wasn't successful, report an error
  // Execute the plan
  if (success)
  {
    arm_group_interface.execute(plan);
  }
    else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shut down ROS 2 cleanly when we're done
  rclcpp::shutdown();
  return 0;
}