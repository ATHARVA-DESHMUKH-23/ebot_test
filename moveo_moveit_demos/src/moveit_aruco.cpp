#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <thread>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "moveit_aruco",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto logger = rclcpp::get_logger("moveit_aruco");

  // ---------------------------
  // Start executor (CRITICAL)
  // ---------------------------
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  // ---------------------------
  // Receive target from Python node
  // ---------------------------
  geometry_msgs::msg::PoseStamped target_pose;
  bool received = false;

  auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/arm_target_pose", 10,
    [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      target_pose = *msg;
      received = true;
      RCLCPP_INFO(logger, "Received target pose!");
    }
  );

  // ---------------------------
  // MoveIt Setup
  // ---------------------------
  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface arm_group(node, "arm");

  arm_group.setPlanningPipelineId("ompl");
  arm_group.setPlannerId("RRTConnectkConfigDefault");
  arm_group.setPlanningTime(1.5);
  arm_group.setMaxVelocityScalingFactor(0.5);
  arm_group.setMaxAccelerationScalingFactor(0.5);

  // Wait for robot state (Ctrl+C safe)
  while (rclcpp::ok() && !arm_group.getCurrentState(1.0)) {
    RCLCPP_WARN(logger, "Waiting for robot state...");
  }

  if (!rclcpp::ok()) {
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
  }

  RCLCPP_INFO(logger, "MoveIt ready. Waiting for target...");

  // ---------------------------
  // Wait for Aruco target
  // ---------------------------
  while (rclcpp::ok() && !received) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!received) {
    RCLCPP_ERROR(logger, "No target received. Exiting.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
  }

  // ---------------------------
  // Plan + Execute
  // ---------------------------
  RCLCPP_INFO(logger, "Planning to received target...");
  RCLCPP_INFO(logger, "Target: x=%.3f y=%.3f z=%.3f",
    target_pose.pose.position.x,
    target_pose.pose.position.y,
    target_pose.pose.position.z);
  
  target_pose.pose.orientation.x = -0.5;
  target_pose.pose.orientation.y = 0.5;
  target_pose.pose.orientation.z = -0.5;
  target_pose.pose.orientation.w = 0.5;
  // Set the target pose for the end effector
  arm_group.setGoalOrientationTolerance(0.44159); // 25 degrees tolerance
  arm_group.setGoalPositionTolerance(0.01);  // 5 cm tolerance
  // arm_group.setPositionTarget(
  //     target_pose.pose.position.x,
  //     target_pose.pose.position.y,
  //     target_pose.pose.position.z
  //   );
  arm_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(logger, "Planning successful. Executing...");
    arm_group.execute(plan);
    arm_group.clearPoseTargets();
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // ---------------------------
  // Clean shutdown
  // ---------------------------
  executor.cancel();
  spinner.join();

  rclcpp::shutdown();
  return 0;
}