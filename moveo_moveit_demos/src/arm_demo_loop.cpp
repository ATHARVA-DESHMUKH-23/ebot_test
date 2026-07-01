#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <thread>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "arm_demo_loop",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  auto logger = rclcpp::get_logger("arm_demo");

  using moveit::planning_interface::MoveGroupInterface;
  using moveit::core::MoveItErrorCode;

  MoveGroupInterface arm(node, "arm");
  MoveGroupInterface gripper(node, "hand");

  // Smooth motion
  arm.setMaxVelocityScalingFactor(0.1);
  arm.setMaxAccelerationScalingFactor(0.1);
  gripper.setMaxVelocityScalingFactor(1.0);

  // Wait for robot state
  while (!arm.getCurrentState(1.0))
  {
    RCLCPP_WARN(logger, "Waiting for robot state...");
  }

  RCLCPP_INFO(logger, "Starting demo loop...");

  while (rclcpp::ok())
  {
    // ---- READY ----
    RCLCPP_INFO(logger, "Going to READY");
    arm.setNamedTarget("Ready");
    if (arm.move() != MoveItErrorCode::SUCCESS) continue;
    rclcpp::sleep_for(std::chrono::seconds(1));

    // ---- OPEN GRIPPER ----
    RCLCPP_INFO(logger, "Opening gripper");
    gripper.setNamedTarget("open_gripper");
    if (gripper.move() != MoveItErrorCode::SUCCESS) continue;
    rclcpp::sleep_for(std::chrono::seconds(1));

    // ---- PICK ----
    RCLCPP_INFO(logger, "Going to PICK");
    arm.setNamedTarget("Pick");
    if (arm.move() != MoveItErrorCode::SUCCESS) continue;
    rclcpp::sleep_for(std::chrono::seconds(1));

    // ---- CLOSE GRIPPER ----
    RCLCPP_INFO(logger, "Closing gripper");
    gripper.setNamedTarget("close_gripper");
    if (gripper.move() != MoveItErrorCode::SUCCESS) continue;

    // Stabilization delay (important)
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ---- UPRIGHT ----
    RCLCPP_INFO(logger, "Going UP");
    arm.setNamedTarget("Upright");   // correct case
    if (arm.move() != MoveItErrorCode::SUCCESS) continue;
    rclcpp::sleep_for(std::chrono::seconds(1));

    // ---- READY (transition) ----
    RCLCPP_INFO(logger, "Going to READY (transition before place)");
    arm.setNamedTarget("Ready");
    if (arm.move() != MoveItErrorCode::SUCCESS) continue;
    rclcpp::sleep_for(std::chrono::seconds(1));

    // ---- PLACE ----
    RCLCPP_INFO(logger, "Going to PLACE");
    arm.setNamedTarget("Place");
    if (arm.move() != MoveItErrorCode::SUCCESS) continue;
    rclcpp::sleep_for(std::chrono::seconds(1));

    // ---- OPEN GRIPPER ----
    RCLCPP_INFO(logger, "Releasing object");
    gripper.setNamedTarget("open_gripper");
    if (gripper.move() != MoveItErrorCode::SUCCESS) continue;
    rclcpp::sleep_for(std::chrono::seconds(1));

    // ---- BACK TO READY ----
    RCLCPP_INFO(logger, "Returning to READY");
    arm.setNamedTarget("Ready");
    if (arm.move() != MoveItErrorCode::SUCCESS) continue;
    rclcpp::sleep_for(std::chrono::seconds(2));
  }

  rclcpp::shutdown();
  return 0;
}