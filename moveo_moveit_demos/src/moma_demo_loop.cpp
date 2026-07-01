#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <thread>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

bool sendNavGoal(
  rclcpp::Node::SharedPtr node,
  rclcpp_action::Client<NavigateToPose>::SharedPtr client,
  double x, double y,
  double qz, double qw)
{
  auto logger = node->get_logger();

  NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = node->now();

  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.orientation.z = qz;
  goal_msg.pose.pose.orientation.w = qw;

  auto goal_handle_future = client->async_send_goal(goal_msg);

  // Wait for goal handle
  while (rclcpp::ok())
  {
    if (goal_handle_future.wait_for(std::chrono::milliseconds(0)) 
        == std::future_status::ready)
      break;

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (!rclcpp::ok())
  {
    RCLCPP_WARN(logger, "Interrupted before goal accepted");
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(logger, "Goal rejected");
    return false;
  }

  RCLCPP_INFO(logger, "Navigating...");

  auto result_future = client->async_get_result(goal_handle);

  // Wait for result
  while (rclcpp::ok())
  {
    auto status = result_future.wait_for(std::chrono::milliseconds(0));

    if (status == std::future_status::ready)
      break;

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (!rclcpp::ok())
  {
    RCLCPP_WARN(node->get_logger(), "Interrupted! Cancelling navigation...");

    try {
      client->async_cancel_all_goals();
    } catch (...) {}

    return false;
  }

  auto result = result_future.get();

  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(logger, "Navigation failed");
    return false;
  }

  RCLCPP_INFO(logger, "Reached goal!");
  return true;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "arm_demo_loop",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() {
    while (rclcpp::ok())
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }).detach();
  auto logger = rclcpp::get_logger("arm_demo");

  using moveit::planning_interface::MoveGroupInterface;
  using moveit::core::MoveItErrorCode;

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  auto nav_client = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");

  if (!nav_client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(logger, "Nav2 action server not available!");
    return 1;
  }

  MoveGroupInterface arm(node, "arm");
  MoveGroupInterface gripper(node, "hand");

  // Smooth motion
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);
  gripper.setMaxVelocityScalingFactor(1.0);

  double pick_x = -0.845059;
  double pick_y = -0.944412;
  double pick_qz = -0.681187;
  double pick_qw = 0.732110;

  double place_x = 1.812490;
  double place_y = 0.683485;
  double place_qz = 0.696012;
  double place_qw = 0.718030;
  // Wait for robot state
  while (!arm.getCurrentState(2.0))
  {
    RCLCPP_WARN(logger, "Waiting for robot state...");
  }

  RCLCPP_INFO(logger, "Starting demo loop...");

  while (rclcpp::ok())
  {
    // ---- READY ----
    arm.setNamedTarget("Ready");
    if (arm.move() != MoveItErrorCode::SUCCESS) continue;

    // ---- GO TO PICK (BASE) ----
    RCLCPP_INFO(logger, "Going to PICK location (base)");
    if (!sendNavGoal(node, nav_client, pick_x, pick_y, pick_qz, pick_qw)) continue;

    rclcpp::sleep_for(std::chrono::seconds(1));

    // ---- ARM PICK ----
    arm.setNamedTarget("Pick");
    if (arm.move() != MoveItErrorCode::SUCCESS) continue;

    gripper.setNamedTarget("close_gripper");
    if (gripper.move() != MoveItErrorCode::SUCCESS) continue;

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ---- RETREAT ----
    arm.setNamedTarget("Ready");
    if (arm.move() != MoveItErrorCode::SUCCESS) continue;

    // ---- GO TO PLACE (BASE) ----
    RCLCPP_INFO(logger, "Going to PLACE location (base)");
    if (!sendNavGoal(node, nav_client, place_x, place_y, place_qz, place_qw)) continue;

    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // // ---- UPRIGHT BEFORE PLACE ----
    // RCLCPP_INFO(logger, "Going Upright before placing");
    // arm.setNamedTarget("Upright");
    // if (arm.move() != MoveItErrorCode::SUCCESS) continue;

    // rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ---- ARM PLACE ----
    arm.setNamedTarget("Place");
    if (arm.move() != MoveItErrorCode::SUCCESS) continue;

    gripper.setNamedTarget("open_gripper");
    if (gripper.move() != MoveItErrorCode::SUCCESS) continue;

    // ---- RETREAT ----
    arm.setNamedTarget("Ready");
    if (arm.move() != MoveItErrorCode::SUCCESS) continue;
  }
  rclcpp::shutdown();
  return 0;
}