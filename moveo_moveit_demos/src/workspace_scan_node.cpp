#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <thread>
#include <fstream>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "workspace_scanner",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto logger = rclcpp::get_logger("workspace_scanner");

  // Executor (REQUIRED for MoveIt)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface arm_group(node, "arm");

  // Planner settings (fast)
  arm_group.setPlanningPipelineId("ompl");
  arm_group.setPlannerId("RRTConnectkConfigDefault");
  arm_group.setPlanningTime(0.5);
  arm_group.setNumPlanningAttempts(1);
  arm_group.setMaxVelocityScalingFactor(1.0);
  arm_group.setMaxAccelerationScalingFactor(1.0);
  arm_group.setGoalTolerance(0.05);
  arm_group.setGoalOrientationTolerance(0.54159);

  // Wait for robot state
  while (rclcpp::ok() && !arm_group.getCurrentState(1.0)) {
    RCLCPP_WARN(logger, "Waiting for robot state...");
  }

  if (!rclcpp::ok()) {
    rclcpp::shutdown();
    return 0;
  }

  RCLCPP_INFO(logger, "Starting workspace scan...");

  // -----------------------------
  // Parameters (easy to tweak)
  // -----------------------------
  double start_x = 0.0;
  double start_y = 0.0;
  double start_z = 1.0;

  // -----------------------------
  // Fixed orientation
  // -----------------------------
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = -0.5;
  target_pose.orientation.y = 0.5;
  target_pose.orientation.z = -0.5;
  target_pose.orientation.w = 0.5;

  // -----------------------------
  // CSV logging (optional)
  // -----------------------------
  std::ofstream file("workspace.csv");
  file << "x,y,z,reachable\n";

  // -----------------------------
  // Pyramid scan
  // -----------------------------
  double z = 1.0;
  double z_step = 0.02;

  double range_x = 0.2;
  double max_x_cap = 0.6;
  double x_step = 0.02;

  double fixed_y = 0.0;   // keep Y constant

  while (z >= 0.0)
  {
    double current_x_limit = std::min(range_x, max_x_cap);

    RCLCPP_INFO(logger,
      "Scanning z=%.2f | x range: 0 → %.2f", z, current_x_limit);

    for (double x = 0.0; x <= current_x_limit; x += x_step)
    {
      target_pose.position.x = x;
      target_pose.position.y = fixed_y;
      target_pose.position.z = z;

      arm_group.clearPoseTargets();
      arm_group.setStartStateToCurrentState();
      arm_group.setPoseTarget(target_pose);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success =
        (arm_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success)
      {
        RCLCPP_INFO(logger,
          "✅ x=%.2f z=%.2f", x, z);
      }
      else
      {
        RCLCPP_WARN(logger,
          "❌ x=%.2f z=%.2f", x, z);
      }

      file << x << "," << fixed_y << "," << z << "," << success << "\n";
    }

    // 🔽 go down
    z -= z_step;

    // 🔼 expand x range
    range_x += 0.02;
  }
  file.close();

  RCLCPP_INFO(logger, "Workspace scan complete. Saved to workspace.csv");

  // Shutdown cleanly
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();

  return 0;
}