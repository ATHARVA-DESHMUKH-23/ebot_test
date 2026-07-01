#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <thread>
#include <atomic>
#include <rclcpp/executors/single_threaded_executor.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "moveit_goal",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto logger = node->get_logger();

  // TF
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  auto tf_broadcaster =
    std::make_shared<tf2_ros::TransformBroadcaster>(node);

  // Executor (for TF + MoveIt)
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread([&exec]() { exec.spin(); }).detach();

  // Params
  std::string target_frame =
    node->declare_parameter("target_frame", "test_target");

  std::string base_frame =
    node->declare_parameter("base_frame", "ebot_base_link");

  // MoveIt
  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface arm(node, "arm");

  arm.setPlanningPipelineId("ompl");
  arm.setPlannerId("RRTConnectkConfigDefault");
  arm.setPlanningTime(5.0);
  arm.setNumPlanningAttempts(10);

  arm.setMaxVelocityScalingFactor(1.0);
  arm.setMaxAccelerationScalingFactor(1.0);

  arm.setStartStateToCurrentState();
  arm.setPoseReferenceFrame(base_frame);

  RCLCPP_INFO(logger, "Waiting for TF...");

  geometry_msgs::msg::TransformStamped tf_target;

  auto start = node->now();

  while (rclcpp::ok())
  {
    try
    {
      tf_target = tf_buffer.lookupTransform(
        base_frame,
        target_frame,
        tf2::TimePointZero);
      break;
    }
    catch (...)
    {
      if ((node->now() - start).seconds() > 5.0)
      {
        RCLCPP_ERROR(logger, "TF timeout");
        rclcpp::shutdown();
        return 1;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
  }

  RCLCPP_INFO(logger, "TF received");

  // TF → Pose
  geometry_msgs::msg::Pose target_pose;

  target_pose.position.x = tf_target.transform.translation.x;
  target_pose.position.y = tf_target.transform.translation.y;
  target_pose.position.z = tf_target.transform.translation.z;

  target_pose.orientation = tf_target.transform.rotation;
  
  arm.setGoalPositionTolerance(0.1);        // 5 cm
  arm.setGoalOrientationTolerance(0.3);      // ~17.2 degrees
  
  RCLCPP_INFO(logger,
    "Target Pose: x=%.3f y=%.3f z=%.3f",
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z);

  // TF publisher thread
  std::atomic<bool> publish_tf{true};

  std::thread tf_thread([&]()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.frame_id = base_frame;
    t.child_frame_id = "moveit_goal";

    t.transform.translation.x = target_pose.position.x;
    t.transform.translation.y = target_pose.position.y;
    t.transform.translation.z = target_pose.position.z;
    t.transform.rotation = target_pose.orientation;

    rclcpp::Rate rate(20);

    while (rclcpp::ok() && publish_tf)
    {
      t.header.stamp = node->now();
      tf_broadcaster->sendTransform(t);
      rate.sleep();
    }
  });

  // MoveIt
  arm.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success =
    (arm.plan(plan) ==
     moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(logger, "Executing plan...");
    arm.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed");
  }

  // Stop TF thread cleanly
  publish_tf = false;
  tf_thread.join();

  rclcpp::shutdown();
  return 0;
}