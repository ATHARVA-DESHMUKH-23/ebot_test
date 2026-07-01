#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <vector>
#include <cmath>

struct Solution
{
  double base_x, base_y, base_yaw;
  std::vector<double> joint_values;
};

class MobileManipulatorPlanner : public rclcpp::Node
{
public:
  MobileManipulatorPlanner()
  : Node("mobile_manipulator_planner")
  {
    RCLCPP_INFO(this->get_logger(), "Starting Mobile Manipulator Planner...");

    // Delay to ensure MoveIt is ready
    rclcpp::sleep_for(std::chrono::seconds(1));

  }
  void start()
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    init_moveit();
    plan_and_execute();
  }

private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  void init_moveit()
  {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm");
  }

  geometry_msgs::msg::PoseStamped getTargetPose()
  {
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "odom";

    target.pose.position.x = 0.8;
    target.pose.position.y = 0.0;
    target.pose.position.z = 0.4;

    // Better orientation
    tf2::Quaternion q;
    q.setRPY(0, M_PI, 0);
    target.pose.orientation = tf2::toMsg(q);

    return target;
  }

  geometry_msgs::msg::Pose transformToBase(
      const geometry_msgs::msg::PoseStamped& target,
      double base_x, double base_y, double base_yaw)
  {
    tf2::Transform T_world_base;
    T_world_base.setOrigin(tf2::Vector3(base_x, base_y, 0));

    tf2::Quaternion q;
    q.setRPY(0, 0, base_yaw);
    T_world_base.setRotation(q);

    tf2::Transform T_world_ee;
    tf2::fromMsg(target.pose, T_world_ee);

    tf2::Transform T_base_ee = T_world_base.inverse() * T_world_ee;

    geometry_msgs::msg::Pose pose;

    pose.position.x = T_base_ee.getOrigin().x();
    pose.position.y = T_base_ee.getOrigin().y();
    pose.position.z = T_base_ee.getOrigin().z();

    pose.orientation = tf2::toMsg(T_base_ee.getRotation());

    return pose;
  }

  void plan_and_execute()
  {
    auto target = getTargetPose();

    double r_min = 0.3;
    double r_max = 0.8;   // limited to arm reach
    double r_step = 0.1;
    double theta_step = 0.3;

    std::vector<Solution> solutions;

    auto current_state = move_group_->getCurrentState(2.0);
    const moveit::core::JointModelGroup* jmg =
        current_state->getJointModelGroup("arm");

    // Focused search direction
    double target_yaw = atan2(target.pose.position.y,
                              target.pose.position.x);

    for (double theta = target_yaw - M_PI/2;
         theta <= target_yaw + M_PI/2;
         theta += theta_step)
    {
      for (double r = r_min; r <= r_max; r += r_step)
      {
        double base_x = target.pose.position.x - r * cos(theta);
        double base_y = target.pose.position.y - r * sin(theta);
        double base_yaw = theta;

        auto target_in_base = transformToBase(target, base_x, base_y, base_yaw);

        moveit::core::RobotState test_state(*current_state);

        bool found_ik = test_state.setFromIK(
            jmg,
            target_in_base,
            0.1);

        if (found_ik)
        {
          std::vector<double> joint_values;
          test_state.copyJointGroupPositions(jmg, joint_values);

          solutions.push_back({base_x, base_y, base_yaw, joint_values});
        }
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "Found %ld valid solutions", solutions.size());

    if (solutions.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "No valid solution found!");
      return;
    }

    // 🔥 Improved scoring
    Solution best = solutions[0];
    double best_score = computeScore(best);

    for (auto& sol : solutions)
    {
      double score = computeScore(sol);
      if (score < best_score)
      {
        best = sol;
        best_score = score;
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "Best base: x=%.2f y=%.2f yaw=%.2f",
                best.base_x, best.base_y, best.base_yaw);

    // 👉 Base execution placeholder
    RCLCPP_INFO(this->get_logger(), "Send base goal using Nav2 here!");

    // Execute arm
    move_group_->setJointValueTarget(best.joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      move_group_->execute(plan);
      RCLCPP_INFO(this->get_logger(), "Arm executed!");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
  }

  double computeScore(const Solution& sol)
  {
    double base_cost = hypot(sol.base_x, sol.base_y);

    double joint_cost = 0;
    for (auto j : sol.joint_values)
      joint_cost += fabs(j);

    return base_cost + 0.1 * joint_cost;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MobileManipulatorPlanner>();

  // 🔥 CRITICAL FIX (same as hello_moveit)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Now safe to start
  node->start();

  rclcpp::shutdown();
  return 0;
}