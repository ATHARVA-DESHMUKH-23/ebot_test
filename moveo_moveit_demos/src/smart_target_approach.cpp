#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>

struct ReachPoint
{
  double x;
  double y;
  double z;
  int reachable;
};

class SmartTargetApproachNode : public rclcpp::Node
{
public:
  SmartTargetApproachNode()
  : Node("smart_target_approach"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    declare_parameter("target_frame", "target_frame");

    declare_parameter(
      "csv_path",
      "/home/rajvardhan/mobile_manipulator/workspace7.csv");

    declare_parameter("arm_base_height", 0.30);

    declare_parameter("linear_gain", 0.6);
    declare_parameter("angular_gain", 1.5);

    declare_parameter("max_linear", 0.20);
    declare_parameter("max_angular", 0.80);

    declare_parameter("pos_tolerance", 0.01);
    declare_parameter("yaw_tolerance_deg", 25.0);

    declare_parameter("base_frame", "ebot_base_link");

    loadCSV();

    cmd_pub_ =
      create_publisher<
        geometry_msgs::msg::TwistStamped>(
          "/cmd_vel", 10);

    RCLCPP_INFO(get_logger(), "Waiting for TF...");

    while (rclcpp::ok() &&
      !tf_buffer_.canTransform(
        "map",
        get_parameter("target_frame").as_string(),
        tf2::TimePointZero))
    {
      rclcpp::sleep_for(
        std::chrono::milliseconds(200));
    }

    RCLCPP_INFO(get_logger(), "TF ready");

    computeGoalOnce();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(
        &SmartTargetApproachNode::controlLoop,
        this));
  }

private:
  std::vector<ReachPoint> data_;

  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp::Publisher<
    geometry_msgs::msg::TwistStamped>::SharedPtr
      cmd_pub_;

  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_yaw_{0.0};

  bool goal_ready_{false};

  double clamp(
    double v,
    double lo,
    double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  double normAngle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void loadCSV()
  {
    std::string path =
      get_parameter("csv_path").as_string();

    std::ifstream file(path);

    if (!file.is_open())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Cannot open CSV");
      return;
    }

    std::string line;
    std::getline(file, line);

    while (std::getline(file, line))
    {
      std::stringstream ss(line);
      std::string item;

      ReachPoint p;

      std::getline(ss, item, ',');
      p.x = std::stod(item);

      std::getline(ss, item, ',');
      p.y = std::stod(item);

      std::getline(ss, item, ',');
      p.z = std::stod(item);

      std::getline(ss, item, ',');
      p.reachable = std::stoi(item);

      if (p.reachable == 1)
        data_.push_back(p);
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded %zu reachable points",
      data_.size());
  }

  double bestOffsetForHeight(double z_rel)
  {
    double best_x = 0.35;

    double best_score =
      std::numeric_limits<double>::max();

    for (const auto &p : data_)
    {
      double dz =
        std::fabs(p.z - z_rel);

      if (dz < best_score)
      {
        best_score = dz;
        best_x = p.x;
      }
    }

    return best_x;
  }

  void publishCmd(
    double v,
    double w)
  {
    geometry_msgs::msg::TwistStamped msg;

    msg.header.stamp = now();
    msg.header.frame_id = "base_link";

    msg.twist.linear.x = v;
    msg.twist.angular.z = w;

    cmd_pub_->publish(msg);
  }

  void computeGoalOnce()
  {
    try
    {
      std::string target_frame =
        get_parameter("target_frame")
          .as_string();

      auto tf_target =
        tf_buffer_.lookupTransform(
          "map",
          target_frame,
          tf2::TimePointZero);

      double tx =
        tf_target.transform.translation.x;

      double ty =
        tf_target.transform.translation.y;

      double tz =
        tf_target.transform.translation.z;

      double arm_base_height =
        get_parameter(
          "arm_base_height")
            .as_double();

      double z_rel =
        tz - arm_base_height;

      double offset =
        bestOffsetForHeight(z_rel);

      // --------------------------------
      // Create goal frame relative to target
      // target_frame -> base_target_frame
      // Move along target local -Z
      // --------------------------------
      geometry_msgs::msg::TransformStamped child;

      child.header.stamp = now();
      child.header.frame_id = target_frame;
      child.child_frame_id =
        "base_target_frame";

      child.transform.translation.x = 0.0;
      child.transform.translation.y = 0.0;
      child.transform.translation.z =
        -offset;

      child.transform.rotation.x = 0.0;
      child.transform.rotation.y = 0.0;
      child.transform.rotation.z = 0.0;
      child.transform.rotation.w = 1.0;

      tf_broadcaster_.sendTransform(child);

      rclcpp::sleep_for(
        std::chrono::milliseconds(200));

      auto tf_goal =
        tf_buffer_.lookupTransform(
          "map",
          "base_target_frame",
          tf2::TimePointZero);

      goal_x_ =
        tf_goal.transform.translation.x;

      goal_y_ =
        tf_goal.transform.translation.y;

      // face toward target
      goal_yaw_ =
        std::atan2(
          ty - goal_y_,
          tx - goal_x_);

      goal_ready_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Goal fixed: x=%.2f y=%.2f yaw=%.1f",
        goal_x_,
        goal_y_,
        goal_yaw_ * 180.0 / M_PI);
    }
    catch (...)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to compute goal");
    }
  }

  void controlLoop()
  {
    if (!goal_ready_)
      return;

    geometry_msgs::msg::TransformStamped tf_robot;

    try
    {
      tf_robot =
        tf_buffer_.lookupTransform(
          "map",
          get_parameter("base_frame")
            .as_string(),
          tf2::TimePointZero);
    }
    catch (...)
    {
      publishCmd(0.0, 0.0);
      return;
    }

    double rx =
      tf_robot.transform.translation.x;

    double ry =
      tf_robot.transform.translation.y;

    double robot_yaw =
      tf2::getYaw(
        tf_robot.transform.rotation);

    double dx = goal_x_ - rx;
    double dy = goal_y_ - ry;

    double dist =
      std::sqrt(dx * dx + dy * dy);

    double heading =
      std::atan2(dy, dx);

    double yaw_error =
      normAngle(
        heading - robot_yaw);

    double lin_k =
      get_parameter(
        "linear_gain")
          .as_double();

    double ang_k =
      get_parameter(
        "angular_gain")
          .as_double();

    double max_lin =
      get_parameter(
        "max_linear")
          .as_double();

    double max_ang =
      get_parameter(
        "max_angular")
          .as_double();

    double pos_tol =
      get_parameter(
        "pos_tolerance")
          .as_double();

    double yaw_tol =
      get_parameter(
        "yaw_tolerance_deg")
          .as_double() *
      M_PI / 180.0;

    double v = 0.0;
    double w = 0.0;

    if (dist > pos_tol)
    {
      w = clamp(
        ang_k * yaw_error,
        -max_ang,
        max_ang);

      if (std::fabs(yaw_error) <
          yaw_tol)
      {
        v = clamp(
          lin_k * dist,
          0.05,
          max_lin);
      }
      else
      {
        v = 0.03;
      }
    }
    else
    {
      double final_yaw_error =
        normAngle(
          goal_yaw_ - robot_yaw);

      if (std::fabs(final_yaw_error) >
          0.08)
      {
        w = clamp(
          ang_k * final_yaw_error,
          -0.4,
          0.4);
      }
    }

    publishCmd(v, w);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "dist=%.2f yaw=%.1f v=%.2f w=%.2f",
      dist,
      yaw_error * 180.0 / M_PI,
      v,
      w);
  }
};

int main(
  int argc,
  char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<
      SmartTargetApproachNode>());

  rclcpp::shutdown();

  return 0;
}