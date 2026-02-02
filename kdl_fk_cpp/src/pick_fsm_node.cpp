#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ebot_navigation_interfaces/action/navigate_to_pose_odom.hpp>

#include <cmath>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;

class PickFSM : public rclcpp::Node
{
public:
  using NavAction = ebot_navigation_interfaces::action::NavigateToPoseOdom;
  using ArmAction = control_msgs::action::FollowJointTrajectory;

  PickFSM()
  : Node("pick_fsm_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    /* ---------------- Subscribers ---------------- */
    pick_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/pick_object", 10,
      std::bind(&PickFSM::pickCB, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PickFSM::odomCB, this, std::placeholders::_1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&PickFSM::jointStateCB, this, std::placeholders::_1));

    /* ---------------- Publishers ---------------- */
    arm_target_pub_ = create_publisher<geometry_msgs::msg::Point>(
      "/arm/cartesian_target", 10);

    arm_pitch_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/arm/cartesian_pitch", 10);

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/pick_marker", 10);

    /* ---------------- Action Clients ---------------- */
    nav_client_ =
      rclcpp_action::create_client<NavAction>(
        this, "navigate_to_pose_odom");

    arm_client_ =
      rclcpp_action::create_client<ArmAction>(
        this, "/arm_controller/follow_joint_trajectory");

    /* ---------------- Timer ---------------- */
    timer_ = create_wall_timer(
      100ms, std::bind(&PickFSM::fsmStep, this));

    RCLCPP_INFO(get_logger(), "✅ Pick FSM node started (robust + safe)");
  }

private:
  /* ================= FSM ================= */

  enum class State {
    IDLE,
    ARM_TO_START,
    WAIT_ARM_START,
    MOVE_BASE,
    WAIT_BASE,
    MOVE_ARM,
    WAIT_ARM
  };

  State state_ = State::IDLE;

  /* ================= Callbacks ================= */

  void pickCB(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (state_ != State::IDLE) {
      RCLCPP_WARN(get_logger(), "Pick request ignored (busy)");
      return;
    }

    object_odom_ = *msg;
    goal_received_ = true;

    /* RESET FSM FLAGS */
    arm_init_sent_ = false;
    nav_goal_done_ = false;
    nav_goal_success_ = false;
    joint_state_ok_ = false;

    publishPickMarker(*msg);

    RCLCPP_INFO(get_logger(),
      "📦 Pick request: [%.2f %.2f %.2f]",
      msg->point.x, msg->point.y, msg->point.z);
  }

  void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    odom_ok_ = true;
  }

  void jointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto idx = [&](const std::string & name) -> int {
      auto it = std::find(msg->name.begin(), msg->name.end(), name);
      return (it == msg->name.end()) ? -1 : std::distance(msg->name.begin(), it);
    };

    int i0 = idx("base_rotation_joint");
    int i1 = idx("shoulder_joint");
    int i2 = idx("elbow_joint");
    int i3 = idx("wrist_joint");

    if (i0 < 0 || i1 < 0 || i2 < 0 || i3 < 0) return;

    current_q_ = {
      msg->position[i0],
      msg->position[i1],
      msg->position[i2],
      msg->position[i3]
    };

    joint_state_ok_ = true;
  }

  /* ================= FSM LOOP ================= */

  void fsmStep()
  {
    switch (state_)
    {
      case State::IDLE:
        if (goal_received_ && odom_ok_) {
          state_ = State::ARM_TO_START;
        }
        break;

      case State::ARM_TO_START:
        if (!arm_init_sent_) sendArmToStartPose();
        state_ = State::WAIT_ARM_START;
        break;

      case State::WAIT_ARM_START:
        if (joint_state_ok_ && armAtStartPose()) {
          RCLCPP_INFO(get_logger(), "🛡️ Arm confirmed at START pose");
          state_ = State::MOVE_BASE;
        }
        break;

      case State::MOVE_BASE:
        sendBaseGoal();
        state_ = State::WAIT_BASE;
        break;

      case State::WAIT_BASE:
        if (nav_goal_done_) {
          if (nav_goal_success_) {
            state_ = State::MOVE_ARM;
          } else {
            reset();
          }
        }
        break;


      case State::MOVE_ARM:
        sendArmCartesian();
        state_ = State::WAIT_ARM;
        break;

      case State::WAIT_ARM:
        if (armReached()) {
          RCLCPP_INFO(get_logger(), "✅ Pick completed");
          reset();
        }
        break;
    }
  }

  /* ================= Arm START pose ================= */

  void sendArmToStartPose()
  {
    if (!arm_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(get_logger(), "Arm action server unavailable");
      return;
    }

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {
      "base_rotation_joint",
      "shoulder_joint",
      "elbow_joint",
      "wrist_joint"
    };

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = start_q_;
    pt.time_from_start = rclcpp::Duration::from_seconds(2.0);
    traj.points.push_back(pt);

    ArmAction::Goal goal;
    goal.trajectory = traj;

    arm_client_->async_send_goal(goal);
    arm_init_sent_ = true;

    RCLCPP_INFO(get_logger(), "🛡️ Sending arm to START pose");
  }

  bool armAtStartPose()
  {
    double err = 0.0;
    for (size_t i = 0; i < start_q_.size(); i++)
      err += std::abs(start_q_[i] - current_q_[i]);

    return err < arm_tol_;
  }

  /* ================= Base ================= */

  void sendBaseGoal()
  {
    if (!nav_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(get_logger(), "Base nav action server unavailable");
      return;
    }

    double yaw = std::atan2(
      object_odom_.point.y - robot_y_,
      object_odom_.point.x - robot_x_);

    NavAction::Goal goal;
    goal.x = object_odom_.point.x;
    goal.y = object_odom_.point.y;
    goal.yaw = yaw;

    nav_goal_done_ = false;
    nav_goal_success_ = false;

    rclcpp_action::Client<NavAction>::SendGoalOptions options;
    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavAction>::WrappedResult & res)
      {
        nav_goal_done_ = true;
        nav_goal_success_ =
          (res.code == rclcpp_action::ResultCode::SUCCEEDED);

        RCLCPP_INFO(
          get_logger(),
          "🚗 Base nav result: %s",
          nav_goal_success_ ? "SUCCESS" : "FAILED");
      };

    nav_client_->async_send_goal(goal, options);
  }


  /* ================= Arm Cartesian ================= */

  void sendArmCartesian()
  {
    auto obj_arm =
      tf_buffer_.transform(object_odom_, "arm_base_link");

    geometry_msgs::msg::Point p = obj_arm.point;
    arm_target_pub_->publish(p);

    std_msgs::msg::Float64 pitch;
    pitch.data = 1.57;
    arm_pitch_pub_->publish(pitch);

    last_arm_ = {p.x, p.y, p.z};
  }

  bool armReached()
  {
    auto ee =
      tf_buffer_.lookupTransform("arm_base_link","tool0",tf2::TimePointZero);

    return std::hypot(
      ee.transform.translation.x - last_arm_[0],
      ee.transform.translation.y - last_arm_[1]) < 0.01;
  }

  /* ================= Marker ================= */

  void publishPickMarker(const geometry_msgs::msg::PointStamped & obj)
  {
    visualization_msgs::msg::Marker m;
    m.header = obj.header;
    m.ns = "pick_object";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = obj.point;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.10;
    m.color.r = 1.0;
    m.color.a = 1.0;
    marker_pub_->publish(m);
  }

  void reset()
  {
    state_ = State::IDLE;
    goal_received_ = false;
    arm_init_sent_ = false;
  }

  /* ================= Members ================= */

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pick_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr arm_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm_pitch_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp_action::Client<NavAction>::SharedPtr nav_client_;
  rclcpp_action::Client<ArmAction>::SharedPtr arm_client_;

  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::msg::PointStamped object_odom_;

  bool goal_received_{false};
  bool odom_ok_{false};
  bool nav_goal_done_{false};
  bool nav_goal_success_{false};
  bool joint_state_ok_{false};
  bool arm_init_sent_{false};

  std::vector<double> current_q_{0,0,0,0};
  const std::vector<double> start_q_{0.0, -0.6, 1.8, 0.0};
  const double arm_tol_{0.03};

  double robot_x_{0.0}, robot_y_{0.0};
  std::vector<double> last_arm_{0,0,0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PickFSM>());
  rclcpp::shutdown();
  return 0;
}
