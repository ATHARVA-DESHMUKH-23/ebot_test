#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

using namespace std::chrono_literals;

class KDLCartesianPointController : public rclcpp::Node
{
public:
  KDLCartesianPointController() : Node("kdl_cartesian_point_controller")
  {
    /* ---------------- robot_description ---------------- */
    param_client_ =
      std::make_shared<rclcpp::SyncParametersClient>(
        this, "robot_state_publisher");

    while (!param_client_->wait_for_service(1s))
      RCLCPP_INFO(get_logger(), "Waiting for robot_state_publisher...");

    auto params = param_client_->get_parameters({"robot_description"});
    auto robot = urdf::parseURDF(params[0].as_string());

    if (!kdl_parser::treeFromUrdfModel(*robot, tree_) ||
        !tree_.getChain("arm_base_link", "tool0", chain_))
    {
      RCLCPP_FATAL(get_logger(), "Failed to build KDL chain");
      rclcpp::shutdown();
      return;
    }

    fk_solver_  = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);

    RCLCPP_INFO(get_logger(), "KDL ready | joints=%d", chain_.getNrOfJoints());

    /* ---------------- ROS interfaces ---------------- */
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&KDLCartesianPointController::jointCB, this, std::placeholders::_1));

    target_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/arm/cartesian_target", 10,
      std::bind(&KDLCartesianPointController::targetCB, this, std::placeholders::_1));

    pitch_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/arm/cartesian_pitch", 10,
      std::bind(&KDLCartesianPointController::pitchCB, this, std::placeholders::_1));

    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);

    timer_ = create_wall_timer(
      20ms, std::bind(&KDLCartesianPointController::controlLoop, this));
  }

private:
  /* ---------------- callbacks ---------------- */
  void jointCB(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_names_ = msg->name;
    joint_pos_   = msg->position;
    joints_ok_   = true;
  }

  void targetCB(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    target_ << msg->x, msg->y, msg->z;
    target_ok_ = true;
    RCLCPP_INFO(get_logger(),
      "🎯 Target XYZ = [%.3f %.3f %.3f]", target_.x(), target_.y(), target_.z());
  }

  void pitchCB(const std_msgs::msg::Float64::SharedPtr msg)
  {
    target_pitch_ = msg->data;
    pitch_ok_ = true;
    RCLCPP_INFO(get_logger(),
      "🎯 Target pitch = %.3f rad", target_pitch_);
  }

  /* ---------------- control loop ---------------- */
  void controlLoop()
  {
    if (!joints_ok_ || !target_ok_ || !pitch_ok_)
      return;

    /* ---- build joint array ---- */
    KDL::JntArray q(chain_.getNrOfJoints());
    unsigned idx = 0;

    for (unsigned i = 0; i < chain_.getNrOfSegments(); ++i)
    {
      const auto & joint = chain_.getSegment(i).getJoint();
      if (joint.getType() == KDL::Joint::None) continue;

      auto it = std::find(joint_names_.begin(),
                          joint_names_.end(),
                          joint.getName());

      q(idx++) = (it != joint_names_.end())
        ? joint_pos_[std::distance(joint_names_.begin(), it)]
        : 0.0;
    }

    /* ---- FK ---- */
    KDL::Frame ee;
    fk_solver_->JntToCart(q, ee);

    double roll, pitch_cur, yaw;
    ee.M.GetRPY(roll, pitch_cur, yaw);

    Eigen::Vector3d x_cur(ee.p.x(), ee.p.y(), ee.p.z());

    Eigen::Vector4d err;
    err <<
      target_.x() - x_cur.x(),
      target_.y() - x_cur.y(),
      target_.z() - x_cur.z(),
      target_pitch_ - pitch_cur;

    if (err.head<3>().norm() < 0.005 && std::abs(err(3)) < 0.01)
      return;  // ⭐ anti-jitter

    /* ---- Jacobian ---- */
    KDL::Jacobian J_kdl(chain_.getNrOfJoints());
    jac_solver_->JntToJac(q, J_kdl);

    Eigen::MatrixXd J(4, chain_.getNrOfJoints());
    for (unsigned c = 0; c < chain_.getNrOfJoints(); c++)
    {
      J(0, c) = J_kdl(0, c); // vx
      J(1, c) = J_kdl(1, c); // vy
      J(2, c) = J_kdl(2, c); // vz
      J(3, c) = J_kdl(4, c); // pitch (wy)
    }

    /* ---- DLS IK ---- */
    double lambda = 0.02;
    Eigen::VectorXd qdot =
      J.transpose() *
      (J * J.transpose() +
       lambda * lambda * Eigen::Matrix4d::Identity()).inverse()
      * (2.0 * err);

    /* ---- integrate ---- */
    Eigen::VectorXd q_cmd(chain_.getNrOfJoints());
    for (unsigned i = 0; i < q.rows(); i++)
      q_cmd(i) = q(i) + qdot(i) * 0.02;

    publishTrajectory(q_cmd);
  }

  void publishTrajectory(const Eigen::VectorXd &q)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {
      "base_rotation_joint",
      "shoulder_joint",
      "elbow_joint",
      "wrist_joint"
    };

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    for (int i = 0; i < q.size(); i++)
      pt.positions.push_back(q(i));

    pt.time_from_start = rclcpp::Duration::from_seconds(0.02);
    traj.points.push_back(pt);

    traj_pub_->publish(traj);
  }

  /* ---------------- members ---------------- */
  rclcpp::SyncParametersClient::SharedPtr param_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  KDL::Tree tree_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  std::vector<std::string> joint_names_;
  std::vector<double> joint_pos_;

  Eigen::Vector3d target_;
  double target_pitch_ = 0.0;

  bool joints_ok_ = false;
  bool target_ok_ = false;
  bool pitch_ok_  = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KDLCartesianPointController>());
  rclcpp::shutdown();
  return 0;
}
