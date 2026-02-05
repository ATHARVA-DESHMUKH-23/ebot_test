#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
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

class KDLCartesianServo : public rclcpp::Node
{
public:
  KDLCartesianServo()
  : Node("kdl_cartesian_servo")
  {
    /* -------- robot_description -------- */
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

    nj_ = chain_.getNrOfJoints();

    /* ---- joint limits ---- */
    q_min_ = Eigen::Vector4d(-3.14, -1.5, -2.2, -2.5);
    q_max_ = Eigen::Vector4d( 3.14,  1.5,  2.2,  2.5);
    q_mid_ = 0.5 * (q_min_ + q_max_);

    /* -------- ROS -------- */
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&KDLCartesianServo::jointCB, this, std::placeholders::_1));

    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/arm/cartesian_twist", 10,
      std::bind(&KDLCartesianServo::twistCB, this, std::placeholders::_1));

    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);

    cmd_vel_pub_ =
      create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    timer_ = create_wall_timer(
      10ms, std::bind(&KDLCartesianServo::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Mobile manipulator Cartesian servo READY");
  }

private:
  /* ---------------- callbacks ---------------- */

  void jointCB(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_names_ = msg->name;
    joint_pos_   = msg->position;
    joints_ok_   = true;
  }

  void twistCB(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    xdot_ << msg->linear.x,
             msg->linear.y,
             msg->linear.z,
             msg->angular.y;
    twist_ok_ = true;
  }

  /* ---------------- helpers ---------------- */

  double manipulability(const Eigen::MatrixXd &J)
  {
    Eigen::MatrixXd JJ = J * J.transpose();
    return std::sqrt(std::max(0.0, JJ.determinant()));
  }

  double adaptiveLambda(double w)
  {
    return (w > 0.02) ? 0.0004 : 0.01;
  }

  Eigen::VectorXd jointLimitGradient(const KDL::JntArray &q)
  {
    Eigen::VectorXd grad(nj_);
    grad.setZero();

    for (unsigned i = 0; i < nj_; i++)
    {
      double range = q_max_(i) - q_min_(i);
      grad(i) = -2.0 * (q(i) - q_mid_(i)) / (range * range);
    }
    return grad;
  }

  /* ---------------- servo loop ---------------- */

/* ===================== SERVO LOOP ===================== */

  void controlLoop()
  {
    if (!joints_ok_ || !twist_ok_) return;

    /* Always reset base command */
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";  // or base_footprint

    cmd.twist.linear.x  = 0.0;
    cmd.twist.angular.z = 0.0;


    if (xdot_.norm() < 1e-4)
    {
      cmd_vel_pub_->publish(cmd);
      return;
    }

    /* ---- build joint vector ---- */
    KDL::JntArray q(nj_);
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

    /* ---- ARM JACOBIAN ---- */
    KDL::Jacobian J_arm_kdl(nj_);
    jac_solver_->JntToJac(q, J_arm_kdl);

    Eigen::MatrixXd J_arm(4, nj_);
    for (unsigned c = 0; c < nj_; c++)
    {
      J_arm(0,c) = J_arm_kdl(0,c);
      J_arm(1,c) = J_arm_kdl(1,c);
      J_arm(2,c) = J_arm_kdl(2,c);
      J_arm(3,c) = J_arm_kdl(4,c);
    }

    /* ---- BASE JACOBIAN ---- */
    KDL::Frame ee_frame;
    fk_solver_->JntToCart(q, ee_frame);

    double x_ee = ee_frame.p.x();
    double y_ee = ee_frame.p.y();

    Eigen::Matrix<double,4,3> J_base;
    J_base <<
      1, 0, -y_ee,
      0, 1,  x_ee,
      0, 0,  0,
      0, 0,  1;

    /* ---- FULL JACOBIAN ---- */
    const int nb = 3;
    Eigen::MatrixXd J(4, nb + nj_);
    J.block<4,3>(0,0) = J_base;
    J.block(0, nb, 4, nj_) = J_arm;

    /* ---- MANIPULABILITY ---- */
    double w = manipulability(J_arm);
    double lambda = adaptiveLambda(w);

    /* ---- WEIGHTS (VERY CONSERVATIVE) ---- */
    Eigen::MatrixXd W =
      Eigen::MatrixXd::Identity(nb + nj_, nb + nj_);

    /* Base: extremely expensive by default */
    W(0,0) = 20.0;   // x
    W(1,1) = 100.0;  // y (never)
    W(2,2) = 15.0;   // yaw

    /* Arm: cheap */
    for (unsigned i = 0; i < nj_; i++)
      W(nb + i, nb + i) = 1.0;

    /* ---- Stage 1: yaw assist only ---- */
    if (w < 0.02)
    {
      W(2,2) = 2.0;   // allow yaw
      for (unsigned i = 0; i < nj_; i++)
        W(nb + i, nb + i) = 3.0;
    }

    /* ---- Stage 2: translation allowed (rare) ---- */
    if (w < 0.015)
    {
      W(0,0) = 3.0;   // allow forward motion
    }

    /* ---- WEIGHTED DLS ---- */
    Eigen::MatrixXd JWJt =
      J * W.inverse() * J.transpose();

    Eigen::MatrixXd J_pinv =
      W.inverse() * J.transpose() *
      (JWJt + lambda * Eigen::Matrix4d::Identity()).inverse();

    Eigen::VectorXd qdot_task = J_pinv * xdot_;

    /* ---- NULL SPACE ---- */
    Eigen::MatrixXd I =
      Eigen::MatrixXd::Identity(nb + nj_, nb + nj_);

    Eigen::MatrixXd N = I - J_pinv * J;

    Eigen::VectorXd qdot_null(nb + nj_);
    qdot_null.setZero();
    qdot_null.tail(nj_) = 0.3 * jointLimitGradient(q);

    Eigen::VectorXd qdot = qdot_task + N * qdot_null;

    /* ---- BASE COMMAND (ONLY IF ALLOWED) ---- */
    if (w < 0.02)
    {
      cmd.twist.angular.z = qdot(2);
      if (w < 0.015)
        cmd.twist.linear.x = qdot(0);
    }

    cmd_vel_pub_->publish(cmd);

    /* ---- ARM INTEGRATION ---- */
    for (unsigned i = 0; i < nj_; i++)
      q(i) += qdot(nb + i) * dt_;

    publishTrajectory(q);
  }


  /* ---------------- publish ---------------- */

  void publishTrajectory(const KDL::JntArray &q)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {
      "base_rotation_joint",
      "shoulder_joint",
      "elbow_joint",
      "wrist_joint"
    };

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    for (unsigned i = 0; i < q.rows(); i++)
      pt.positions.push_back(q(i));

    pt.time_from_start = rclcpp::Duration::from_seconds(dt_);
    traj.points.push_back(pt);

    traj_pub_->publish(traj);
  }

  /* ---------------- members ---------------- */

  rclcpp::SyncParametersClient::SharedPtr param_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  KDL::Tree tree_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  std::vector<std::string> joint_names_;
  std::vector<double> joint_pos_;

  Eigen::Vector4d xdot_{Eigen::Vector4d::Zero()};

  Eigen::VectorXd q_min_, q_max_, q_mid_;

  bool joints_ok_{false};
  bool twist_ok_{false};

  unsigned nj_{0};

  double dt_{0.01};
  double w_warn_{0.03};
  double w_crit_{0.015};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KDLCartesianServo>());
  rclcpp::shutdown();
  return 0;
}
