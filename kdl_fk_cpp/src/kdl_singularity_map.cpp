#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <Eigen/Dense>
#include <algorithm>

using namespace std::chrono_literals;

class KDLSingularityMap : public rclcpp::Node
{
public:
  KDLSingularityMap()
  : Node("kdl_singularity_map")
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

    nj_ = chain_.getNrOfJoints();

    fk_solver_  = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&KDLSingularityMap::jointCB, this, std::placeholders::_1));

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/arm/singularity_map", 1);

    timer_ = create_wall_timer(
      200ms, std::bind(&KDLSingularityMap::computeMap, this));
  }

private:
  /* -------------------------------------------------- */

  void jointCB(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_names_ = msg->name;
    joint_pos_   = msg->position;
    joints_ok_   = true;
  }

  double manipulability(const Eigen::MatrixXd &J)
  {
    Eigen::MatrixXd JJ = J * J.transpose();
    return std::sqrt(std::max(0.0, JJ.determinant()));
  }

  /* -------------------------------------------------- */

  void computeMap()
  {
    if (!joints_ok_) return;

    /* ---- current joint vector ---- */
    KDL::JntArray q(nj_);
    unsigned idx = 0;

    for (unsigned i = 0; i < chain_.getNrOfSegments(); ++i)
    {
      const auto &joint = chain_.getSegment(i).getJoint();
      if (joint.getType() == KDL::Joint::None) continue;

      auto it = std::find(joint_names_.begin(),
                          joint_names_.end(),
                          joint.getName());

      q(idx++) = (it != joint_names_.end())
        ? joint_pos_[std::distance(joint_names_.begin(), it)]
        : 0.0;
    }

    /* ---- FK: current EE pose ---- */
    KDL::Frame ee;
    fk_solver_->JntToCart(q, ee);

    /* ---- base Jacobian ---- */
    KDL::Jacobian J_kdl(nj_);
    jac_solver_->JntToJac(q, J_kdl);

    Eigen::MatrixXd J(4, nj_);
    for (unsigned c = 0; c < nj_; c++)
    {
      J(0,c) = J_kdl(0,c);
      J(1,c) = J_kdl(1,c);
      J(2,c) = J_kdl(2,c);
      J(3,c) = J_kdl(4,c);   // pitch
    }

    Eigen::MatrixXd JJT = J * J.transpose();
    Eigen::MatrixXd J_pinv =
      J.transpose() * (JJT + 1e-4 * Eigen::Matrix4d::Identity()).inverse();

    /* ---- marker ---- */
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "arm_base_link";
    m.header.stamp = now();
    m.ns = "singularity";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.015;
    m.scale.y = 0.015;
    m.color.a = 1.0;

    /* ---- local Cartesian sampling ---- */
    double r = 0.06;     // 6 cm radius
    double step = 0.02;  // 2 cm grid

    for (double dx = -r; dx <= r; dx += step)
    for (double dy = -r; dy <= r; dy += step)
    for (double dz = -r; dz <= r; dz += step)
    {
      Eigen::Vector4d dx_task(dx, dy, dz, 0.0);
      Eigen::VectorXd dq = J_pinv * dx_task;

      KDL::JntArray q_test = q;
      for (unsigned i = 0; i < nj_; i++)
        q_test(i) += dq(i);

      jac_solver_->JntToJac(q_test, J_kdl);

      for (unsigned c = 0; c < nj_; c++)
      {
        J(0,c) = J_kdl(0,c);
        J(1,c) = J_kdl(1,c);
        J(2,c) = J_kdl(2,c);
        J(3,c) = J_kdl(4,c);
      }

      double w = manipulability(J);

      geometry_msgs::msg::Point p;
      p.x = ee.p.x() + dx;
      p.y = ee.p.y() + dy;
      p.z = ee.p.z() + dz;

      if (w > 0.03)      { m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; }
      else if (w > 0.015){ m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; }
      else               { m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; }

      m.points.push_back(p);
    }

    marker_pub_->publish(m);
  }

  /* -------- members -------- */

  rclcpp::SyncParametersClient::SharedPtr param_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  KDL::Tree tree_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  std::vector<std::string> joint_names_;
  std::vector<double> joint_pos_;
  bool joints_ok_{false};

  unsigned nj_{0};
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KDLSingularityMap>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}