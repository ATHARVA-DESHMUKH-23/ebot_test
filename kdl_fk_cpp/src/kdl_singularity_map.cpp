#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include <Eigen/Dense>
#include <algorithm>

using namespace std::chrono_literals;

class KDLLocalSingularityMap : public rclcpp::Node
{
public:
  KDLLocalSingularityMap()
  : Node("kdl_local_singularity_map")
  {
    /* ---- robot_description ---- */
    auto pc = std::make_shared<rclcpp::SyncParametersClient>(
      this, "robot_state_publisher");

    pc->wait_for_service();

    auto robot = urdf::parseURDF(
      pc->get_parameters({"robot_description"})[0].as_string());

    kdl_parser::treeFromUrdfModel(*robot, tree_);
    tree_.getChain("arm_base_link", "tool0", chain_);

    fk_solver_  = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);
    ik_solver_  = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);

    nj_ = chain_.getNrOfJoints();

    /* ---- ROS ---- */
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&KDLLocalSingularityMap::jointCB, this, std::placeholders::_1));

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/singularity_map",
      rclcpp::QoS(1).transient_local().reliable());

    timer_ = create_wall_timer(
      500ms, std::bind(&KDLLocalSingularityMap::generateLocalMap, this));

    RCLCPP_INFO(get_logger(), "Local singularity map node started");
  }

private:
  /* ---------------- callbacks ---------------- */

  void jointCB(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() < nj_) return;

    q_current_.resize(nj_);
    for (unsigned i = 0; i < nj_; i++)
      q_current_(i) = msg->position[i];

    joints_ok_ = true;
  }

  /* ---------------- helpers ---------------- */

  double manipulability(const Eigen::MatrixXd &J)
  {
    Eigen::MatrixXd JJ = J * J.transpose();
    return std::sqrt(std::max(0.0, JJ.determinant()));
  }

  /* ---------------- local map ---------------- */

  void generateLocalMap()
  {
    if (!joints_ok_) return;

    visualization_msgs::msg::MarkerArray arr;

    /* ---- clear old markers ---- */
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    clear.header.frame_id = "arm_base_link";
    arr.markers.push_back(clear);

    /* ---- FK: current EE pose ---- */
    KDL::JntArray q_kdl(nj_);
    for (unsigned i = 0; i < nj_; i++)
      q_kdl(i) = q_current_(i);

    KDL::Frame ee;
    fk_solver_->JntToCart(q_kdl, ee);

    /* ---- sample local cube ---- */
    int id = 0;
    for (double dx = -0.10; dx <= 0.10; dx += 0.02)
    for (double dy = -0.10; dy <= 0.10; dy += 0.02)
    for (double dz = -0.10; dz <= 0.10; dz += 0.02)
    {
      KDL::Frame target = ee;
      target.p.data[0] += dx;
      target.p.data[1] += dy;
      target.p.data[2] += dz;

      KDL::JntArray q_seed = q_kdl;
      if (ik_solver_->CartToJnt(q_seed, target, q_seed) < 0)
        continue;

      KDL::Jacobian J(nj_);
      jac_solver_->JntToJac(q_seed, J);

      Eigen::MatrixXd Je(3, nj_);
      for (unsigned i = 0; i < nj_; i++)
      {
        Je(0,i) = J(0,i);
        Je(1,i) = J(1,i);
        Je(2,i) = J(2,i);
      }

      double w = manipulability(Je);

      visualization_msgs::msg::Marker m;
      m.header.frame_id = "arm_base_link";
      m.header.stamp = rclcpp::Time(0);
      m.ns = "local_singularity";
      m.id = id++;
      m.type = m.SPHERE;
      m.action = m.ADD;
      m.pose.position.x = target.p.x();
      m.pose.position.y = target.p.y();
      m.pose.position.z = target.p.z();
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.015;
      m.lifetime = rclcpp::Duration::from_seconds(0);

      if (w < 0.015) { m.color.r = 1; m.color.a = 1; }
      else if (w < 0.03) { m.color.r = 1; m.color.g = 1; m.color.a = 1; }
      else { m.color.g = 1; m.color.a = 1; }

      arr.markers.push_back(m);
    }

    pub_->publish(arr);
  }

  /* ---------------- members ---------------- */

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  KDL::Tree tree_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;

  Eigen::VectorXd q_current_;
  bool joints_ok_{false};
  unsigned nj_{0};
};

/* ---------------- main ---------------- */

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KDLLocalSingularityMap>());
  rclcpp::shutdown();
  return 0;
}
