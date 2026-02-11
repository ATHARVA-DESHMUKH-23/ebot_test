#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class KDLWorkspaceCloud : public rclcpp::Node
{
public:
  KDLWorkspaceCloud()
  : Node("kdl_workspace_cloud")
  {
    /* -------- robot_description -------- */
    param_client_ =
      std::make_shared<rclcpp::SyncParametersClient>(
        this, "robot_state_publisher");

    while (!param_client_->wait_for_service(1s))
      RCLCPP_INFO(get_logger(), "Waiting for robot_state_publisher...");

    auto params = param_client_->get_parameters({"robot_description"});
    auto robot = urdf::parseURDF(params[0].as_string());

    /* ---- IMPORTANT: use arm_link_4, NOT tool0 ---- */
    if (!kdl_parser::treeFromUrdfModel(*robot, tree_) ||
        !tree_.getChain("arm_base_link", "arm_link_4", chain_))
    {
      RCLCPP_FATAL(get_logger(), "Failed to build KDL chain");
      rclcpp::shutdown();
      return;
    }

    nj_ = chain_.getNrOfJoints();

    fk_solver_ =
      std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);

    marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>(
        "/arm/workspace_cloud", 1);

    RCLCPP_INFO(
      get_logger(),
      "Workspace FK node ready | joints=%d", nj_);

    computeAndPublish();
  }

private:
  /* -------------------------------------------------- */

  void computeAndPublish()
  {
    visualization_msgs::msg::Marker workspace;
    initMarker(workspace);

    KDL::JntArray q(nj_);
    KDL::Frame ee;

    /* ---- joint limits from URDF ---- */
    std::vector<double> q_min = {
      -3.14,    // base
      -1.57,    // shoulder
       0.00,    // elbow
      -3.14     // wrist
    };

    std::vector<double> q_max = {
       3.14,
       1.57,
       3.10,
       3.14
    };

    /* ---- sampling resolution ---- */
    std::vector<double> step = {
      0.25,   // base
      0.20,   // shoulder
      0.20,   // elbow
      0.30    // wrist
    };

    size_t count = 0;

    for (double q0 = q_min[0]; q0 <= q_max[0]; q0 += step[0])
    for (double q1 = q_min[1]; q1 <= q_max[1]; q1 += step[1])
    for (double q2 = q_min[2]; q2 <= q_max[2]; q2 += step[2])
    for (double q3 = q_min[3]; q3 <= q_max[3]; q3 += step[3])
    {
      q(0) = q0;
      q(1) = q1;
      q(2) = q2;
      q(3) = q3;

      if (fk_solver_->JntToCart(q, ee) < 0)
        continue;

      geometry_msgs::msg::Point p;
      p.x = ee.p.x();
      p.y = ee.p.y();
      p.z = ee.p.z();

      workspace.points.push_back(p);
      count++;
    }

    marker_pub_->publish(workspace);

    RCLCPP_INFO(
      get_logger(),
      "Workspace published | points=%zu", count);
  }

  /* -------------------------------------------------- */

  void initMarker(visualization_msgs::msg::Marker &m)
  {
    m.header.frame_id = "arm_base_link";
    m.header.stamp = now();
    m.ns = "workspace";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::POINTS;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.scale.x = 0.015;
    m.scale.y = 0.015;

    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 0.8;
  }

  /* -------- members -------- */

  rclcpp::SyncParametersClient::SharedPtr param_client_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  KDL::Tree tree_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  unsigned nj_{0};
};

/* -------------------------------------------------- */

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KDLWorkspaceCloud>());
  rclcpp::shutdown();
  return 0;
}
