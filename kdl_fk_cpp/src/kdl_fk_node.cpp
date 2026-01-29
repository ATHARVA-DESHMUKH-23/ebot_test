#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <algorithm>
#include <vector>
#include <string>

using namespace std::chrono_literals;

void printChain(const KDL::Chain & chain, rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "========== KDL CHAIN ==========");

  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
  {
    const auto & seg = chain.getSegment(i);
    const auto & joint = seg.getJoint();
    const auto & frame = seg.getFrameToTip();

    std::string joint_type;
    switch (joint.getType())
    {
      case KDL::Joint::RotX: joint_type = "RotX"; break;
      case KDL::Joint::RotY: joint_type = "RotY"; break;
      case KDL::Joint::RotZ: joint_type = "RotZ"; break;
      case KDL::Joint::RotAxis: joint_type = "RotAxis"; break;
      case KDL::Joint::None: joint_type = "Fixed"; break;
      default: joint_type = "Other"; break;
    }

    RCLCPP_INFO(
      logger,
      "Segment %d | link=%s | joint=%s | type=%s | xyz=[%.3f %.3f %.3f]",
      i,
      seg.getName().c_str(),
      joint.getName().c_str(),
      joint_type.c_str(),
      frame.p.x(),
      frame.p.y(),
      frame.p.z()
    );
  }

  RCLCPP_INFO(logger, "===============================");
}

class KDLFKNode : public rclcpp::Node
{
public:
  KDLFKNode() : Node("kdl_fk_node")
  {
    RCLCPP_INFO(get_logger(), "Waiting for robot_state_publisher...");

    param_client_ =
      std::make_shared<rclcpp::SyncParametersClient>(
        this, "robot_state_publisher");

    while (!param_client_->wait_for_service(1s)) {
      RCLCPP_INFO(get_logger(), "Waiting for parameter service...");
    }

    auto params = param_client_->get_parameters({"robot_description"});
    std::string robot_desc = params[0].as_string();

    if (robot_desc.empty()) {
      RCLCPP_FATAL(get_logger(), "robot_description is empty");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "robot_description received");

    auto robot = urdf::parseURDF(robot_desc);

    if (!kdl_parser::treeFromUrdfModel(*robot, tree_)) {
      RCLCPP_FATAL(get_logger(), "Failed to create KDL tree");
      rclcpp::shutdown();
      return;
    }

    if (!tree_.getChain("arm_base_link", "tool0", chain_)) {
      RCLCPP_FATAL(get_logger(), "Failed to extract KDL chain");
      rclcpp::shutdown();
      return;
    }

    fk_solver_ =
      std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);

    RCLCPP_INFO(
      get_logger(),
      "KDL chain joints = %d",
      chain_.getNrOfJoints());

    printChain(chain_, get_logger());

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&KDLFKNode::jointCallback, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(
      1s, std::bind(&KDLFKNode::computeFK, this)
    );
  }

private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_names_ = msg->name;
    joint_positions_ = msg->position;
    joints_received_ = true;
  }

  void computeFK()
  {
    if (!joints_received_) {
      RCLCPP_WARN(get_logger(), "Waiting for joint_states...");
      return;
    }

    KDL::JntArray q(chain_.getNrOfJoints());
    unsigned int idx = 0;

    for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i)
    {
      const auto & joint = chain_.getSegment(i).getJoint();
      if (joint.getType() == KDL::Joint::None)
        continue;

      auto it = std::find(
        joint_names_.begin(),
        joint_names_.end(),
        joint.getName());

      if (it != joint_names_.end()) {
        q(idx) = joint_positions_[std::distance(joint_names_.begin(), it)];
      } else {
        q(idx) = 0.0;
      }
      idx++;
    }

    KDL::Frame frame;
    fk_solver_->JntToCart(q, frame);

    RCLCPP_INFO(
      get_logger(),
      "LIVE FK → x=%.3f y=%.3f z=%.3f",
      frame.p.x(),
      frame.p.y(),
      frame.p.z()
    );
  }

  rclcpp::SyncParametersClient::SharedPtr param_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  KDL::Tree tree_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  bool joints_received_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KDLFKNode>());
  rclcpp::shutdown();
  return 0;
}
