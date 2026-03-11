#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
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

class KDLCartesianController : public rclcpp::Node
{
public:
  KDLCartesianController()
  : Node("kdl_cartesian_controller")
  {
    param_client_ =
      std::make_shared<rclcpp::SyncParametersClient>(
        this, "robot_state_publisher");

    while (!param_client_->wait_for_service(1s))
      RCLCPP_INFO(get_logger(), "Waiting for robot_state_publisher...");

    auto params = param_client_->get_parameters({"robot_description"});
    auto robot = urdf::parseURDF(params[0].as_string());

    if (!kdl_parser::treeFromUrdfModel(*robot, tree_) ||
        !tree_.getChain("moveo_base_link", "Link_5", chain_))
    {
      RCLCPP_FATAL(get_logger(), "Failed to build KDL chain");
      rclcpp::shutdown();
      return;
    }

    fk_solver_  = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);

    RCLCPP_INFO(get_logger(), "KDL ready | joints=%d", chain_.getNrOfJoints());
    for(unsigned i=0;i<chain_.getNrOfSegments();i++)
      {
          auto j = chain_.getSegment(i).getJoint();
          RCLCPP_INFO(get_logger(),"Joint: %s Type: %d",
              j.getName().c_str(), j.getType());
      }

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&KDLCartesianController::jointCB, this, std::placeholders::_1));

    pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
      "/arm/cartesian_target", 10,
      std::bind(&KDLCartesianController::targetCB, this, std::placeholders::_1));

    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10);
  }

private:

  void jointCB(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_names_ = msg->name;
    joint_pos_   = msg->position;
    joints_ok_   = true;

    RCLCPP_INFO_ONCE(get_logger(),
    "✅ Received joint_states (%ld joints)",
    joint_pos_.size());
  }

  void targetCB(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    if (!joints_ok_ || executing_)
      return;
  
    if (!joints_ok_) {
      RCLCPP_WARN(get_logger(), "❌ No joint_states yet");
      return;
    }
    if (executing_) {
      RCLCPP_WARN(get_logger(), "⚠ IK already executing");
      return;
    }
    executing_ = true;

    KDL::Vector p(msg->position.x,
                  msg->position.y,
                  msg->position.z);

    KDL::Rotation R =
      KDL::Rotation::Quaternion(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    target_frame_ = KDL::Frame(R, p);

    solveIK();
    executing_ = false;
  }

  void solveIK()
  {
    RCLCPP_INFO(get_logger(), "🚀 Starting IK solve");
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

    for (int iter = 0; iter < 100000; iter++)
    {
      KDL::Frame current;
      fk_solver_->JntToCart(q, current);

      double cr, cp, cy;
      current.M.GetRPY(cr, cp, cy);

      double tr, tp, ty;
      target_frame_.M.GetRPY(tr, tp, ty);

      RCLCPP_INFO(get_logger(),
        "Current RPY: %.3f %.3f %.3f | Target RPY: %.3f %.3f %.3f",
        cr, cp, cy, tr, tp, ty);


      // --- Position error ---
      KDL::Vector dp = target_frame_.p - current.p;

      // --- Orientation error (axis-angle) ---
      KDL::Rotation R_err =
         target_frame_.M * current.M.Inverse();

      double ex = 0.5 * (R_err(2,1) - R_err(1,2));
      double ey = 0.5 * (R_err(0,2) - R_err(2,0));
      double ez = 0.5 * (R_err(1,0) - R_err(0,1));

      KDL::Vector dw(ex, ey, ez);

      Eigen::VectorXd err(6);
      err << dp.x(), dp.y(), dp.z(),
            dw.x(), dw.y(), dw.z();

      double rot_error_norm = dw.Norm();

      if (dp.Norm() < 0.002 && rot_error_norm < 0.005)
          break;


      KDL::Jacobian J_kdl(chain_.getNrOfJoints());
      jac_solver_->JntToJac(q, J_kdl);

      Eigen::MatrixXd J(6, chain_.getNrOfJoints());
      for (unsigned r = 0; r < 6; r++)
        for (unsigned c = 0; c < chain_.getNrOfJoints(); c++)
          J(r,c) = J_kdl(r,c);

      double lambda = 0.01;

      Eigen::MatrixXd J_pinv =
          J.transpose() *
          (J * J.transpose()
           + lambda * Eigen::MatrixXd::Identity(6,6))
          .inverse();

      Eigen::VectorXd qdot = J_pinv * err;

      for (unsigned i = 0; i < q.rows(); i++)
        q(i) += qdot(i) * 0.05;
      
      if (iter % 50 == 0)
      {
        RCLCPP_INFO(get_logger(),
          "Iter %d | Pos error: %.4f | Rot error: %.4f",
          iter,
          dp.Norm(),
          rot_error_norm);
      }


    }
    RCLCPP_INFO(get_logger(), "✅ IK finished");

    publishTrajectory(q);
  }

  void publishTrajectory(const KDL::JntArray &q)
  {
    trajectory_msgs::msg::JointTrajectory traj;

    
    for (unsigned i = 0; i < chain_.getNrOfSegments(); ++i)
    {
      const auto & joint = chain_.getSegment(i).getJoint();
      if (joint.getType() != KDL::Joint::None)
        traj.joint_names.push_back(joint.getName());
    }

    trajectory_msgs::msg::JointTrajectoryPoint pt;

    for (unsigned i = 0; i < q.rows(); i++)
      pt.positions.push_back(q(i));

    pt.time_from_start = rclcpp::Duration::from_seconds(1.5);
    traj.points.push_back(pt);
    RCLCPP_INFO(get_logger(),
    "📤 Publishing trajectory with %ld joints",
    traj.joint_names.size());

    traj_pub_->publish(traj);
  }

  rclcpp::SyncParametersClient::SharedPtr param_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

  KDL::Tree tree_;
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  std::vector<std::string> joint_names_;
  std::vector<double> joint_pos_;

  KDL::Frame target_frame_;

  bool joints_ok_{false};
  bool executing_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KDLCartesianController>());
  rclcpp::shutdown();
  return 0;
}
