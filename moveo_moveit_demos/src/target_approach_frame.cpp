#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <cmath>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class TargetApproachNode : public rclcpp::Node
{
public:
    TargetApproachNode()
    : Node("target_approach_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      tf_broadcaster_(this)
    {
        this->declare_parameter("target_frame", "target_frame");

        nav_client_ =
            rclcpp_action::create_client<NavigateToPose>(
                this, "navigate_to_pose");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TargetApproachNode::updateFrame, this));
    }

private:
    void updateFrame()
    {
        std::string target_frame =
            this->get_parameter("target_frame").as_string();

        geometry_msgs::msg::TransformStamped map_to_target;

        try
        {
            map_to_target = tf_buffer_.lookupTransform(
                "map", target_frame, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "TF not found: %s", ex.what());
            return;
        }

        // ---------------------------------------
        // Publish:
        // target_frame -> base_target_frame
        // offset local z = -0.50
        // ---------------------------------------
        geometry_msgs::msg::TransformStamped child_tf;

        child_tf.header.stamp = now();
        child_tf.header.frame_id = target_frame;
        child_tf.child_frame_id = "base_target_frame";

        child_tf.transform.translation.x = 0.0;
        child_tf.transform.translation.y = 0.0;
        child_tf.transform.translation.z = -0.50;

        child_tf.transform.rotation.x = 0.0;
        child_tf.transform.rotation.y = 0.0;
        child_tf.transform.rotation.z = 0.0;
        child_tf.transform.rotation.w = 1.0;

        tf_broadcaster_.sendTransform(child_tf);

        // ---------------------------------------
        // Lookup map -> base_target_frame
        // ---------------------------------------
        geometry_msgs::msg::TransformStamped map_to_base_target;

        try
        {
            map_to_base_target = tf_buffer_.lookupTransform(
                "map", "base_target_frame", tf2::TimePointZero);
        }
        catch (tf2::TransformException &)
        {
            return;
        }

        double bx = map_to_base_target.transform.translation.x;
        double by = map_to_base_target.transform.translation.y;

        double tx = map_to_target.transform.translation.x;
        double ty = map_to_target.transform.translation.y;

        // Face robot toward target
        double yaw = std::atan2(ty - by, tx - bx);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Goal -> x=%.2f y=%.2f yaw=%.2f deg",
            bx, by, yaw * 180.0 / M_PI);

        // ---------------------------------------
        // Send Nav2 Goal only once
        // ---------------------------------------
        if (!goal_sent_)
        {
            if (!nav_client_->wait_for_action_server(
                    std::chrono::seconds(2)))
            {
                RCLCPP_WARN(
                    get_logger(),
                    "NavigateToPose action server not ready");
                return;
            }

            NavigateToPose::Goal goal_msg;

            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.header.stamp = now();

            goal_msg.pose.pose.position.x = bx;
            goal_msg.pose.pose.position.y = by;
            goal_msg.pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);

            goal_msg.pose.pose.orientation.x = q.x();
            goal_msg.pose.pose.orientation.y = q.y();
            goal_msg.pose.pose.orientation.z = q.z();
            goal_msg.pose.pose.orientation.w = q.w();

            nav_client_->async_send_goal(goal_msg);

            RCLCPP_INFO(get_logger(), "Nav2 goal sent.");
            goal_sent_ = true;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

    bool goal_sent_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetApproachNode>());
    rclcpp::shutdown();
    return 0;
}