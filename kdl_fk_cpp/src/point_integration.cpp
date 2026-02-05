#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class PickPointIntegrator : public rclcpp::Node
{
public:
  PickPointIntegrator()
  : Node("pick_point_integrator")
  {
    /* ---------- Initial Pick Point ---------- */
    pick_point_.header.frame_id = "odom";
    pick_point_.point.x = 0.0;
    pick_point_.point.y = 0.0;
    pick_point_.point.z = 0.55;   // IMPORTANT: not starting from 0

    /* ---------- Subscriber ---------- */
    vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/pick_velocity", 10,
      std::bind(&PickPointIntegrator::velCB, this, std::placeholders::_1));

    /* ---------- Publisher ---------- */
    pick_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/pick_object", 10);

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/pick_marker_target", 10);


    /* ---------- Timer ---------- */
    timer_ = create_wall_timer(
      50ms, std::bind(&PickPointIntegrator::update, this));

    last_time_ = now();

    RCLCPP_INFO(get_logger(),
      "📍 Pick point integrator started at (0.0, 0.0, 0.55)");
  }

private:
  /* ---------- Velocity Callback ---------- */
  void velCB(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    vz_ = msg->linear.z;
  }

  /* ---------- Timer Update ---------- */
  void update()
  {
    rclcpp::Time current = now();
    double dt = (current - last_time_).seconds();
    last_time_ = current;

    // Integrate velocity
    pick_point_.point.x += vx_ * dt;
    pick_point_.point.y += vy_ * dt;
    pick_point_.point.z += vz_ * dt;

    // Optional safety clamp (recommended)
    pick_point_.point.z = std::clamp(pick_point_.point.z, 0.2, 1.0);

    pick_point_.header.stamp = current;
    pick_pub_->publish(pick_point_);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = current;
    marker.ns = "pick_target";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = pick_point_.point;
    marker.pose.orientation.w = 1.0;

    // Size of the blob
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;

    // Yellow color
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_pub_->publish(marker);

  }

  /* ---------- Members ---------- */
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pick_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  geometry_msgs::msg::PointStamped pick_point_;

  double vx_{0.0}, vy_{0.0}, vz_{0.0};
  rclcpp::Time last_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PickPointIntegrator>());
  rclcpp::shutdown();
  return 0;
}
