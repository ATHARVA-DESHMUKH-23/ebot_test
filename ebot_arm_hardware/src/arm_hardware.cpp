#include "ebot_arm_hardware/arm_hardware.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sstream>

namespace ebot_arm_hardware
{

// -------------------------
// Helper: baudrate mapping
// -------------------------
static speed_t baud_to_flag(int baud)
{
  switch (baud)
  {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return B115200;
  }
}

hardware_interface::CallbackReturn
ArmHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  for (const auto & joint : info.joints)
  {
    joint_names_.push_back(joint.name);

    double lower = -1e9;
    double upper =  1e9;

    if (!joint.command_interfaces.empty())
    {
      if (!joint.command_interfaces[0].min.empty())
        lower = std::stod(joint.command_interfaces[0].min);

      if (!joint.command_interfaces[0].max.empty())
        upper = std::stod(joint.command_interfaces[0].max);
    }

    lower_limits_.push_back(lower);
    upper_limits_.push_back(upper);
  }

  size_t n = joint_names_.size();
  hw_positions_.assign(n, 0.0);
  hw_commands_.assign(n, 0.0);

  port_ = info.hardware_parameters.at("port");
  baudrate_ = std::stoi(info.hardware_parameters.at("baudrate"));

  RCLCPP_INFO(rclcpp::get_logger("ArmHardware"),
              "Initialized arm hardware with %zu joints", n);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    states.emplace_back(joint_names_[i], "position", &hw_positions_[i]);
  }
  return states;
}

std::vector<hardware_interface::CommandInterface>
ArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmds;
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    cmds.emplace_back(joint_names_[i], "position", &hw_commands_[i]);
  }
  return cmds;
}

hardware_interface::CallbackReturn
ArmHardware::on_activate(const rclcpp_lifecycle::State &)
{
  fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArmHardware"),
                 "Failed to open serial port %s", port_.c_str());
    return CallbackReturn::ERROR;
  }

  termios tty{};
  tcgetattr(fd_, &tty);

  speed_t speed = baud_to_flag(baudrate_);
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tcsetattr(fd_, TCSANOW, &tty);

  hw_commands_ = hw_positions_;  // hold position

  RCLCPP_INFO(rclcpp::get_logger("ArmHardware"),
              "Serial port %s opened", port_.c_str());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArmHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (fd_ >= 0)
  {
    close(fd_);
    fd_ = -1;
  }
  return CallbackReturn::SUCCESS;
}

int ArmHardware::rad_to_servo(double rad,
                             double rad_min, double rad_max,
                             int servo_min, int servo_max)
{
  rad = std::clamp(rad, rad_min, rad_max);
  double ratio = (rad - rad_min) / (rad_max - rad_min);
  return servo_min + ratio * (servo_max - servo_min);
}

hardware_interface::return_type
ArmHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  hw_positions_ = hw_commands_;  // open-loop
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
ArmHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (fd_ < 0) return hardware_interface::return_type::OK;

  for (size_t i = 0; i < hw_commands_.size(); i++)
  {
    hw_commands_[i] = std::clamp(
      hw_commands_[i],
      lower_limits_[i],
      upper_limits_[i]);
  }

  // base, shoulder, elbow, wrist
  int s0 = rad_to_servo(hw_commands_[0], -M_PI, M_PI, 0, 180);
  int s1 = rad_to_servo(hw_commands_[1], -1.57, 1.57, 10, 170);
  int s2 = rad_to_servo(hw_commands_[2], 0.0, 3.1, 20, 160);
  int s3 = rad_to_servo(hw_commands_[3], -M_PI, M_PI, 0, 180);

  std::stringstream ss;
  ss << "#A," << s0 << "," << s1 << "," << s2 << "," << s3 << "\n";

  ::write(fd_, ss.str().c_str(), ss.str().size());

  return hardware_interface::return_type::OK;
}

}  // namespace ebot_arm_hardware

PLUGINLIB_EXPORT_CLASS(
  ebot_arm_hardware::ArmHardware,
  hardware_interface::SystemInterface)
