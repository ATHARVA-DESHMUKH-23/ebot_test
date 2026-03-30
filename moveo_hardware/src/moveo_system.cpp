#include "moveo_hardware/moveo_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

namespace moveo_hardware
{

// ======== CONFIGURATION ========

// Adjust these values according to your Moveo setup
static const double STEPS_PER_REV = 200.0;      // Motor native steps
static const double MICROSTEPPING = 16.0;        // TB6560 DIP switch value
static const double GEAR_RATIO[5] = {1.0, 1.0, 1.0, 1.0, 1.0};  // Example ratios

// ================================

hardware_interface::CallbackReturn MoveoSystem::on_init(
    const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    joint_name_to_index_[info_.joints[i].name] = i;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoveoSystem::on_activate(
    const rclcpp_lifecycle::State &)
{
  serial_fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);

  if (serial_fd_ < 0)
  {
    std::cerr << "Failed to open serial port\n";
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct termios tty;
  memset(&tty, 0, sizeof tty);

  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    std::cerr << "Error getting serial attributes\n";
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_iflag = 0;

  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  tcsetattr(serial_fd_, TCSANOW, &tty);

  std::cout << "Serial connected\n";

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MoveoSystem::on_deactivate(
    const rclcpp_lifecycle::State &)
{
  if (serial_fd_ > 0)
    close(serial_fd_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MoveoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_positions_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MoveoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_commands_[i]));
  }

  return command_interfaces;
}

// ======== Conversion Functions ========

long rad_to_steps(double rad, int joint_index)
{
  double steps = rad *
                 (STEPS_PER_REV * MICROSTEPPING * GEAR_RATIO[joint_index]) /
                 (2.0 * M_PI);

  return static_cast<long>(steps);
}

double steps_to_rad(long steps, int joint_index)
{
  return (steps * 2.0 * M_PI) /
         (STEPS_PER_REV * MICROSTEPPING * GEAR_RATIO[joint_index]);
}

// =======================================

hardware_interface::return_type MoveoSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (serial_fd_ < 0)
    return hardware_interface::return_type::ERROR;

  char buffer[128];

  size_t idx1 = joint_name_to_index_["Joint_1"];
  size_t idx2 = joint_name_to_index_["Joint_2"];
  size_t idx3 = joint_name_to_index_["Joint_3"];
  size_t idx4 = joint_name_to_index_["Joint_4"];
  size_t idx5 = joint_name_to_index_["Joint_5"];
  size_t grip_idx = joint_name_to_index_["Gripper_Servo_Gear_Joint"];
  size_t grip2_idx = joint_name_to_index_["Gripper_Idol_Gear_Joint"];
  hw_commands_[grip2_idx] = -hw_commands_[grip_idx];
  long s1 = rad_to_steps(hw_commands_[idx1], 0);
  long s2 = rad_to_steps(hw_commands_[idx2], 1);
  long s3 = rad_to_steps(hw_commands_[idx3], 2);
  long s4 = rad_to_steps(hw_commands_[idx4], 3);
  long s5 = rad_to_steps(hw_commands_[idx5], 4);
  double grip_rad = hw_commands_[grip_idx];
  int grip_val = static_cast<int>(grip_rad * 180.0 / M_PI);

  long steps[5] = {s1, s2, s3, s4, s5};

  bool changed = false;

  for (int i = 0; i < 5; i++)
  {
    if (steps[i] != last_steps_[i])
    {
      changed = true;
      break;
    }
  }
  // check gripper
  if (grip_val != last_gripper_)
  {
    changed = true;
  }

  if (changed)
  {
    snprintf(buffer, sizeof(buffer),
            "CMD %ld %ld %ld %ld %ld G %d\n",
            s1, s2, s3, s4, s5, grip_val);

    ::write(serial_fd_, buffer, strlen(buffer));

    for (int i = 0; i < 5; i++)
      last_steps_[i] = steps[i];
    
    last_gripper_ = grip_val;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoveoSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (serial_fd_ < 0)
    return hardware_interface::return_type::ERROR;

  static std::string serial_buffer;   // ADD THIS (top of function or static)

  char temp[128];
  int n = ::read(serial_fd_, temp, sizeof(temp));

  if (n <= 0)
  {
    return hardware_interface::return_type::OK;
  }

  // append new data
  serial_buffer.append(temp, n);

  // process full lines only
  size_t pos;
  while ((pos = serial_buffer.find('\n')) != std::string::npos)
  {
    std::string line = serial_buffer.substr(0, pos);
    serial_buffer.erase(0, pos + 1);

    long s1, s2, s3, s4, s5;
    int grip;
    size_t grip_idx = joint_name_to_index_["Gripper_Servo_Gear_Joint"];
    size_t grip2_idx = joint_name_to_index_["Gripper_Idol_Gear_Joint"];

    

    if (sscanf(line.c_str(), "FB %ld %ld %ld %ld %ld G %d",
              &s1, &s2, &s3, &s4, &s5, &grip) == 6)
    {
      hw_positions_[joint_name_to_index_["Joint_1"]] = steps_to_rad(s1, 0);
      hw_positions_[joint_name_to_index_["Joint_2"]] = steps_to_rad(s2, 1);
      hw_positions_[joint_name_to_index_["Joint_3"]] = steps_to_rad(s3, 2);
      hw_positions_[joint_name_to_index_["Joint_4"]] = steps_to_rad(s4, 3);
      hw_positions_[joint_name_to_index_["Joint_5"]] = steps_to_rad(s5, 4);
      double grip_rad = grip * M_PI / 180.0;
      hw_positions_[grip_idx] = grip_rad;
      hw_positions_[grip2_idx] = -grip_rad;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace moveo_hardware

PLUGINLIB_EXPORT_CLASS(
    moveo_hardware::MoveoSystem,
    hardware_interface::SystemInterface)