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

  hw_positions_.resize(5, 0.0);
  hw_commands_.resize(5, 0.0);
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
  tty.c_cc[VTIME] = 10;

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

  for (size_t i = 0; i < 5; ++i)
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

  for (size_t i = 0; i < 5; ++i)
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

  long s1 = rad_to_steps(hw_commands_[idx1], 0);
  long s2 = rad_to_steps(hw_commands_[idx2], 1);
  long s3 = rad_to_steps(hw_commands_[idx3], 2);
  long s4 = rad_to_steps(hw_commands_[idx4], 3);
  long s5 = rad_to_steps(hw_commands_[idx5], 4);

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

  if (changed)
  {
    snprintf(buffer, sizeof(buffer),
            "CMD %ld %ld %ld %ld %ld\n",
            s1, s2, s3, s4, s5);

    ::write(serial_fd_, buffer, strlen(buffer));

    for (int i = 0; i < 5; i++)
      last_steps_[i] = steps[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MoveoSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (serial_fd_ < 0)
    return hardware_interface::return_type::ERROR;

  char buffer[128];
  int n = ::read(serial_fd_, buffer, sizeof(buffer) - 1);

  if (n <= 0)
  {
    return hardware_interface::return_type::OK;
  }

  if (n > 0)
  {
    buffer[n] = '\0';
    RCLCPP_INFO(rclcpp::get_logger("MoveoSystem"),
                "Received raw: %s", buffer);

    long s1, s2, s3, s4, s5;

    if (sscanf(buffer, "FB %ld %ld %ld %ld %ld",
               &s1, &s2, &s3, &s4, &s5) == 5)
    {
      hw_positions_[joint_name_to_index_["Joint_1"]] = steps_to_rad(s1, 0);
      hw_positions_[joint_name_to_index_["Joint_2"]] = steps_to_rad(s2, 1);
      hw_positions_[joint_name_to_index_["Joint_3"]] = steps_to_rad(s3, 2);
      hw_positions_[joint_name_to_index_["Joint_4"]] = steps_to_rad(s4, 3);
      hw_positions_[joint_name_to_index_["Joint_5"]] = steps_to_rad(s5, 4);
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace moveo_hardware

PLUGINLIB_EXPORT_CLASS(
    moveo_hardware::MoveoSystem,
    hardware_interface::SystemInterface)