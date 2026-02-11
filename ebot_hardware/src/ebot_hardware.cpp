#include "ebot_hardware/ebot_hardware.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <algorithm>

namespace ebot_hardware
{

hardware_interface::CallbackReturn
EbotHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  for (const auto & joint : info.joints)
  {
    joint_names_.push_back(joint.name);
    lower_limits_.push_back(-1e9);
    upper_limits_.push_back( 1e9);
  }

  size_t n = joint_names_.size();
  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  hw_commands_.assign(n, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("EbotHardware"),
              "Initialized with %zu joints", n);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
EbotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;

  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    states.emplace_back(joint_names_[i], "position", &hw_positions_[i]);
    states.emplace_back(joint_names_[i], "velocity", &hw_velocities_[i]);
  }

  return states;
}

std::vector<hardware_interface::CommandInterface>
EbotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmds;

  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    cmds.emplace_back(joint_names_[i], "position", &hw_commands_[i]);
    cmds.emplace_back(joint_names_[i], "velocity", &hw_commands_[i]);
  }

  return cmds;
}

hardware_interface::CallbackReturn
EbotHardware::on_activate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
EbotHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
EbotHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
EbotHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

}  // namespace ebot_hardware

PLUGINLIB_EXPORT_CLASS(
  ebot_hardware::EbotHardware,
  hardware_interface::SystemInterface)
