#pragma once

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <string>

namespace ebot_hardware
{

class EbotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EbotHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override;

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // joint bookkeeping
  std::vector<std::string> joint_names_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  // limits
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
};

}  // namespace ebot_hardware
