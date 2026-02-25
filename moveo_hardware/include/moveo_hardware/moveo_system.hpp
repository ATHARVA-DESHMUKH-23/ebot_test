#ifndef MOVEO_SYSTEM_HPP
#define MOVEO_SYSTEM_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <unordered_map>   // ✅ ADDED

namespace moveo_hardware
{

class MoveoSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  int serial_fd_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_commands_;

  long last_steps_[5] = {0, 0, 0, 0, 0};

  std::unordered_map<std::string, size_t> joint_name_to_index_;   // ✅ ADDED
};

}

#endif