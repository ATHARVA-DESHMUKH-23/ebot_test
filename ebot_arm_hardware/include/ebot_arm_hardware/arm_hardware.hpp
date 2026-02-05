#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <string>

namespace ebot_arm_hardware
{

class ArmHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // Serial
  int fd_ = -1;
  std::string port_;
  int baudrate_;

  // Joint helpers
  int rad_to_servo(double rad, double rad_min, double rad_max,
                   int servo_min, int servo_max);

  // Joints
  std::vector<std::string> joint_names_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_commands_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
};

}  // namespace ebot_arm_hardware
