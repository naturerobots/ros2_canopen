//    Copyright 2025 Georg John, Nature Robots GmbH
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
#include "canopen_ros2_control/furo_pre_mapped_system.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace
{
auto const kLogger = rclcpp::get_logger("FuroPreMappedSystem");
}

namespace canopen_ros2_control
{

FuroPreMappedSystem::FuroPreMappedSystem()
: CanopenSystem(), motor_running_(false), rollover(0)
{
}

hardware_interface::CallbackReturn FuroPreMappedSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (CanopenSystem::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FuroPreMappedSystem::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  return CanopenSystem::on_configure(previous_state);
}

std::vector<hardware_interface::StateInterface> FuroPreMappedSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // underlying base class export first
  state_interfaces = CanopenSystem::export_state_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++) {

    auto joint_name = info_.joints[i].name;

    // actual position
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, hardware_interface::HW_IF_POSITION,
        &motor_data_[joint_name].actual_position));
    // actual speed
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, hardware_interface::HW_IF_VELOCITY,
        &motor_data_[joint_name].actual_speed));

  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FuroPreMappedSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // underlying base class export first
  command_interfaces = CanopenSystem::export_command_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++) {
    // target
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &motor_data_[info_.joints[i].name].target_velocity));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn FuroPreMappedSystem::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret_val = CanopenSystem::on_activate(previous_state);
  // auto drivers = device_container_->get_registered_drivers();
  motor_running_ = true; // Set the motor running flag to true on activation
  rollover = 0; // Reset rollover on activation
  RCLCPP_INFO_STREAM(kLogger, "FuroPreMappedSystem ACTIVATED");
  return ret_val;
}

hardware_interface::CallbackReturn FuroPreMappedSystem::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  motor_running_ = false;
  auto drivers = device_container_->get_registered_drivers();
  for (auto it = drivers.begin(); it != drivers.end(); ++it) {
    auto driver = std::dynamic_pointer_cast<ros2_canopen::PreMappedDriver>(it->second);
    if (driver) {
      driver->get_node_canopen_driver_interface()->deactivate();
    }
  }
  auto ret_val = CanopenSystem::on_deactivate(previous_state);
  RCLCPP_INFO_STREAM(kLogger, "FuroPreMappedSystem DEACTIVATED");
  return ret_val;
}

hardware_interface::return_type FuroPreMappedSystem::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  auto ret_val = CanopenSystem::read(time, period);
  return ret_val;
}

hardware_interface::return_type FuroPreMappedSystem::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  // auto ret_val = CanopenSystem::write(time, period);
  auto drivers = device_container_->get_registered_drivers();
  for (auto it = drivers.begin(); it != drivers.end(); ++it) {
    auto pre_mapped_driver = std::dynamic_pointer_cast<ros2_canopen::PreMappedDriver>(it->second);
    if (pre_mapped_driver != nullptr && motor_running_) {

      auto joint_name = pre_mapped_driver->get_motor_joint_name();
      auto target_velocity = motor_data_[joint_name].target_velocity;
      // pre_mapped_driver->set_target(target_velocity, rollover);
    }
  }
  rollover++;
  return hardware_interface::return_type::OK;
}


} // namespace canopen_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  canopen_ros2_control::FuroPreMappedSystem,
  hardware_interface::SystemInterface)
