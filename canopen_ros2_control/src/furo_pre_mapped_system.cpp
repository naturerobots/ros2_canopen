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


namespace
{
auto const kLogger = rclcpp::get_logger("FuroPreMappedSystem");
}

namespace canopen_ros2_control
{

FuroPreMappedSystem::FuroPreMappedSystem()
: CanopenSystem()
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
  auto ret_val = CanopenSystem::on_configure(previous_state);
  return ret_val;
}

std::vector<hardware_interface::StateInterface> FuroPreMappedSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  //underlying base class export first
  state_interfaces = CanopenSystem::export_state_interfaces();

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FuroPreMappedSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces = CanopenSystem::export_command_interfaces();

  return command_interfaces;
}

hardware_interface::CallbackReturn FuroPreMappedSystem::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret_val = CanopenSystem::on_activate(previous_state);
  return ret_val;
}

hardware_interface::CallbackReturn FuroPreMappedSystem::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret_val = CanopenSystem::on_deactivate(previous_state);
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
  auto ret_val = CanopenSystem::write(time, period);
  return ret_val;
}


} // namespace canopen_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  canopen_ros2_control::FuroPreMappedSystem,
  hardware_interface::SystemInterface)
