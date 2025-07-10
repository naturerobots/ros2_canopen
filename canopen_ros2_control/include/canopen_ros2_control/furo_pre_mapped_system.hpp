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
#ifndef CANOPEN_ROS2_CONTROL__FURO_PRE_MAPPED_SYSTEM_HPP_
#define CANOPEN_ROS2_CONTROL__FURO_PRE_MAPPED_SYSTEM_HPP_

#include "canopen_pre_mapped_driver/pre_mapped_driver.hpp"
#include "canopen_ros2_control/canopen_system.hpp"

namespace canopen_ros2_control
{

struct MotorNodeData
{
  // feedback
  double actual_position;
  double actual_speed;

  // setpoint
  double target_velocity;
};

using namespace ros2_canopen;
class FuroPreMappedSystem : public CanopenSystem
{
public:
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  FuroPreMappedSystem();
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  ~FuroPreMappedSystem() = default;
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces();

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period);

protected:
  // Map to store motor nodes and their data
  std::map<std::string, MotorNodeData> motor_nodes_;

// private:
//   void initDeviceContainer();

};  // class FuroPreMappedSystem

}  // namespace canopen_ros2_control

#endif  // CANOPEN_ROS2_CONTROL__FURO_PRE_MAPPED_SYSTEM_HPP_
