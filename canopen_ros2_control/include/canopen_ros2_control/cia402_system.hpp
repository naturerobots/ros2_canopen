// Copyright (c) 2024, Malte kleine Piening, Nature Robots GmbH
// Copyright (c) 2022, StoglRobotics
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2022-08-01
 *
 */
//----------------------------------------------------------------------

#ifndef CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_
#define CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_

#include "canopen_402_driver/cia402_driver.hpp"
#include "canopen_ros2_control/canopen_system.hpp"

constexpr double kResponseOk = 1.0;
constexpr double kResponseFail = 0.0;

namespace canopen_ros2_control
{

struct MotorNodeData
{
  // feedback
  double actual_position;
  double actual_speed;

  // mode control
  uint16_t operation_mode;

  // setpoint
  double target_position;
  double target_velocity;
  double target_torque;
};

using namespace ros2_canopen;
class Cia402System : public CanopenSystem
{
public:
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  Cia402System();
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  ~Cia402System() = default;
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces();

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period);

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period);

protected:
  // motor data for each registered joint name
  std::map<std::string, MotorNodeData> motor_data_;

private:
  void switchModes(uint id, const std::shared_ptr<ros2_canopen::Cia402Driver>& driver);

  void handleInit(uint id, const std::shared_ptr<ros2_canopen::Cia402Driver>& driver);

  void handleRecover(uint id, const std::shared_ptr<ros2_canopen::Cia402Driver>& driver);

  void handleHalt(uint id, const std::shared_ptr<ros2_canopen::Cia402Driver>& driver);

  void initDeviceContainer();
};

}  // namespace canopen_ros2_control

#endif  // CANOPEN_ROS2_CONTROL__CIA402_SYSTEM_HPP_
