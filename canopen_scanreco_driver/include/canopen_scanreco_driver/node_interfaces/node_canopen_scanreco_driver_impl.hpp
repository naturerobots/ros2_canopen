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
#ifndef NODE_CANOPEN_SCANRECO_DRIVER_IMPL_HPP_
#define NODE_CANOPEN_SCANRECO_DRIVER_IMPL_HPP_

#include "canopen_scanreco_driver/node_interfaces/node_canopen_scanreco_driver.hpp"

using namespace ros2_canopen::node_interfaces;

template<class NODETYPE>
NodeCanopenScanrecoDriver<NODETYPE>::NodeCanopenScanrecoDriver(NODETYPE * node)
: ros2_canopen::node_interfaces::NodeCanopenProxyDriver<NODETYPE>(node),
  remote_online_(false),
  speed_regulator_(0),
  left_joystick_x_(0.0),
  left_joystick_y_(0.0),
  right_joystick_x_(0.0),
  right_joystick_y_(0.0)
{
}

template<class NODETYPE>
void NodeCanopenScanrecoDriver<NODETYPE>::init(bool called_from_base)
{
  called_from_base = false;
  RCLCPP_ERROR(this->node_->get_logger(), "Not init implemented.");
}

template<>
void NodeCanopenScanrecoDriver<rclcpp::Node>::init(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<rclcpp::Node>::init(called_from_base);
}

template<>
void NodeCanopenScanrecoDriver<rclcpp_lifecycle::LifecycleNode>::init(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(called_from_base);
}

template<>
void NodeCanopenScanrecoDriver<rclcpp::Node>::setupRosInterfaces()
{
  four_wheel_steering_cmd_publisher_ =
    this->node_->create_publisher<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>(
    std::string(this->node_->get_name()).append("/cmd_four_wheel_steering").c_str(), 10);
}

template<>
void NodeCanopenScanrecoDriver<rclcpp_lifecycle::LifecycleNode>::setupRosInterfaces()
{
  four_wheel_steering_cmd_publisher_ =
    this->node_->create_publisher<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>(
    std::string(this->node_->get_name()).append("/cmd_four_wheel_steering").c_str(), 10);
}

template<class NODETYPE>
void NodeCanopenScanrecoDriver<NODETYPE>::configure(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<NODETYPE>::configure(called_from_base);

  RCLCPP_INFO_STREAM(this->node_->get_logger(), "CONFIGURE");
  // Create Publisher
  setupRosInterfaces();
}

template<class NODETYPE>
void NodeCanopenScanrecoDriver<NODETYPE>::activate(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<NODETYPE>::activate(called_from_base);
  remote_online_ = false;
  speed_regulator_ = 0.f;
  left_joystick_x_ = 0.f;
  left_joystick_y_ = 0.f;
  right_joystick_x_ = 0.f;
  right_joystick_y_ = 0.f;
}

template<class NODETYPE>
void NodeCanopenScanrecoDriver<NODETYPE>::deactivate(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<NODETYPE>::deactivate(called_from_base);
  remote_online_ = false;
  speed_regulator_ = 0.f;
  left_joystick_x_ = 0.f;
  left_joystick_y_ = 0.f;
  right_joystick_x_ = 0.f;
  right_joystick_y_ = 0.f;
}

template<class NODETYPE>
void NodeCanopenScanrecoDriver<NODETYPE>::cleanup(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<NODETYPE>::cleanup(called_from_base);
}

template<class NODETYPE>
void NodeCanopenScanrecoDriver<NODETYPE>::shutdown(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<NODETYPE>::shutdown(called_from_base);
}

bool isSystemRunning(ros2_canopen::COData d)
{
  if (d.index_ == 0x6404 && d.subindex_ == 0x01) {
    return d.data_ & 0x01;
  }
  return false;
}

float normalizedJoystickValue(int v)
{
  // Reading has offset of 0x7F
  return (v - 127) / 127.f;
}

float normalizedSpeedRegulatorValue(int v)
{
  // Reading has offset of 0x7F
  return std::max((v - 127) / 127.f, 0.f);
}

float readLeftJoystickDirectionX(ros2_canopen::COData d)
{
  if (d.index_ == 0x6400 && d.subindex_ == 0x01) {
    return normalizedJoystickValue(d.data_);
  } else {
    return 0.f;
  }
}

float readLeftJoystickDirectionY(ros2_canopen::COData d)
{
  if (d.index_ == 0x6400 && d.subindex_ == 0x02) {
    return normalizedJoystickValue(d.data_);
  } else {
    return 0.f;
  }
}

float readRightJoystickDirectionX(ros2_canopen::COData d)
{
  if (d.index_ == 0x6400 && d.subindex_ == 0x03) {
    return normalizedJoystickValue(d.data_);
  } else {
    return 0.f;
  }
}

float readRightJoystickDirectionY(ros2_canopen::COData d)
{
  if (d.index_ == 0x6400 && d.subindex_ == 0x04) {
    return normalizedJoystickValue(d.data_);
  } else {
    return 0.f;
  }
}

float readSpeedRegulator(ros2_canopen::COData d)
{
  if (d.index_ == 0x6400 && d.subindex_ == 0x05) {
    return normalizedSpeedRegulatorValue(d.data_);
  } else {
    return 0.f;
  }
}


template<class NODETYPE>
void NodeCanopenScanrecoDriver<NODETYPE>::on_rpdo(COData d)
{
  if (this->activated_.load()) {
    remote_online_ = isSystemRunning(d);
    speed_regulator_ = readSpeedRegulator(d);
    left_joystick_x_ = readLeftJoystickDirectionX(d);
    left_joystick_y_ = readLeftJoystickDirectionY(d);
    right_joystick_x_ = readRightJoystickDirectionX(d);
    right_joystick_y_ = readRightJoystickDirectionY(d);
  }
  if (not remote_online_) {
    this->publish_stop_cmd();
  } else {
    // If we received a control command recently, publish the control command
    this->publish_control_cmd();
  }
}

template<class NODETYPE>
void NodeCanopenScanrecoDriver<NODETYPE>::publish_stop_cmd()
{
  if (this->activated_.load()) {
    auto message = four_wheel_steering_msgs::msg::FourWheelSteeringStamped();
    message.header.stamp = this->node_->now();
    message.data.front_steering_angle = 0.0;   // No steering movement
    message.data.rear_steering_angle = 0.0;   // No steering movement
    message.data.speed = 0.0;   // No forward/backward movement
    four_wheel_steering_cmd_publisher_->publish(message);
  }
}

template<class NODETYPE>
void NodeCanopenScanrecoDriver<NODETYPE>::publish_control_cmd()
{
  if (this->activated_.load()) {
    auto message = four_wheel_steering_msgs::msg::FourWheelSteeringStamped();
    message.header.stamp = this->node_->now();

    message.data.speed = speed_regulator_ * speed_scale_;

    float dir_ratio_ = std::clamp(left_joystick_y_ / left_joystick_x_, -1.f, 1.f);
    message.data.front_steering_angle = atan(dir_ratio_);
    four_wheel_steering_cmd_publisher_->publish(message);
  }
}

#endif  // NODE_CANOPEN_SCANRECO_DRIVER_IMPL_HPP_false
