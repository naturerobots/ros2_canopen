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
#ifndef NODE_CANOPEN_TELEOP_DRIVER_IMPL_HPP_
#define NODE_CANOPEN_TELEOP_DRIVER_IMPL_HPP_

#include "canopen_teleop_driver/node_interfaces/node_canopen_teleop_driver.hpp"

using namespace ros2_canopen::node_interfaces;

template<class NODETYPE>
NodeCanopenTeleopDriver<NODETYPE>::NodeCanopenTeleopDriver(NODETYPE * node)
: ros2_canopen::node_interfaces::NodeCanopenProxyDriver<NODETYPE>(node),
  remote_online_(false),
  speed_regulator_(0),
  linear_stick_amplitude_(0),
  turning_stick_amplitude_(0),
  drive_mode_(0),
  direction_(0)
{
}

template<class NODETYPE>
void NodeCanopenTeleopDriver<NODETYPE>::init(bool called_from_base)
{
  called_from_base = false;
  RCLCPP_ERROR(this->node_->get_logger(), "Not init implemented.");
}

template<>
void NodeCanopenTeleopDriver<rclcpp::Node>::init(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<rclcpp::Node>::init(called_from_base);
}

template<>
void NodeCanopenTeleopDriver<rclcpp_lifecycle::LifecycleNode>::init(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(called_from_base);
}

template<>
void NodeCanopenTeleopDriver<rclcpp::Node>::setupRosInterfaces()
{
  twist_cmd_publisher_ = this->node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    std::string(this->node_->get_name()).append("/cmd_vel").c_str(), 10);
}

template<>
void NodeCanopenTeleopDriver<rclcpp_lifecycle::LifecycleNode>::setupRosInterfaces()
{
  twist_cmd_publisher_ = this->node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    std::string(this->node_->get_name()).append("/cmd_vel").c_str(), 10);
}

template<class NODETYPE>
void NodeCanopenTeleopDriver<NODETYPE>::configure(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<NODETYPE>::configure(called_from_base);

  RCLCPP_INFO_STREAM(this->node_->get_logger(), "CONFIGURE");
  // Create Publisher
  setupRosInterfaces();
}

template<class NODETYPE>
void NodeCanopenTeleopDriver<NODETYPE>::activate(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<NODETYPE>::activate(called_from_base);
  remote_online_ = false;
  speed_regulator_ = 0.0;
  linear_stick_amplitude_ = 0.0;
  turning_stick_amplitude_ = 0.0;
  drive_mode_ = 0;
  direction_ = 0;
  last_control_ = this->node_->now();
}

template<class NODETYPE>
void NodeCanopenTeleopDriver<NODETYPE>::deactivate(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<NODETYPE>::deactivate(called_from_base);
  remote_online_ = false;
  speed_regulator_ = 0.0;
  linear_stick_amplitude_ = 0.0;
  turning_stick_amplitude_ = 0.0;
  drive_mode_ = 0;
  direction_ = 0;
  last_control_ = this->node_->now();
}

template<class NODETYPE>
void NodeCanopenTeleopDriver<NODETYPE>::cleanup(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<NODETYPE>::cleanup(called_from_base);
}

template<class NODETYPE>
void NodeCanopenTeleopDriver<NODETYPE>::shutdown(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<NODETYPE>::shutdown(called_from_base);
}

template<class NODETYPE>
void NodeCanopenTeleopDriver<NODETYPE>::on_rpdo(COData d)
{
  if (this->activated_.load()) {
    // RCLCPP_INFO(
    //   this->node_->get_logger(), "Node ID 0x%X: Received PDO index %#04x, subindex %hhu, data %x",
    //   this->lely_driver_->get_id(), d.index_, d.subindex_, d.data_);
    if (d.index_ == 0x6000 && d.subindex_ == 0x02) {  // Check if remote controller is running and not in emergency stop
      if (d.data_ & 0b00000111) {
        remote_online_ = true;
      } else {
        remote_online_ = false;
      }
    } else if (d.index_ == 0x6000 && d.subindex_ == 0x01) {  // Read direction register
      direction_ = d.data_;
    } else if (d.index_ == 0x6400 && d.subindex_ == 0x01) {  // Read linear stick amplitude
      linear_stick_amplitude_ = d.data_ / 255.0;  // Convert to float in range [0.0, 1.0]
    } else if (d.index_ == 0x6400 && d.subindex_ == 0x02) {  // Read turning stick amplitude
      turning_stick_amplitude_ = d.data_ / 255.0;  // Convert to float in range [0.0, 1.0]
    } else if (d.index_ == 0x6400 && d.subindex_ == 0x03) {  // Read speed regulator position
      speed_regulator_ = d.data_ / 255.0;  // Convert to float in range [0.0, 1.0]
    } else if (d.index_ == 0x6000 && d.subindex_ == 0x11) {  // Read if control command was received
      if (d.data_ & 0b00010000) {
        last_control_ = this->node_->now();
      }
    }
    if (not remote_online_) {
      this->publish_stop_cmd();
    } else if (this->node_->now() - last_control_ < rclcpp::Duration::from_seconds(1.0)) {
      // If we received a control command recently, publish the control command
      this->publish_control_cmd();
    }
  }
}

template<class NODETYPE>
void NodeCanopenTeleopDriver<NODETYPE>::publish_stop_cmd()
{
  if (this->activated_.load()) {
    auto message = geometry_msgs::msg::TwistStamped();
    message.header.stamp = this->node_->now();
    message.twist.linear.x = 0.0;  // No forward/backward movement
    message.twist.angular.z = 0.0;  // No turning movement
    twist_cmd_publisher_->publish(message);
  }
}

template<class NODETYPE>
void NodeCanopenTeleopDriver<NODETYPE>::publish_control_cmd()
{
  if (this->activated_.load()) {
    auto message = geometry_msgs::msg::TwistStamped();
    message.header.stamp = this->node_->now();

    // Clip commands to [0.0, 1.0]
    speed_regulator_ = std::max(0.0f, std::min(speed_regulator_, 1.0f));
    linear_stick_amplitude_ = std::max(0.0f, std::min(linear_stick_amplitude_, 1.0f));
    turning_stick_amplitude_ = std::max(0.0f, std::min(turning_stick_amplitude_, 1.0f));

    if (direction_ & 0b0001) {  // FORWARD
      message.twist.linear.x = speed_regulator_ * linear_stick_amplitude_ * linear_speed_scale_;
    } else if (direction_ & 0b0010) {  // BACKWARD
      message.twist.linear.x = -speed_regulator_ * linear_stick_amplitude_ * linear_speed_scale_;
    } else {
      message.twist.linear.x = 0.0;  // No forward/backward movement
    }

    if (direction_ & 0b0100) {  // RIGHT
      message.twist.angular.z = -speed_regulator_ * turning_stick_amplitude_ * angular_speed_scale_;
    } else if (direction_ & 0b1000) {  // LEFT
      message.twist.angular.z = speed_regulator_ * turning_stick_amplitude_ * angular_speed_scale_;
    } else {
      message.twist.angular.z = 0.0;  // No turning movement
    }

    twist_cmd_publisher_->publish(message);
  }
}

#endif  // NODE_CANOPEN_TELEOP_DRIVER_IMPL_HPP_false
