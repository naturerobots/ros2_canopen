//    Copyright 2024 Malte kleine Piening, Nature Robots GmbH
//    Copyright 2023 Christoph Hellmann Santos
//                          Vishnuprasad Prachandabhanu
//                          Lovro Ivanov
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

#ifndef NODE_CANOPEN_PRE_MAPPED_DRIVER_IMPL_HPP_
#define NODE_CANOPEN_PRE_MAPPED_DRIVER_IMPL_HPP_

#include "canopen_pre_mapped_driver/node_interfaces/node_canopen_pre_mapped_driver.hpp"
#include "canopen_core/driver_error.hpp"

#include <optional>

using namespace ros2_canopen::node_interfaces;
using namespace std::placeholders;

template<class NODETYPE>
NodeCanopenPreMappedDriver<NODETYPE>::NodeCanopenPreMappedDriver(NODETYPE * node)
: ros2_canopen::node_interfaces::NodeCanopenProxyDriver<NODETYPE>(node)
{
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::init(bool called_from_base)
{
  RCLCPP_ERROR(this->node_->get_logger(), "Not init implemented.");
}

template<>
void NodeCanopenPreMappedDriver<rclcpp::Node>::init(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp::Node>::init(false);
}

template<>
void NodeCanopenPreMappedDriver<rclcpp_lifecycle::LifecycleNode>::init(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(false);
}

template<>
void NodeCanopenPreMappedDriver<rclcpp::Node>::setupRosInterfaces(const std::string & joint_name)
{
  publish_joint_state =
    this->node_->create_publisher<sensor_msgs::msg::JointState>(
    "~/" + joint_name + "/joint_states",
    10);

  handle_init_service = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/" + joint_name + "/init",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
      this->handle_init(request, response);
    });

  handle_halt_service = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/" + joint_name + "/halt",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
      this->handle_halt(request, response);
    });

  handle_recover_service = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/" + joint_name + "/recover",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
      this->handle_recover(request, response);
    });

  handle_set_target_service =
    this->node_->create_service<canopen_interfaces::srv::COTargetDouble>(
    "~/" + joint_name + "/target",
    [this](const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
    canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response) {
      this->handle_set_target(request, response);
    });
}

template<>
void NodeCanopenPreMappedDriver<rclcpp_lifecycle::LifecycleNode>::setupRosInterfaces(
  const std::string & joint_name)
{
  publish_joint_state =
    this->node_->create_publisher<sensor_msgs::msg::JointState>(
    "~/" + joint_name + "/joint_states",
    10);

  handle_init_service = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/" + joint_name + "/init",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
      this->handle_init(request, response);
    });

  handle_halt_service = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/" + joint_name + "/halt",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
      this->handle_halt(request, response);
    });

  handle_recover_service = this->node_->create_service<std_srvs::srv::Trigger>(
    "~/" + joint_name + "/recover",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
      this->handle_recover(request, response);
    });

  handle_set_target_service =
    this->node_->create_service<canopen_interfaces::srv::COTargetDouble>(
    "~/" + joint_name + "/target",
    [this](const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
    canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response) {
      this->handle_set_target(request, response);
    });
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::configure(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::configure(false);
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::activate(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::activate(false);
  this->activated_.store(true);
  // Get CANopen Node ID
  uint8_t node_id = this->lely_driver_->get_id();
  // Configure motor controller via SDO's
  if (this->activated_.load()) {
    uint32_t tpdo1_cob_id = 0x180 + node_id;
    this->lely_driver_->async_sdo_write_typed(0x1800, 0x01, tpdo1_cob_id);

    uint32_t rpdo1_cob_id = 0x200 + node_id;
    this->lely_driver_->async_sdo_write_typed(0x1800, 0x01, rpdo1_cob_id);
    RCLCPP_INFO_STREAM(
      this->node_->get_logger(),
      "Successfully set SDOs for node ID: " << static_cast<int>(node_id));
  } else {
    RCLCPP_ERROR_STREAM(
      this->node_->get_logger(),
      "Could not activate driver because lely communication is not available.");
  }
  this->start_node_nmt_command();
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::deactivate(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::deactivate(called_from_base);
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::poll_timer_callback()
{
  NodeCanopenProxyDriver<NODETYPE>::poll_timer_callback();
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::publish()
{
  sensor_msgs::msg::JointState js_msg;
  js_msg.name.push_back(motor_->getJointName());
  js_msg.position.push_back(motor_->get_position());
  js_msg.velocity.push_back(motor_->get_speed());
  js_msg.effort.push_back(0.0);
  publish_joint_state->publish(js_msg);
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::add_to_master()
{
  NodeCanopenProxyDriver<NODETYPE>::add_to_master();
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::handle_init(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (this->activated_.load()) {
  }
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::handle_recover(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (this->activated_.load()) {
  }
}
template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::handle_halt(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (this->activated_.load()) {
  }
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::handle_set_target(
  const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
  canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response)
{
  if (this->activated_.load()) {
  }
}

template<class NODETYPE>
bool NodeCanopenPreMappedDriver<NODETYPE>::init_motor()
{
  if (this->activated_.load()) {
    return true;
  } else {
    RCLCPP_INFO(this->node_->get_logger(), "Initialisation failed.");
    return false;
  }
}

template<class NODETYPE>
bool NodeCanopenPreMappedDriver<NODETYPE>::recover_motor()
{
  if (this->activated_.load()) {
    return true;
  } else {
    return false;
  }
}

template<class NODETYPE>
bool NodeCanopenPreMappedDriver<NODETYPE>::halt_motor()
{
  if (this->activated_.load()) {
    return true;
  } else {
    return false;
  }
}

template<class NODETYPE>
bool NodeCanopenPreMappedDriver<NODETYPE>::set_target(double target, uint8_t rollover)
{
  if (this->activated_.load()) {
    return true;
  } else {
    return false;
  }
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::diagnostic_callback(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
}

#endif
