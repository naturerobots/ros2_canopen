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

#ifndef NODE_CANOPEN_402_DRIVER_IMPL_HPP_
#define NODE_CANOPEN_402_DRIVER_IMPL_HPP_

#include "canopen_402_driver/node_interfaces/node_canopen_402_driver.hpp"
#include "canopen_core/driver_error.hpp"

#include <optional>

using namespace ros2_canopen::node_interfaces;
using namespace std::placeholders;

template <class NODETYPE>
NodeCanopen402Driver<NODETYPE>::NodeCanopen402Driver(NODETYPE* node)
  : ros2_canopen::node_interfaces::NodeCanopenProxyDriver<NODETYPE>(node)
{
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::init(bool called_from_base)
{
  RCLCPP_ERROR(this->node_->get_logger(), "Not init implemented.");
}

template <>
void NodeCanopen402Driver<rclcpp::Node>::init(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp::Node>::init(false);
}

template <>
void NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::init(bool called_from_base)
{
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(false);
}

template <>
void NodeCanopen402Driver<rclcpp::Node>::setupRosInterfaces(const std::string& joint_name, uint8_t channel)
{
  publish_joint_state[channel] =
      this->node_->create_publisher<sensor_msgs::msg::JointState>("~/" + joint_name + "/joint_states", 10);

  handle_init_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/init", [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                   std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_init(request, response, channel);
      });

  handle_halt_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/halt", [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                   std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_halt(request, response, channel);
      });

  handle_recover_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/recover", [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                      std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_recover(request, response, channel);
      });

  handle_set_mode_position_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/position_mode", [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                            std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_set_operation_mode(request, response, channel, MotorBase::Profiled_Position);
      });

  handle_set_mode_velocity_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/velocity_mode", [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                            std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_set_operation_mode(request, response, channel, MotorBase::Profiled_Velocity);
      });

  handle_set_mode_cyclic_velocity_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/cyclic_velocity_mode",
      [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_set_operation_mode(request, response, channel, MotorBase::Cyclic_Synchronous_Velocity);
      });

  handle_set_mode_cyclic_position_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/cyclic_position_mode",
      [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_set_operation_mode(request, response, channel, MotorBase::Cyclic_Synchronous_Position);
      });

  handle_set_mode_interpolated_position_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/interpolated_position_mode",
      [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_set_operation_mode(request, response, channel, MotorBase::Interpolated_Position);
      });

  handle_set_mode_torque_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/torque_mode", [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                          std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_set_operation_mode(request, response, channel, MotorBase::Profiled_Torque);
      });

  handle_set_target_service[channel] = this->node_->create_service<canopen_interfaces::srv::COTargetDouble>(
      "~/" + joint_name + "/target",
      [this, channel](const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
                      canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response) {
        this->handle_set_target(request, response, channel);
      });
}

template <>
void NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::setupRosInterfaces(const std::string& joint_name,
                                                                               uint8_t channel)
{
  publish_joint_state[channel] =
      this->node_->create_publisher<sensor_msgs::msg::JointState>("~/" + joint_name + "/joint_states", 10);

  handle_init_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/init", [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                   std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_init(request, response, channel);
      });

  handle_halt_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/halt", [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                   std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_halt(request, response, channel);
      });

  handle_recover_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/recover", [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                      std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_recover(request, response, channel);
      });

  handle_set_mode_position_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/position_mode", [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                            std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_set_operation_mode(request, response, channel, MotorBase::Profiled_Position);
      });

  handle_set_mode_velocity_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/velocity_mode", [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                            std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_set_operation_mode(request, response, channel, MotorBase::Profiled_Velocity);
      });

  handle_set_mode_cyclic_velocity_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/cyclic_velocity_mode",
      [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_set_operation_mode(request, response, channel, MotorBase::Cyclic_Synchronous_Velocity);
      });

  handle_set_mode_cyclic_position_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/cyclic_position_mode",
      [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_set_operation_mode(request, response, channel, MotorBase::Cyclic_Synchronous_Position);
      });

  handle_set_mode_interpolated_position_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/interpolated_position_mode",
      [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_set_operation_mode(request, response, channel, MotorBase::Interpolated_Position);
      });

  handle_set_mode_torque_service[channel] = this->node_->create_service<std_srvs::srv::Trigger>(
      "~/" + joint_name + "/torque_mode", [this, channel](const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                          std_srvs::srv::Trigger::Response::SharedPtr response) {
        this->handle_set_operation_mode(request, response, channel, MotorBase::Profiled_Torque);
      });

  handle_set_target_service[channel] = this->node_->create_service<canopen_interfaces::srv::COTargetDouble>(
      "~/" + joint_name + "/target",
      [this, channel](const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
                      canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response) {
        this->handle_set_target(request, response, channel);
      });
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::configure(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::configure(false);

  RCLCPP_ERROR_STREAM(this->node_->get_logger(), "CONFIGURE");

  // get channels part of configuration. this contains key value pairs as:
  // <channel_id>:
  //   <key>: <value>
  //    ...
  // <next_channel_id>:
  //   <key>: <value>
  //    ...
  YAML::Node channels_conf;
  try
  {
    channels_conf = this->config_["motor_channels"];
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(this->node_->get_logger(),
                        "Cannot load channels for 402 driver. Make sure all 402 drivers have their channels defined in "
                        "the bus config.");
    return;
  }

  // iterate over channels
  for (auto it = channels_conf.begin(); it != channels_conf.end(); it++)
  {
    auto channel_conf = it->second;

    uint8_t channel = it->first.as<uint8_t>();
    std::optional<std::string> joint_name;
    std::optional<double> scale_pos_to_dev;
    std::optional<double> scale_pos_from_dev;
    std::optional<double> scale_vel_to_dev;
    std::optional<double> scale_vel_from_dev;
    std::optional<int> switching_state;
    std::optional<int> default_operation_mode;
    try
    {
      joint_name = std::optional(channel_conf["joint_name"].as<std::string>());
    }
    catch (...)
    {
      RCLCPP_ERROR_STREAM(this->node_->get_logger(), "Cannot find attribute joint_name for channel id " << channel
                                                                                                        << ". Make "
                                                                                                           "sure these "
                                                                                                           "are "
                                                                                                           "defined "
                                                                                                           "for all "
                                                                                                           "402 "
                                                                                                           "drivers in "
                                                                                                           "the bus "
                                                                                                           "config!");
      return;
    }
    try
    {
      scale_pos_to_dev = std::optional(channel_conf["scale_pos_to_dev"].as<double>());
    }
    catch (...)
    {
    }
    try
    {
      scale_pos_from_dev = std::optional(channel_conf["scale_pos_from_dev"].as<double>());
    }
    catch (...)
    {
    }
    try
    {
      scale_vel_to_dev = std::optional(channel_conf["scale_vel_to_dev"].as<double>());
    }
    catch (...)
    {
    }
    try
    {
      scale_vel_from_dev = std::optional(channel_conf["scale_vel_from_dev"].as<double>());
    }
    catch (...)
    {
    }
    try
    {
      switching_state = std::optional(channel_conf["switching_state"].as<int>());
    }
    catch (...)
    {
    }
    try
    {
      default_operation_mode = std::optional(channel_conf["default_operation_mode"].as<int>());
    }
    catch (...)
    {
    }

    motors_[channel] =
        std::make_shared<Motor402>(nullptr,
                                   (ros2_canopen::State402::InternalState)switching_state.value_or(
                                       (int)ros2_canopen::State402::InternalState::Operation_Enable),
                                   joint_name.value(), scale_pos_to_dev.value_or(1000.0),
                                   scale_pos_from_dev.value_or(0.001), scale_vel_to_dev.value_or(1000.0),
                                   scale_vel_from_dev.value_or(0.001), default_operation_mode.value_or(0), channel);
    motor_channels_.push_back(channel);

    // create publishers and subscribers
    setupRosInterfaces(joint_name.value(), channel);
  }
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::activate(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::activate(false);
  for (const auto& motor : motors_)
  {
    motor.second->registerDefaultModes();
    motor.second->set_diagnostic_status_msgs(this->diagnostic_collector_, this->diagnostic_enabled_);
  }
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::deactivate(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::deactivate(false);
  timer_->cancel();
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::poll_timer_callback()
{
  NodeCanopenProxyDriver<NODETYPE>::poll_timer_callback();
  for (const auto& motor : motors_)
  {
    motor.second->handleRead();
    motor.second->handleWrite();
  }
  publish();
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::publish()
{
  for (const auto& motor : motors_)
  {
    sensor_msgs::msg::JointState js_msg;
    js_msg.name.push_back(motor.second->getJointName());
    js_msg.position.push_back(motor.second->get_position());
    js_msg.velocity.push_back(motor.second->get_speed());
    js_msg.effort.push_back(0.0);
    publish_joint_state[motor.first]->publish(js_msg);
  }
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::add_to_master()
{
  NodeCanopenProxyDriver<NODETYPE>::add_to_master();
  for (const auto& motor : motors_)
  {
    RCLCPP_INFO_STREAM(this->node_->get_logger(), "setting driver for motor");
    motor.second->setDriver(this->lely_driver_);
  }
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_init(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                 std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel)
{
  if (this->activated_.load())
  {
    bool temp = motors_[channel]->handleInit();
    response->success = temp;
  }
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_recover(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                    std_srvs::srv::Trigger::Response::SharedPtr response,
                                                    uint8_t channel)
{
  if (this->activated_.load())
  {
    response->success = motors_[channel]->handleRecover();
  }
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_halt(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                 std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel)
{
  if (this->activated_.load())
  {
    response->success = motors_[channel]->handleHalt();
  }
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_operation_mode(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                                               std_srvs::srv::Trigger::Response::SharedPtr response,
                                                               uint8_t channel, uint16_t mode)
{
  response->success = set_operation_mode(channel, mode);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_target(
    const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
    canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response, uint8_t channel)
{
  if (this->activated_.load())
  {
    response->success = set_target(channel, request->target);
  }
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::init_motor(uint8_t channel)
{
  if (this->activated_.load())
  {
    return motors_[channel]->handleInit();
  }
  else
  {
    RCLCPP_INFO(this->node_->get_logger(), "Initialisation failed.");
    return false;
  }
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::recover_motor(uint8_t channel)
{
  if (this->activated_.load())
  {
    return motors_[channel]->handleRecover();
  }
  else
  {
    return false;
  }
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::halt_motor(uint8_t channel)
{
  if (this->activated_.load())
  {
    return motors_[channel]->handleHalt();
  }
  else
  {
    return false;
  }
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::set_operation_mode(uint8_t channel, uint16_t mode)
{
  if (this->activated_.load())
  {
    if (motors_[channel]->getMode() != mode)
    {
      return motors_[channel]->enterModeAndWait(mode);
    }
    else
    {
      return false;
    }
  }
  return false;
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::set_default_operation_mode(uint8_t channel)
{
  return set_operation_mode(channel, motors_[channel]->getDefaultOperationMode());
}

template <class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::set_target(uint8_t channel, double target)
{
  if (this->activated_.load())
  {
    return motors_[channel]->setTarget(target);
  }
  else
  {
    return false;
  }
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  unsigned char summary_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string summary_msg = "";

  for (const auto& motor : motors_)
  {
    motor.second->handleDiag();

    if (this->diagnostic_collector_->getLevel() >= summary_level)
    {
      summary_level = this->diagnostic_collector_->getLevel();
      summary_msg = motor.second->getJointName() + ": " + this->diagnostic_collector_->getMessage();
    }

    stat.add(motor.second->getJointName() + "_device_state", this->diagnostic_collector_->getValue("DEVICE"));
    stat.add(motor.second->getJointName() + "_nmt_state", this->diagnostic_collector_->getValue("NMT"));
    stat.add(motor.second->getJointName() + "_emcy_state", this->diagnostic_collector_->getValue("EMCY"));
    stat.add(motor.second->getJointName() + "_cia402_mode",
             this->diagnostic_collector_->getValue(motor.second->getJointName() + "_cia402_mode"));
    stat.add(motor.second->getJointName() + "_cia402_set_mode",
             this->diagnostic_collector_->getValue(motor.second->getJointName() + "_cia402_set_mode"));
    stat.add(motor.second->getJointName() + "_cia402_state",
             this->diagnostic_collector_->getValue(motor.second->getJointName() + "_cia402_state"));
    stat.add(motor.second->getJointName() + "_cia402_set_state",
             this->diagnostic_collector_->getValue(motor.second->getJointName() + "_cia402_set_state"));
    stat.add(motor.second->getJointName() + "_cia402_set_state",
             this->diagnostic_collector_->getValue(motor.second->getJointName() + "_cia402_set_state"));
    stat.add(motor.second->getJointName() + "_cia402_is_initialized",
             this->diagnostic_collector_->getValue(motor.second->getJointName() + "_cia402_is_initialized"));
    stat.add(motor.second->getJointName() + "_cia402_has_communication_failure",
             this->diagnostic_collector_->getValue(motor.second->getJointName() + "_cia402_has_communication_failure"));
  }
  stat.summary(summary_level, summary_msg);
}

#endif
