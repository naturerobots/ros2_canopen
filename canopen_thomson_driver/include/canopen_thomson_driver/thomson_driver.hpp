// Copyright (c) 2026, Nature Robots GmbH
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

#ifndef CANOPEN_THOMSON_DRIVER__THOMSON_DRIVER_HPP_
#define CANOPEN_THOMSON_DRIVER__THOMSON_DRIVER_HPP_

#include "canopen_core/driver_node.hpp"
#include "canopen_thomson_driver/node_interfaces/node_canopen_thomson_driver.hpp"

namespace ros2_canopen
{
/**
 * @brief Driver for a Thomson linear actuator (vendor-specific PDOs, not CiA402).
 *
 * See NodeCanopenThomsonDriver for the protocol and the bus.yml parameters.
 */
class ThomsonDriver : public ros2_canopen::CanopenDriver
{
  std::shared_ptr<node_interfaces::NodeCanopenThomsonDriver<rclcpp::Node>> node_canopen_thomson_driver_;

public:
  ThomsonDriver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

  virtual bool reset_node_nmt_command()
  {
    return node_canopen_thomson_driver_->reset_node_nmt_command();
  }

  virtual bool start_node_nmt_command()
  {
    return node_canopen_thomson_driver_->start_node_nmt_command();
  }

  virtual bool tpdo_transmit(ros2_canopen::COData& data)
  {
    return node_canopen_thomson_driver_->tpdo_transmit(data);
  }

  virtual bool sdo_write(ros2_canopen::COData& data)
  {
    return node_canopen_thomson_driver_->sdo_write(data);
  }

  virtual bool sdo_read(ros2_canopen::COData& data)
  {
    return node_canopen_thomson_driver_->sdo_read(data);
  }

  void register_nmt_state_cb(std::function<void(canopen::NmtState, uint8_t)> nmt_state_cb)
  {
    node_canopen_thomson_driver_->register_nmt_state_cb(nmt_state_cb);
  }

  void register_rpdo_cb(std::function<void(COData, uint8_t)> rpdo_cb)
  {
    node_canopen_thomson_driver_->register_rpdo_cb(rpdo_cb);
  }

  bool set_target(double position)
  {
    return node_canopen_thomson_driver_->set_target(position);
  }

  double get_position() const
  {
    return node_canopen_thomson_driver_->get_position();
  }

  double get_speed() const
  {
    return node_canopen_thomson_driver_->get_speed();
  }

  uint16_t get_current() const
  {
    return node_canopen_thomson_driver_->get_current();
  }

  void enable()
  {
    node_canopen_thomson_driver_->enable();
  }

  void disable()
  {
    node_canopen_thomson_driver_->disable();
  }

  bool has_communication_failure() const
  {
    return node_canopen_thomson_driver_->has_communication_failure();
  }

  bool has_fault() const
  {
    return node_canopen_thomson_driver_->has_fault();
  }

  bool is_ready() const
  {
    return node_canopen_thomson_driver_->is_ready();
  }

  bool is_enabled() const
  {
    return node_canopen_thomson_driver_->is_enabled();
  }

  const std::string& get_joint_name() const
  {
    return node_canopen_thomson_driver_->get_joint_name();
  }
};
}  // namespace ros2_canopen

#endif  // CANOPEN_THOMSON_DRIVER__THOMSON_DRIVER_HPP_
