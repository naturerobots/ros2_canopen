//    Copyright 2023 Christoph Hellmann Santos
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

#ifndef CANOPEN_PRE_MAPPED_DRIVER__PRE_MAPPED_DRIVER_HPP_
#define CANOPEN_PRE_MAPPED_DRIVER__PRE_MAPPED_DRIVER_HPP_
#include "canopen_pre_mapped_driver/node_interfaces/node_canopen_pre_mapped_driver.hpp"
#include "canopen_core/driver_node.hpp"

namespace ros2_canopen
{
/**
 * @brief Abstract Class for a CANopen Device Node
 *
 * This class provides the base functionality for creating a
 * CANopen device node. It provides callbacks for nmt and rpdo.
 */
class PreMappedDriver : public ros2_canopen::CanopenDriver
{
  std::shared_ptr<node_interfaces::NodeCanopenPreMappedDriver<rclcpp::Node>>
  node_canopen_pre_mapped_driver_;

public:
  PreMappedDriver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

  virtual bool reset_node_nmt_command()
  {
    return node_canopen_pre_mapped_driver_->reset_node_nmt_command();
  }

  virtual bool start_node_nmt_command()
  {
    return node_canopen_pre_mapped_driver_->start_node_nmt_command();
  }

  virtual bool tpdo_transmit(ros2_canopen::COData & data)
  {
    return node_canopen_pre_mapped_driver_->tpdo_transmit(data);
  }

  virtual bool sdo_write(ros2_canopen::COData & data)
  {
    return node_canopen_pre_mapped_driver_->sdo_write(data);
  }

  virtual bool sdo_read(ros2_canopen::COData & data)
  {
    return node_canopen_pre_mapped_driver_->sdo_read(data);
  }

  void register_nmt_state_cb(std::function<void(canopen::NmtState, uint8_t)> nmt_state_cb)
  {
    node_canopen_pre_mapped_driver_->register_nmt_state_cb(nmt_state_cb);
  }

  void register_rpdo_cb(std::function<void(COData, uint8_t)> rpdo_cb)
  {
    node_canopen_pre_mapped_driver_->register_rpdo_cb(rpdo_cb);
  }

  double get_speed()
  {
    return node_canopen_pre_mapped_driver_->get_speed();
  }

  double get_position()
  {
    return node_canopen_pre_mapped_driver_->get_position();
  }

  bool set_target(double target, uint8_t rollover)
  {
    return node_canopen_pre_mapped_driver_->set_target(target, rollover);
  }

  bool init_motor()
  {
    return node_canopen_pre_mapped_driver_->init_motor();
  }

  bool recover_motor()
  {
    return node_canopen_pre_mapped_driver_->recover_motor();
  }

  bool is_motor_faulty()
  {
    return node_canopen_pre_mapped_driver_->is_motor_faulty();
  }

  bool is_motor_initialized()
  {
    return node_canopen_pre_mapped_driver_->is_motor_initialized();
  }

  bool has_motor_communication_failure()
  {
    return node_canopen_pre_mapped_driver_->has_motor_communication_failure();
  }

  bool is_motor_halted()
  {
    return node_canopen_pre_mapped_driver_->is_motor_halted();
  }

  bool halt_motor()
  {
    return node_canopen_pre_mapped_driver_->halt_motor();
  }

  const std::string & get_motor_joint_name()
  {
    return node_canopen_pre_mapped_driver_->get_motor_joint_name();
  }
};
}  // namespace ros2_canopen

#endif  // CANOPEN_PRE_MAPPED_DRIVER__PRE_MAPPED_DRIVER_HPP_
