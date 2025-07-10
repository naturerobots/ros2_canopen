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

#ifndef CANOPEN_TELEOP_DRIVER_HPP_
#define CANOPEN_TELEOP_DRIVER_HPP_
#include "canopen_teleop_driver/node_interfaces/node_canopen_teleop_driver.hpp"
#include "canopen_core/driver_node.hpp"

namespace ros2_canopen
{
/**
 * @brief Abstract Class for a CANopen Device Node
 *
 * This class provides the base functionality for creating a
 * CANopen device node. It provides callbacks for nmt and rpdo.
 */
class TeleopDriver : public ros2_canopen::CanopenDriver
{
  std::shared_ptr<node_interfaces::NodeCanopenTeleopDriver<rclcpp::Node>>
  node_canopen_teleop_driver_;

public:
  TeleopDriver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());

  virtual bool reset_node_nmt_command()
  {
    return node_canopen_teleop_driver_->reset_node_nmt_command();
  }

  virtual bool start_node_nmt_command()
  {
    return node_canopen_teleop_driver_->start_node_nmt_command();
  }

  virtual bool tpdo_transmit(ros2_canopen::COData & data)
  {
    return node_canopen_teleop_driver_->tpdo_transmit(data);
  }

  void register_nmt_state_cb(std::function<void(canopen::NmtState, uint8_t)> nmt_state_cb)
  {
    node_canopen_teleop_driver_->register_nmt_state_cb(nmt_state_cb);
  }

  void register_rpdo_cb(std::function<void(COData, uint8_t)> rpdo_cb)
  {
    node_canopen_teleop_driver_->register_rpdo_cb(rpdo_cb);
  }

};
}  // namespace ros2_canopen
#endif // CANOPEN_TELEOP_DRIVER_HPP_
