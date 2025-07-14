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
#ifndef CANOPEN_PRE_MAPPED_DRIVER_HPP_
#define CANOPEN_PRE_MAPPED_DRIVER_HPP_
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

  bool write_target(double target, uint8_t rollover)
  {
    // return this->node_canopen_pre_mapped_driver_->write_target(target, rollover);
    return true;
  }

};
}  // namespace ros2_canopen
#endif  // CANOPEN_PRE_MAPPED_DRIVER_HPP_
