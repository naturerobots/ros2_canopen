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
#ifndef NODE_CANOPEN_PRE_MAPPED_DRIVER_IMPL_HPP_
#define NODE_CANOPEN_PRE_MAPPED_DRIVER_IMPL_HPP_

#include "canopen_pre_mapped_driver/node_interfaces/node_canopen_pre_mapped_driver.hpp"

using namespace ros2_canopen::node_interfaces;

template<class NODETYPE>
NodeCanopenPreMappedDriver<NODETYPE>::NodeCanopenPreMappedDriver(NODETYPE * node)
: ros2_canopen::node_interfaces::NodeCanopenProxyDriver<NODETYPE>(node)
{
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::init(bool called_from_base)
{
  called_from_base = false;
  RCLCPP_ERROR(this->node_->get_logger(), "Not init implemented.");
}

template<>
void NodeCanopenPreMappedDriver<rclcpp::Node>::init(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<rclcpp::Node>::init(called_from_base);
}

template<>
void NodeCanopenPreMappedDriver<rclcpp_lifecycle::LifecycleNode>::init(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(called_from_base);
}

template<>
void NodeCanopenPreMappedDriver<rclcpp::Node>::setupRosInterfaces()
{
  // TODO: Implement ROS interfaces setup
}

template<>
void NodeCanopenPreMappedDriver<rclcpp_lifecycle::LifecycleNode>::setupRosInterfaces()
{
  // TODO: Implement ROS interfaces setup
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::on_nmt(canopen::NmtState nmt_state)
{
  // TODO: Implement NMT state handling
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::poll_timer_callback()
{
  // TODO: Implement poll timer callback
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::on_rpdo(COData d)
{
  // TODO: Implement RPDO handling
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::configure(bool called_from_base)
{
  called_from_base = false;
  // Call base class method
  NodeCanopenProxyDriver<NODETYPE>::configure(called_from_base);
  RCLCPP_INFO_STREAM(this->node_->get_logger(), "CONFIGURE");
  // Create ROS Publisher and Subscriber
  setupRosInterfaces();
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::activate(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<NODETYPE>::activate(called_from_base);

  // Get CANopen Node ID
  uint8_t node_id = this->lely_driver_->get_id();

  // Configure motor controller via SDO's
  try {
    uint32_t tpdo1_cob_id = 0x180 + node_id;
    this->lely_driver_->async_sdo_write_typed(0x1800, 0x01, tpdo1_cob_id);

    uint32_t rpdo1_cob_id = 0x200 + node_id;
    this->lely_driver_->async_sdo_write_typed(0x1800, 0x01, rpdo1_cob_id);
    RCLCPP_INFO_STREAM(
      this->node_->get_logger(),
      "Successfully set SDOs for node ID: " << static_cast<int>(node_id));
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->node_->get_logger(), "SDO write failed: " << e.what());
  }

  RCLCPP_INFO_STREAM(this->node_->get_logger(), "ACTIVATE");
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::deactivate(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<NODETYPE>::deactivate(called_from_base);
  RCLCPP_INFO_STREAM(this->node_->get_logger(), "DEACTIVATE");
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::cleanup(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<NODETYPE>::cleanup(called_from_base);
}

template<class NODETYPE>
void NodeCanopenPreMappedDriver<NODETYPE>::shutdown(bool called_from_base)
{
  called_from_base = false;
  NodeCanopenProxyDriver<NODETYPE>::shutdown(called_from_base);
  RCLCPP_INFO_STREAM(this->node_->get_logger(), "SHUTDOWN");
}

#endif  // NODE_CANOPEN_PRE_MAPPED_DRIVER_IMPL_HPP_
