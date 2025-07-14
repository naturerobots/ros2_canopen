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
#ifndef NODE_CANOPEN_PRE_MAPPED_DRIVER_HPP_
#define NODE_CANOPEN_PRE_MAPPED_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"


namespace ros2_canopen
{
namespace node_interfaces
{
template<class NODETYPE>
class NodeCanopenPreMappedDriver : public NodeCanopenProxyDriver<NODETYPE>
{
  static_assert(
    std::is_base_of<rclcpp::Node, NODETYPE>::value ||
    std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
    "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

protected:
  virtual void on_nmt(canopen::NmtState nmt_state) override;
  virtual void poll_timer_callback() override;
  virtual void on_rpdo(COData d) override;

public:
  NodeCanopenPreMappedDriver(NODETYPE * node);

  void setupRosInterfaces();
  bool write_target(double target, uint8_t rollover);

  virtual void init(bool called_from_base) override;
  virtual void configure(bool called_from_base) override;
  virtual void activate(bool called_from_base) override;
  virtual void deactivate(bool called_from_base) override;
  virtual void cleanup(bool called_from_base) override;
  virtual void shutdown(bool called_from_base) override;

  virtual bool stop_node_nmt_command();

};
}  // namespace node_interfaces
}  // namespace ros2_canopen
#endif  // NODE_CANOPEN_PRE_MAPPED_DRIVER_HPP_
