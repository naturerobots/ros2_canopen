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
#ifndef NODE_CANOPEN_SCANRECO_DRIVER_HPP_
#define NODE_CANOPEN_SCANRECO_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "four_wheel_steering_msgs/msg/four_wheel_steering_stamped.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"


namespace ros2_canopen
{
namespace node_interfaces
{
template<class NODETYPE>
class NodeCanopenScanrecoDriver : public NodeCanopenProxyDriver<NODETYPE>
{
  static_assert(
    std::is_base_of<rclcpp::Node, NODETYPE>::value ||
    std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
    "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

protected:
  bool remote_online_ = false; // Indicates if the remote controller is online and not in emergency stop
  float speed_regulator_;
  float left_joystick_x_;
  float left_joystick_y_;
  float right_joystick_x_;
  float right_joystick_y_;
  float speed_scale_ = 0.5;   // Scale for speed

  rclcpp::Publisher<four_wheel_steering_msgs::msg::FourWheelSteeringStamped>::SharedPtr
    four_wheel_steering_cmd_publisher_;

  void publish_stop_cmd();
  void publish_control_cmd();

public:
  NodeCanopenScanrecoDriver(NODETYPE * node);

  void setupRosInterfaces();

  virtual void init(bool called_from_base) override;
  virtual void configure(bool called_from_base) override;
  virtual void activate(bool called_from_base) override;
  virtual void deactivate(bool called_from_base) override;
  virtual void cleanup(bool called_from_base) override;
  virtual void shutdown(bool called_from_base) override;
  virtual void on_rpdo(COData d) override;
};
}  // namespace node_interfaces
}  // namespace ros2_canopen

#endif  // NODE_CANOPEN_SCANRECO_DRIVER_HPP_
