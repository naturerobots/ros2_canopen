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
#ifndef NODE_CANOPEN_TELEOP_DRIVER_HPP_
#define NODE_CANOPEN_TELEOP_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"


namespace ros2_canopen
{
namespace node_interfaces
{
template<class NODETYPE>
class NodeCanopenTeleopDriver : public NodeCanopenProxyDriver<NODETYPE>
{
  static_assert(
    std::is_base_of<rclcpp::Node, NODETYPE>::value ||
    std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
    "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

protected:
  bool remote_online_ = false; // Indicates if the remote controller is online and not in emergency stop
  float speed_regulator_;
  float linear_stick_amplitude_;
  float turning_stick_amplitude_;
  uint8_t drive_mode_; // 0b0001: MAN,     0b0010: COST,     0b0100: AUTO
  uint8_t direction_;  // 0b0001: FORWARD, 0b0010: BACKWARD, 0b0100: RIGHT, 0b1000: LEFT
  rclcpp::Time last_control_; // Timestamp of the last control input received
  rclcpp::Time last_publish_; // Timestamp of the last published control command
  rclcpp::Duration publish_interval_ = rclcpp::Duration::from_seconds(0.005); // Minimum interval between publishes
  rclcpp::Duration control_timeout_ = rclcpp::Duration::from_seconds(1.0); // Time after which we consider the control input to be outdated
  
  float linear_speed_scale_ = 1.0;   // Scale for linear speed
  float angular_speed_scale_ = 0.5 * 3.14; // Scale for angular speed

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_publisher_;

  void publish_stop_cmd();
  void publish_control_cmd();
  bool ready_to_publish() const;
  bool command_is_not_to_old() const;


public:
  NodeCanopenTeleopDriver(NODETYPE * node);

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

#endif  // NODE_CANOPEN_TELEOP_DRIVER_HPP_
