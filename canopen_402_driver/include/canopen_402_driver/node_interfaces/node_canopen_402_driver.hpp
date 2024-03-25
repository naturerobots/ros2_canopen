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

#ifndef NODE_CANOPEN_402_DRIVER
#define NODE_CANOPEN_402_DRIVER

#include "canopen_402_driver/motor.hpp"
#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace ros2_canopen
{
namespace node_interfaces
{

template <class NODETYPE>
class NodeCanopen402Driver : public NodeCanopenProxyDriver<NODETYPE>
{
  static_assert(std::is_base_of<rclcpp::Node, NODETYPE>::value ||
                    std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
                "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

protected:
  std::map<uint8_t, std::shared_ptr<Motor402>> motors_;  // map from channel to motor
  std::vector<uint8_t> motor_channels_;                  // list of all registered motor channels
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<uint8_t, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_init_service;
  std::map<uint8_t, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_halt_service;
  std::map<uint8_t, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_recover_service;
  std::map<uint8_t, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_set_mode_position_service;
  std::map<uint8_t, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_set_mode_torque_service;
  std::map<uint8_t, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_set_mode_velocity_service;
  std::map<uint8_t, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_set_mode_cyclic_velocity_service;
  std::map<uint8_t, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_set_mode_cyclic_position_service;
  std::map<uint8_t, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> handle_set_mode_interpolated_position_service;
  std::map<uint8_t, rclcpp::Service<canopen_interfaces::srv::COTargetDouble>::SharedPtr> handle_set_target_service;
  std::map<uint8_t, rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr> publish_joint_state;

  void publish();
  virtual void poll_timer_callback() override;
  void diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper& stat) override;

public:
  NodeCanopen402Driver(NODETYPE* node);

  void setupRosInterfaces(const std::string& joint_name, uint8_t channel);

  virtual void init(bool called_from_base) override;
  virtual void configure(bool called_from_base) override;
  virtual void activate(bool called_from_base) override;
  virtual void deactivate(bool called_from_base) override;
  virtual void add_to_master() override;

  virtual bool recover_motor_on_fault(uint8_t channel)
  {
    return motors_[channel]->handleRecoverOnFault();
  }

  virtual double get_speed(uint8_t channel)
  {
    return motors_[channel]->get_speed();
  }

  virtual double get_position(uint8_t channel)
  {
    return motors_[channel]->get_position();
  }

  virtual uint16_t get_mode(uint8_t channel)
  {
    return motors_[channel]->getMode();
  }

  const std::vector<uint8_t>& get_available_motor_channels()
  {
    return motor_channels_;
  }

  const std::string& get_motor_joint_name(uint8_t channel)
  {
    return motors_[channel]->getJointName();
  }

  /**
   * @brief Service Callback to initialise device
   *
   * Calls Motor402::handleInit function. Brings motor to enabled
   * state and homes it.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_init(const std_srvs::srv::Trigger::Request::SharedPtr request,
                   std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel);

  /**
   * @brief Method to initialise device
   *
   * Calls Motor402::handleInit function. Brings motor to enabled
   * state and homes it.
   *
   * @param [in] void
   *
   * @return  bool
   * Indicates initialisation procedure result
   */
  bool init_motor(uint8_t channel);

  /**
   * @brief Service Callback to recover device
   *
   * Calls Motor402::handleRecover function. Resets faults and brings
   * motor to enabled state.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_recover(const std_srvs::srv::Trigger::Request::SharedPtr request,
                      std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel);

  /**
   * @brief Method to recover device
   *
   * Calls Motor402::handleRecover function. Resets faults and brings
   * motor to enabled state.
   *
   * @param [in] void
   *
   * @return bool
   */
  bool recover_motor(uint8_t channel);

  /**
   * @brief Service Callback to halt device
   *
   * Calls Motor402::handleHalt function. Calls Quickstop. Resulting
   * Motor state depends on devices configuration specifically object
   * 0x605A.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_halt(const std_srvs::srv::Trigger::Request::SharedPtr request,
                   std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel);

  /**
   * @brief Method to halt device
   *
   * Calls Motor402::handleHalt function. Calls Quickstop. Resulting
   * Motor state depends on devices configuration specifically object
   * 0x605A.
   *
   * @param [in] void
   *
   * @return bool
   */
  bool halt_motor(uint8_t channel);

  /**
   * @brief Method to set desired mode
   *
   * Calls Motor402::enterModeAndWait with desired Mode as
   * Target Operation Mode. If successful, the motor was transitioned
   * to desired Mode.
   *
   * @param [in] void
   * @param [out] bool
   */
  bool set_operation_mode(uint8_t channel, uint16_t mode);

  /**
   * @brief Method to set default mode
   *
   * Calls Motor402::enterModeAndWait with default Mode as
   * Target Operation Mode. If successful, the motor was transitioned
   * to default Mode.
   *
   * @param [in] void
   * @param [out] bool
   */
  bool set_default_operation_mode(uint8_t channel);

  /**
   * @brief Service Callback to set desired mode
   *
   * Calls Motor402::enterModeAndWait with desired Mode as
   * Target Operation Mode. If successful, the motor was transitioned
   * to desired Mode.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_set_operation_mode(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                 std_srvs::srv::Trigger::Response::SharedPtr response, uint8_t channel, uint16_t mode);

  /**
   * @brief Service Callback to set target
   *
   * Calls Motor402::setTarget and sets the requested target value. Note
   * that the resulting movement is dependent on the Operation Mode and the
   * drives state.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_set_target(const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
                         canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response, uint8_t channel);

  /**
   * @brief Method to set target
   *
   * Calls Motor402::setTarget and sets the requested target value. Note
   * that the resulting movement is dependent on the Operation Mode and the
   * drives state.
   *
   * @param [in] double target value
   *
   * @return bool
   */
  bool set_target(uint8_t channel, double target);
};
}  // namespace node_interfaces
}  // namespace ros2_canopen

#endif
