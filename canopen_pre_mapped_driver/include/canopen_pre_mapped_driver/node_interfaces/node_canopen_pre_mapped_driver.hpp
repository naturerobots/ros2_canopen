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

#ifndef NODE_CANOPEN_PRE_MAPPED_DRIVER
#define NODE_CANOPEN_PRE_MAPPED_DRIVER

#include "canopen_pre_mapped_driver/motor.hpp"
#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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
  std::shared_ptr<PreMappedMotor> motor_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_init_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_halt_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_recover_service;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publish_joint_state;

  void publish();
  virtual void poll_timer_callback() override;
  void diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper & stat) override;

public:
  NodeCanopenPreMappedDriver(NODETYPE * node);

  void setupRosInterfaces(const std::string & joint_name);

  virtual void init(bool called_from_base) override;
  virtual void configure(bool called_from_base) override;
  virtual void activate(bool called_from_base) override;
  virtual void deactivate(bool called_from_base) override;
  virtual void add_to_master() override;

  virtual bool is_motor_faulty()
  {
    return motor_->isFaulty();
  }

  virtual bool is_motor_initialized()
  {
    return motor_->isInitialized();
  }

  virtual bool has_motor_communication_failure()
  {
    return motor_->hasCommunicationFailure();
  }

  virtual bool is_motor_halted()
  {
    return motor_->isHalted();
  }

  virtual double get_speed()
  {
    return motor_->get_speed();
  }

  virtual double get_position()
  {
    return motor_->get_position();
  }

  const std::string & get_motor_joint_name()
  {
    return motor_->getJointName();
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
  void handle_init(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

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
  bool init_motor();

  /**
   * @brief Service Callback to recover device
   *
   * Calls Motor402::handleRecover function. Resets faults and brings
   * motor to enabled state.
   *
   * @param [in] request
   * @param [out] response
   */
  void handle_recover(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

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
  bool recover_motor();

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
  void handle_halt(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

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
  bool halt_motor();

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
  bool set_target(double target, uint8_t rollover);
};
}  // namespace node_interfaces
}  // namespace ros2_canopen

#endif
