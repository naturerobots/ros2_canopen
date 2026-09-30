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

#ifndef NODE_CANOPEN_THOMSON_DRIVER
#define NODE_CANOPEN_THOMSON_DRIVER

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

#include "canopen_base_driver/value_conversion.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"

namespace ros2_canopen
{
namespace node_interfaces
{

/// One object of the actuator's object dictionary that is mapped into a PDO.
struct ThomsonObject
{
  uint16_t index;
  uint8_t subindex;
};

/**
 * @brief Driver for Thomson linear actuators with CANopen (vendor-specific, not CiA402).
 *
 * The actuator is commanded through RPDO1 (target position, current limit, speed and a motion
 * enable byte) and reports through TPDO1 (actual position, current and status). Motion only
 * starts after the enable byte went 1 -> 0 -> 1, so the driver runs that sequence whenever it
 * (re)enables the actuator.
 *
 * The command RPDO must be mapped with synchronous transmission (bus.yml `transmission: 0x01`):
 * the master then sends the complete command frame on every SYNC.
 *
 * The joint position is the steering angle in rad, converted by ValueConversion "position":
 * linear raw = center_position + rad * scale_pos_to_dev (default), or `position_conversion: table`
 * with measured `position_table_to_dev` / `position_table_from_dev` for a nonlinear steering linkage.
 */
template <class NODETYPE>
class NodeCanopenThomsonDriver : public NodeCanopenProxyDriver<NODETYPE>
{
  static_assert(std::is_base_of<rclcpp::Node, NODETYPE>::value ||
                    std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
                "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

public:
  NodeCanopenThomsonDriver(NODETYPE* node);

  virtual void configure(bool called_from_base) override;
  virtual void activate(bool called_from_base) override;
  virtual void deactivate(bool called_from_base) override;

  /// Target steering angle in rad. Clamped to [min_raw, max_raw] of the actuator.
  bool set_target(double position);

  /// Actual steering angle in rad, from the last TPDO.
  double get_position() const;

  /// Steering velocity in rad/s, differentiated from TPDO positions.
  double get_speed() const;

  /// Raw actuator current as reported in the TPDO.
  uint16_t get_current() const;

  /// Raw status and flag bytes as reported in the TPDO.
  uint8_t get_status() const;
  uint8_t get_flags() const;

  /// Request motion: runs the 1 -> 0 -> 1 enable sequence, then keeps the enable byte at 1.
  void enable();

  /// Stop motion: enable byte 0, target follows the actual position.
  void disable();

  /// True if no TPDO arrived within feedback_timeout_ms.
  bool has_communication_failure() const;

  /// True if the TPDO flags report a fault (fault_flag_mask).
  bool has_fault() const;

  /// Feedback alive and no fault, i.e. the actuator may be enabled.
  bool is_ready() const;

  /// Enable sequence finished and the actuator follows the target.
  bool is_enabled() const;

  const std::string& get_joint_name() const
  {
    return joint_name_;
  }

protected:
  virtual void poll_timer_callback() override;
  virtual void on_nmt(canopen::NmtState nmt_state) override;
  virtual void on_rpdo(COData data) override;
  virtual void diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper& stat) override;

private:
  enum class EnableState
  {
    Disabled,
    ArmHigh,  // first 1 of the 1 -> 0 -> 1 sequence
    ArmLow,   // the 0
    Enabled,  // final 1, kept while enabled
  };

  ThomsonObject read_object(const std::string& key, ThomsonObject default_value);
  uint16_t to_raw(double position) const;
  double from_raw(uint16_t raw) const;
  void request_rearm(const char* reason);
  void write_command(uint16_t position_raw, uint8_t enable);

  // configuration (bus.yml)
  std::string joint_name_;
  ValueConversion position_conversion_;
  uint16_t min_raw_ = 0;
  uint16_t max_raw_ = 0xFFFF;
  uint16_t current_limit_ = 0x00DC;
  uint8_t speed_ = 0x32;
  uint16_t aux_ = 0x00F0;
  uint32_t enable_step_cycles_ = 3;
  std::chrono::milliseconds feedback_timeout_{ 1000 };
  uint8_t rearm_flag_mask_ = 0x01;
  uint8_t fault_flag_mask_ = 0x00;
  std::chrono::milliseconds fault_reset_interval_{ 2000 };  // 0 = never reset the node on a fault

  ThomsonObject cmd_position_;
  ThomsonObject cmd_current_limit_;
  ThomsonObject cmd_speed_;
  ThomsonObject cmd_aux_;
  ThomsonObject cmd_aux2_;  // Device has aux split into two 8-bit objects
  ThomsonObject cmd_enable_;
  ThomsonObject fb_position_;
  ThomsonObject fb_current_;
  ThomsonObject fb_status_;
  ThomsonObject fb_flags_;

  // command side, written by ros2_control, read by the poll timer
  std::atomic<double> target_position_{ 0.0 };
  std::atomic<bool> target_valid_{ false };
  std::atomic<bool> enable_requested_{ false };
  std::atomic<bool> rearm_requested_{ false };
  std::atomic<bool> sequence_done_{ false };

  // enable sequence, poll timer only
  EnableState enable_state_ = EnableState::Disabled;
  uint32_t state_cycles_ = 0;
  std::chrono::steady_clock::time_point last_rearm_;
  bool comm_failure_seen_ = false;
  bool fault_seen_ = false;
  std::chrono::steady_clock::time_point last_fault_reset_;

  // feedback side, written by on_rpdo, read by ros2_control
  std::atomic<uint16_t> actual_raw_{ 0 };
  std::atomic<double> actual_speed_{ 0.0 };
  std::atomic<uint16_t> actual_current_{ 0 };
  std::atomic<uint8_t> status_{ 0 };
  std::atomic<uint8_t> flags_{ 0 };
  std::atomic<bool> feedback_received_{ false };
  std::atomic<int64_t> last_feedback_ns_{ 0 };
  std::chrono::steady_clock::time_point last_position_time_;
  uint16_t last_position_raw_ = 0;
};

}  // namespace node_interfaces
}  // namespace ros2_canopen

#endif
