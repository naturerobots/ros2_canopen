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

#ifndef NODE_CANOPEN_THOMSON_DRIVER_IMPL_HPP_
#define NODE_CANOPEN_THOMSON_DRIVER_IMPL_HPP_

#include <algorithm>
#include <cmath>
#include <future>
#include <stdexcept>

#include "canopen_core/driver_error.hpp"
#include "canopen_thomson_driver/node_interfaces/node_canopen_thomson_driver.hpp"

using namespace ros2_canopen::node_interfaces;

namespace
{
// Minimum time between two re-arm sequences, so a flag that stays set does not toggle forever.
constexpr auto kRearmInterval = std::chrono::seconds(1);
// Re-arms in a row that did not enable motion before the node is reset.
constexpr uint32_t kMaxRearmAttempts = 5;
// While the actuator is offline every feedback read waits for the full SDO timeout. Probing only this
// often keeps that from stalling the executor the other drivers share.
constexpr auto kOfflineProbeInterval = std::chrono::milliseconds(200);
// Feedback is read by SDO, which also works while the actuator is PRE-OPERATIONAL. The bridge times
// a transfer out after 20 ms, so the future normally resolves before this.
constexpr auto kSdoWait = std::chrono::milliseconds(30);

int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
}  // namespace

template <class NODETYPE>
NodeCanopenThomsonDriver<NODETYPE>::NodeCanopenThomsonDriver(NODETYPE* node)
  : NodeCanopenProxyDriver<NODETYPE>(node)
{
}

template <class NODETYPE>
ThomsonObject NodeCanopenThomsonDriver<NODETYPE>::read_object(const std::string& key, ThomsonObject default_value)
{
  // bus.yml: <key>: [index, subindex]
  YAML::Node node = this->config_[key];
  if (!node)
  {
    return default_value;
  }
  if (!node.IsSequence() || node.size() != 2)
  {
    throw DriverException(key + " must be [index, subindex]");
  }
  return ThomsonObject{ node[0].as<uint16_t>(), static_cast<uint8_t>(node[1].as<uint16_t>()) };
}

template <class NODETYPE>
void NodeCanopenThomsonDriver<NODETYPE>::configure(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::configure(false);
  const YAML::Node config = this->config_;

  if (!config["joint_name"])
  {
    throw DriverException("Thomson driver needs a joint_name in the bus config");
  }
  joint_name_ = config["joint_name"].as<std::string>();

  // Linear: raw = center_position + rad * scale_pos_to_dev. Unused with position_conversion: table.
  const double scale_pos_to_dev = config["scale_pos_to_dev"] ? config["scale_pos_to_dev"].as<double>() : 0.0;
  const double center_position = config["center_position"] ? config["center_position"].as<double>() : 900.0;
  position_conversion_ = ValueConversion(config, "position", scale_pos_to_dev,
                                         scale_pos_to_dev != 0.0 ? 1.0 / scale_pos_to_dev : 0.0, center_position);

  if (config["min_raw"])
    min_raw_ = config["min_raw"].as<uint16_t>();
  if (config["max_raw"])
    max_raw_ = config["max_raw"].as<uint16_t>();
  if (config["current_limit"])
    current_limit_ = config["current_limit"].as<uint16_t>();
  if (config["speed"])
    speed_ = static_cast<uint8_t>(config["speed"].as<uint16_t>());
  if (config["aux"])
    aux_ = config["aux"].as<uint16_t>();
  if (config["enable_step_cycles"])
    enable_step_cycles_ = std::max<uint32_t>(1, config["enable_step_cycles"].as<uint32_t>());
  if (config["feedback_timeout_ms"])
    feedback_timeout_ = std::chrono::milliseconds(config["feedback_timeout_ms"].as<uint32_t>());
  if (config["rearm_flag_mask"])
    rearm_flag_mask_ = static_cast<uint8_t>(config["rearm_flag_mask"].as<uint16_t>());
  if (config["fault_flag_mask"])
    fault_flag_mask_ = static_cast<uint8_t>(config["fault_flag_mask"].as<uint16_t>());
  if (config["fault_reset_interval_ms"])
    fault_reset_interval_ = std::chrono::milliseconds(config["fault_reset_interval_ms"].as<uint32_t>());
  if (config["offline_reset_interval_ms"])
    offline_reset_interval_ = std::chrono::milliseconds(config["offline_reset_interval_ms"].as<uint32_t>());

  if (min_raw_ > max_raw_)
  {
    throw DriverException("min_raw must not be larger than max_raw");
  }

  // Defaults match actual device PDO mapping (0x2100:00, 0x2101:00, etc. - separate objects, not subindices)
  cmd_position_ = read_object("cmd_position_object", { 0x2100, 0 });
  cmd_current_limit_ = read_object("cmd_current_limit_object", { 0x2101, 0 });
  cmd_speed_ = read_object("cmd_speed_object", { 0x2102, 0 });
  cmd_aux_ = read_object("cmd_aux_object", { 0x2103, 0 });   // aux high byte (0xF0 from 0x00F0)
  cmd_aux2_ = read_object("cmd_aux2_object", { 0x2104, 0 }); // aux low byte (0x00 from 0x00F0)
  cmd_enable_ = read_object("cmd_enable_object", { 0x2105, 0 });
  fb_position_ = read_object("fb_position_object", { 0x2200, 0 });
  fb_current_ = read_object("fb_current_object", { 0x2201, 0 });
  fb_status_ = read_object("fb_status_object", { 0x2204, 0 });
  fb_flags_ = read_object("fb_flags_object", { 0x2205, 0 });

  RCLCPP_INFO(this->node_->get_logger(),
              "Thomson actuator '%s': %s position conversion, raw at 0 rad %.1f, raw range [%u, %u], "
              "current limit %u, speed %u",
              joint_name_.c_str(), position_conversion_.is_table() ? "table" : "linear",
              position_conversion_.to_dev(0.0), min_raw_, max_raw_, current_limit_, speed_);
}

template <class NODETYPE>
void NodeCanopenThomsonDriver<NODETYPE>::activate(bool called_from_base)
{
  NodeCanopenProxyDriver<NODETYPE>::activate(false);
  enable_state_ = EnableState::Disabled;
  state_cycles_ = 0;
  sequence_done_ = false;
  rearm_attempts_ = 0;
  // Give a device that is still booting the full interval before resetting it.
  last_offline_reset_ = std::chrono::steady_clock::now();

  if (this->lely_driver_)
  {
    send_nmt_start("activate");
  }
}

template <class NODETYPE>
void NodeCanopenThomsonDriver<NODETYPE>::deactivate(bool called_from_base)
{
  disable();
  NodeCanopenProxyDriver<NODETYPE>::deactivate(false);
}

template <class NODETYPE>
uint16_t NodeCanopenThomsonDriver<NODETYPE>::to_raw(double position) const
{
  const double raw = std::round(position_conversion_.to_dev(position));
  return static_cast<uint16_t>(std::clamp(raw, static_cast<double>(min_raw_), static_cast<double>(max_raw_)));
}

template <class NODETYPE>
double NodeCanopenThomsonDriver<NODETYPE>::from_raw(uint16_t raw) const
{
  return position_conversion_.from_dev(static_cast<double>(raw));
}

template <class NODETYPE>
bool NodeCanopenThomsonDriver<NODETYPE>::set_target(double position)
{
  if (!std::isfinite(position))
  {
    return false;
  }
  target_position_ = position;
  target_valid_ = true;
  return true;
}

template <class NODETYPE>
double NodeCanopenThomsonDriver<NODETYPE>::get_position() const
{
  return from_raw(actual_raw_.load());
}

template <class NODETYPE>
double NodeCanopenThomsonDriver<NODETYPE>::get_speed() const
{
  return actual_speed_.load();
}

template <class NODETYPE>
uint16_t NodeCanopenThomsonDriver<NODETYPE>::get_current() const
{
  return actual_current_.load();
}

template <class NODETYPE>
uint8_t NodeCanopenThomsonDriver<NODETYPE>::get_status() const
{
  return status_.load();
}

template <class NODETYPE>
uint8_t NodeCanopenThomsonDriver<NODETYPE>::get_flags() const
{
  return flags_.load();
}

template <class NODETYPE>
void NodeCanopenThomsonDriver<NODETYPE>::enable()
{
  enable_requested_ = true;
}

template <class NODETYPE>
void NodeCanopenThomsonDriver<NODETYPE>::disable()
{
  enable_requested_ = false;
  target_valid_ = false;
}

template <class NODETYPE>
bool NodeCanopenThomsonDriver<NODETYPE>::has_communication_failure() const
{
  if (!feedback_received_.load())
  {
    return true;
  }
  return std::chrono::nanoseconds(steady_now_ns() - last_feedback_ns_.load()) > feedback_timeout_;
}

template <class NODETYPE>
bool NodeCanopenThomsonDriver<NODETYPE>::has_fault() const
{
  return (flags_.load() & fault_flag_mask_) != 0;
}

template <class NODETYPE>
bool NodeCanopenThomsonDriver<NODETYPE>::is_ready() const
{
  return !has_communication_failure() && !has_fault();
}

template <class NODETYPE>
bool NodeCanopenThomsonDriver<NODETYPE>::is_enabled() const
{
  return sequence_done_.load();
}

template <class NODETYPE>
void NodeCanopenThomsonDriver<NODETYPE>::request_rearm(const char* reason)
{
  if (!rearm_requested_.exchange(true))
  {
    RCLCPP_INFO(this->node_->get_logger(), "Thomson actuator '%s': re-arming motion enable (%s)", joint_name_.c_str(),
                reason);
  }
}

template <class NODETYPE>
void NodeCanopenThomsonDriver<NODETYPE>::send_nmt_start(const char* reason)
{
  // The actuator only accepts the command RPDO in OPERATIONAL. The master does not reliably start it:
  // its boot process aborts when the actuator rejects part of the configuration download. NMT Start
  // to an operational node does nothing, so sending it again is always safe.
  RCLCPP_INFO(this->node_->get_logger(), "Thomson actuator '%s': sending NMT Start (%s)", joint_name_.c_str(), reason);
  this->lely_driver_->nmt_command(canopen::NmtCommand::START);
}

template <class NODETYPE>
void NodeCanopenThomsonDriver<NODETYPE>::send_nmt_reset(const char* reason)
{
  // The boot-up that follows is handled like any other: NMT Start, then the enable sequence.
  RCLCPP_WARN(this->node_->get_logger(), "Thomson actuator '%s': sending NMT reset node (%s)", joint_name_.c_str(),
              reason);
  this->lely_driver_->nmt_command(canopen::NmtCommand::RESET_NODE);
  rearm_attempts_ = 0;
}

template <class NODETYPE>
template <typename T>
T NodeCanopenThomsonDriver<NODETYPE>::read_feedback(const ThomsonObject& object)
{
  // Not universal_get_value(): the feedback objects are mapped in a TPDO, so it would return the
  // master's local copy, which never fails and goes stale when the actuator is gone.
  // Not sync_sdo_read_typed() either: it logs every timeout, which floods the log while unpowered.
  auto fut = this->lely_driver_->template async_sdo_read_typed<T>(object.index, object.subindex);
  if (fut.wait_for(kSdoWait) != std::future_status::ready)
  {
    throw std::runtime_error("no SDO response");
  }
  return fut.get();  // rethrows an SDO abort or timeout
}

template <class NODETYPE>
void NodeCanopenThomsonDriver<NODETYPE>::write_command(uint16_t position_raw, uint8_t enable)
{
  // All objects live in one synchronous RPDO, so the master sends them together on the next SYNC.
  // Object sizes must match PDO mapping: pos(16), current(16), speed(8), aux(8), aux2(8), enable(8)
  this->lely_driver_->template universal_set_value<uint16_t>(cmd_position_.index, cmd_position_.subindex,
                                                             position_raw);
  this->lely_driver_->template universal_set_value<uint16_t>(cmd_current_limit_.index, cmd_current_limit_.subindex,
                                                             current_limit_);
  this->lely_driver_->template universal_set_value<uint8_t>(cmd_speed_.index, cmd_speed_.subindex, speed_);
  // aux is 16-bit config value split into two 8-bit PDO objects (low byte first, matching steer.sh)
  this->lely_driver_->template universal_set_value<uint8_t>(cmd_aux_.index, cmd_aux_.subindex,
                                                            static_cast<uint8_t>(aux_ & 0xFF));
  this->lely_driver_->template universal_set_value<uint8_t>(cmd_aux2_.index, cmd_aux2_.subindex,
                                                            static_cast<uint8_t>(aux_ >> 8));
  this->lely_driver_->template universal_set_value<uint8_t>(cmd_enable_.index, cmd_enable_.subindex, enable);
}

template <class NODETYPE>
void NodeCanopenThomsonDriver<NODETYPE>::poll_timer_callback()
{
  NodeCanopenProxyDriver<NODETYPE>::poll_timer_callback();

  if (!this->activated_.load() || !this->lely_driver_)
  {
    return;
  }

  const auto now = std::chrono::steady_clock::now();

  if (online_ || now - last_offline_probe_ >= kOfflineProbeInterval)
  {
    last_offline_probe_ = now;
    try
    {
      const uint16_t pos_raw = read_feedback<uint16_t>(fb_position_);
      if (feedback_received_.load())
      {
        const double dt = std::chrono::duration<double>(now - last_position_time_).count();
        if (dt > 0.0)
        {
          actual_speed_ = (from_raw(pos_raw) - from_raw(last_position_raw_)) / dt;
        }
      }
      last_position_raw_ = pos_raw;
      last_position_time_ = now;
      actual_raw_ = pos_raw;
      last_feedback_ns_ = steady_now_ns();
      feedback_received_ = true;

      actual_current_ = read_feedback<uint16_t>(fb_current_);
      status_ = read_feedback<uint8_t>(fb_status_);
      flags_ = read_feedback<uint8_t>(fb_flags_);
    }
    catch (const std::exception& e)
    {
      // Offline is reported by the lost/available messages; only a flaky but alive actuator is logged here.
      if (online_)
      {
        RCLCPP_WARN_THROTTLE(this->node_->get_logger(), *this->node_->get_clock(), 1000,
                             "Thomson actuator '%s': reading feedback failed: %s", joint_name_.c_str(), e.what());
      }
    }
  }

  // Recovery cycle. Offline: reset the node now and then. Coming online or booting: NMT Start, then
  // the enable sequence. Online but motion stays disabled: NMT Start and re-arm, reset after
  // kMaxRearmAttempts. A device that just booted (or was reset) is PRE-OPERATIONAL and ignores the
  // command RPDO until it gets NMT Start.
  const bool comm_failure = has_communication_failure();
  if (!comm_failure && !online_)
  {
    RCLCPP_INFO(this->node_->get_logger(), "Thomson actuator '%s': available now", joint_name_.c_str());
    send_nmt_start("came online");
    request_rearm("came online");
  }
  else if (comm_failure && online_)
  {
    RCLCPP_WARN(this->node_->get_logger(), "Thomson actuator '%s': lost communication", joint_name_.c_str());
    last_offline_reset_ = now;
  }
  online_ = !comm_failure;

  if (boot_up_seen_.exchange(false))
  {
    send_nmt_start("boot-up");
    request_rearm("boot-up");
  }

  if (comm_failure && offline_reset_interval_.count() > 0 && now - last_offline_reset_ >= offline_reset_interval_)
  {
    // Harmless while the actuator is unpowered, and the only way out if it hangs while powered.
    RCLCPP_WARN_THROTTLE(this->node_->get_logger(), *this->node_->get_clock(), 30000,
                         "Thomson actuator '%s': not answering, sending NMT reset node every %ld ms until it does",
                         joint_name_.c_str(), (long)offline_reset_interval_.count());
    this->lely_driver_->nmt_command(canopen::NmtCommand::RESET_NODE);
    last_offline_reset_ = now;
  }

  // A latched fault (e.g. overcurrent) keeps the actuator dead. Motion is disabled below; if the
  // fault does not clear, reset the node.
  const bool fault = has_fault();
  if (fault && !fault_seen_)
  {
    RCLCPP_ERROR(this->node_->get_logger(), "Thomson actuator '%s': fault (flags 0x%02X), disabling motion",
                 joint_name_.c_str(), flags_.load());
    last_fault_reset_ = now;
  }
  else if (!fault && fault_seen_)
  {
    RCLCPP_INFO(this->node_->get_logger(), "Thomson actuator '%s': fault cleared", joint_name_.c_str());
  }
  fault_seen_ = fault;
  if (fault && fault_reset_interval_.count() > 0 && now - last_fault_reset_ > fault_reset_interval_)
  {
    send_nmt_reset("fault persists");
    last_fault_reset_ = now;
  }

  const bool motion_disabled = (flags_.load() & rearm_flag_mask_) != 0;
  if (enable_state_ == EnableState::Enabled && motion_disabled && now - last_rearm_ > kRearmInterval)
  {
    if (++rearm_attempts_ > kMaxRearmAttempts)
    {
      send_nmt_reset("motion stays disabled after re-arming");
      last_rearm_ = now;
    }
    else
    {
      // Most likely PRE-OPERATIONAL, so the enable sequence never reached the actuator.
      send_nmt_start("actuator reports motion disabled");
      request_rearm("actuator reports motion disabled");
    }
  }
  else if (enable_state_ == EnableState::Enabled && !motion_disabled)
  {
    rearm_attempts_ = 0;
  }

  const bool want_enabled = enable_requested_.load() && !comm_failure && !has_fault();
  const EnableState previous = enable_state_;

  if (!want_enabled)
  {
    enable_state_ = EnableState::Disabled;
  }
  else if (rearm_requested_.exchange(false))
  {
    // From 1 go straight to 0, from 0 start with 1, so the device always sees 1 -> 0 -> 1.
    enable_state_ = (enable_state_ == EnableState::Disabled) ? EnableState::ArmHigh : EnableState::ArmLow;
    last_rearm_ = now;
  }
  else if (enable_state_ == EnableState::Disabled)
  {
    enable_state_ = EnableState::ArmHigh;
    last_rearm_ = now;
  }
  else if (enable_state_ == EnableState::ArmHigh && state_cycles_ >= enable_step_cycles_)
  {
    enable_state_ = EnableState::ArmLow;
  }
  else if (enable_state_ == EnableState::ArmLow && state_cycles_ >= enable_step_cycles_)
  {
    enable_state_ = EnableState::Enabled;
    RCLCPP_INFO(this->node_->get_logger(), "Thomson actuator '%s': motion enabled", joint_name_.c_str());
  }

  state_cycles_ = (enable_state_ == previous) ? state_cycles_ + 1 : 0;
  if (!want_enabled)
  {
    // Nothing to re-arm while disabled; enabling always runs the full sequence.
    rearm_requested_ = false;
  }
  sequence_done_ = (enable_state_ == EnableState::Enabled);

  // Until the sequence is done, command the actual position so enabling never makes a jump.
  uint16_t position_raw = actual_raw_.load();
  if (enable_state_ == EnableState::Enabled && target_valid_.load())
  {
    position_raw = to_raw(target_position_.load());
  }

  const uint8_t enable_byte = (enable_state_ == EnableState::ArmHigh || enable_state_ == EnableState::Enabled) ? 1 : 0;

  try
  {
    write_command(position_raw, enable_byte);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_THROTTLE(this->node_->get_logger(), *this->node_->get_clock(), 1000,
                          "Thomson actuator '%s': writing command failed: %s", joint_name_.c_str(), e.what());
  }
}

template <class NODETYPE>
void NodeCanopenThomsonDriver<NODETYPE>::on_nmt(canopen::NmtState nmt_state)
{
  NodeCanopenProxyDriver<NODETYPE>::on_nmt(nmt_state);
  // The bridge reports a boot-up as START. The actuator is PRE-OPERATIONAL then and waits for NMT
  // Start and a new 1 -> 0 -> 1. Handled by the poll timer, which owns the recovery cycle.
  if (nmt_state == canopen::NmtState::START)
  {
    boot_up_seen_ = true;
  }
}

template <class NODETYPE>
void NodeCanopenThomsonDriver<NODETYPE>::on_rpdo(COData data)
{
  NodeCanopenProxyDriver<NODETYPE>::on_rpdo(data);

  auto is = [&data](const ThomsonObject& object) {
    return data.index_ == object.index && data.subindex_ == object.subindex;
  };

  if (is(fb_position_))
  {
    const uint16_t raw = static_cast<uint16_t>(data.data_ & 0xFFFF);
    const auto now = std::chrono::steady_clock::now();
    if (feedback_received_.load())
    {
      const double dt = std::chrono::duration<double>(now - last_position_time_).count();
      if (dt > 0.0)
      {
        actual_speed_ = (from_raw(raw) - from_raw(last_position_raw_)) / dt;
      }
    }
    last_position_raw_ = raw;
    last_position_time_ = now;
    actual_raw_ = raw;
    last_feedback_ns_ = steady_now_ns();
    feedback_received_ = true;
  }
  else if (is(fb_current_))
  {
    actual_current_ = static_cast<uint16_t>(data.data_ & 0xFFFF);
  }
  else if (is(fb_status_))
  {
    status_ = static_cast<uint8_t>(data.data_ & 0xFF);
  }
  else if (is(fb_flags_))
  {
    flags_ = static_cast<uint8_t>(data.data_ & 0xFF);
  }
}

template <class NODETYPE>
void NodeCanopenThomsonDriver<NODETYPE>::diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  NodeCanopenProxyDriver<NODETYPE>::diagnostic_callback(stat);
  stat.add("joint_name", joint_name_);
  stat.add("enabled", is_enabled() ? "true" : "false");
  stat.add("communication_failure", has_communication_failure() ? "true" : "false");
  stat.add("fault", has_fault() ? "true" : "false");
  stat.add("rearm_attempts", std::to_string(rearm_attempts_));
  stat.add("position_raw", std::to_string(actual_raw_.load()));
  stat.add("current_raw", std::to_string(actual_current_.load()));
  stat.add("status", std::to_string(status_.load()));
  stat.add("flags", std::to_string(flags_.load()));
}

#endif  // NODE_CANOPEN_THOMSON_DRIVER_IMPL_HPP_
