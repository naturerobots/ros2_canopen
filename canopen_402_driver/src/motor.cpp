//    Copyright 2023 Christoph Hellmann Santos
//    Copyright 2023 Vishnuprasad Prachandabhanu
//    Copyright 2014-2022 Authors of ros_canopen
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

#include <cmath>
#include "canopen_402_driver/motor.hpp"
#include "canopen_402_driver/homing_mode.hpp"
using namespace ros2_canopen;

namespace
{
/// printf-style helper for the short failure reasons stored on each motor.
template <typename... Args>
std::string reason(const char* format, Args... args)
{
  char buffer[160];
  std::snprintf(buffer, sizeof(buffer), format, args...);
  return std::string(buffer);
}
}  // namespace

std::string Motor402::getLastError()
{
  std::scoped_lock lock(last_error_mutex_);
  return last_error_;
}

void Motor402::setLastError(const std::string& text)
{
  {
    std::scoped_lock lock(last_error_mutex_);
    last_error_ = text;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("canopen_402_driver"), "%s: %s", joint_name_.c_str(), text.c_str());
}

bool Motor402::setTarget(double val)
{
  if (state_handler_.getState() == State402::Operation_Enable)
  {
    auto mode = getMode();
    double target;
    if ((mode == MotorBase::Profiled_Position) or (mode == MotorBase::Cyclic_Synchronous_Position) or
        (mode == MotorBase::Interpolated_Position))
    {
      target = val * scale_pos_to_dev_;
    }
    else if ((mode == MotorBase::Velocity) or (mode == MotorBase::Profiled_Velocity) or
             (mode == MotorBase::Cyclic_Synchronous_Velocity))
    {
      target = val * scale_vel_to_dev_;
    }

    std::scoped_lock lock(mode_mutex_);
    return selected_mode_ && selected_mode_->setTarget(target);
  }
  return false;
}
bool Motor402::isModeSupported(uint16_t mode)
{
  return mode != MotorBase::Homing && allocMode(mode);
}

bool Motor402::enterModeAndWait(uint16_t mode)
{
  bool okay = mode != MotorBase::Homing && switchMode(mode);
  return okay;
}

uint16_t Motor402::getMode()
{
  std::scoped_lock lock(mode_mutex_);
  return selected_mode_ ? selected_mode_->mode_id_ : (uint16_t)MotorBase::No_Mode;
}

bool Motor402::isModeSupportedByDevice(uint16_t mode, uint8_t channel)
{
  // Must be initialized: 0x6502 & friends are not PDO mapped, so this is a real SDO read that
  // can time out. The catch below then leaves the value untouched and we would test garbage.
  uint32_t supported_modes = 0;
  bool read_ok = true;
  try
  {
    if (channel == 1)
    {
      supported_modes = driver->universal_get_value<uint32_t>(0x6502, 0x0);
    }
    else if (channel == 2)
    {
      supported_modes = driver->universal_get_value<uint32_t>(0x6D02, 0x0);
    }
    else if (channel == 3)
    {
      supported_modes = driver->universal_get_value<uint32_t>(0x7502, 0x0);
    }
    // communication worked well
    has_communication_failure_ = false;
  }
  catch (std::runtime_error& e)
  {
    // communication was unsuccessful
    has_communication_failure_ = true;
    read_ok = false;
  }

  if (!read_ok)
  {
    setLastError(reason("could not read supported drive modes, treating mode %u as unsupported", mode));
    return false;
  }

  const bool supported = supported_modes & (1u << (mode - 1));
  if (!supported)
  {
    setLastError(reason("device reports supported drive modes 0x%08X, mode %u not among them", supported_modes, mode));
  }

  return mode > 0 && mode <= 32 && supported;
}
void Motor402::registerMode(uint16_t id, const ModeSharedPtr& m, uint8_t channel)
{
  std::scoped_lock map_lock(map_mutex_);
  if (m && m->mode_id_ == id)
  {
    if (channel == 1)
    {
      modes1_.insert(std::make_pair(id, m));
    }
    if (channel == 2)
    {
      modes2_.insert(std::make_pair(id, m));
    }
    if (channel == 3)
    {
      modes3_.insert(std::make_pair(id, m));
    }
  }
}

ModeSharedPtr Motor402::allocMode(uint16_t mode)
{
  ModeSharedPtr res;
  if (isModeSupportedByDevice(mode, channel_))
  {
    std::scoped_lock map_lock(map_mutex_);
    if (channel_ == 1)
    {
      std::unordered_map<uint16_t, ModeSharedPtr>::iterator it = modes1_.find(mode);
      if (it != modes1_.end())
      {
        res = it->second;
      }
    }
    else if (channel_ == 2)
    {
      std::unordered_map<uint16_t, ModeSharedPtr>::iterator it = modes2_.find(mode);
      if (it != modes2_.end())
      {
        res = it->second;
      }
    }
    else if (channel_ == 3)
    {
      std::unordered_map<uint16_t, ModeSharedPtr>::iterator it = modes3_.find(mode);
      if (it != modes3_.end())
      {
        res = it->second;
      }
    }
  }
  return res;
}

bool Motor402::switchMode(uint16_t mode)
{
  if (mode == MotorBase::No_Mode)
  {
    std::scoped_lock lock(mode_mutex_);
    selected_mode_.reset();
    try
    {  // try to set mode
      driver->universal_set_value<int8_t>(op_mode_index, 0x0, mode);
    }
    catch (...)
    {
    }
    if (enable_diagnostics_.load())
    {
      this->diag_collector_->addf(joint_name_ + "_cia402_set_mode", "No mode selected: %d", mode);
    }
    return true;
  }

  ModeSharedPtr next_mode = allocMode(mode);
  if (!next_mode)
  {
    setLastError(reason("mode %u not supported by device", mode));
    return false;
  }

  if (!next_mode->start())
  {
    setLastError(reason("could not start mode %u", mode));
    return false;
  }

  {  // disable mode handler
    std::scoped_lock lock(mode_mutex_);

    if (mode_id_ == mode && selected_mode_ && selected_mode_->mode_id_ == mode)
    {
      // nothing to do
      return true;
    }

    selected_mode_.reset();
  }

  if (!switchState(switching_state_))
    return false;

  driver->universal_set_value<int8_t>(op_mode_index, 0x0, mode);

  bool okay = false;

  {  // wait for switch
    std::unique_lock lock(mode_mutex_);

    try
    {
      std::chrono::steady_clock::time_point abstime = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      if (monitor_mode_)
      {
        while (mode_id_ != mode && mode_cond_.wait_until(lock, abstime) == std::cv_status::no_timeout)
        {
        }
      }
      else
      {
        while (mode_id_ != mode && std::chrono::steady_clock::now() < abstime)
        {
          lock.unlock();                                                    // unlock inside loop
          driver->universal_get_value<int8_t>(op_mode_display_index, 0x0);  // poll
          std::this_thread::sleep_for(std::chrono::milliseconds(20));       // wait some time
          lock.lock();
        }
      }
      has_communication_failure_ = false;
    }
    catch (std::runtime_error& e)
    {
      // communication was unsuccessful
      has_communication_failure_ = true;
    }

    if (mode_id_ == mode)
    {
      selected_mode_ = next_mode;
      okay = true;
      if (enable_diagnostics_.load())
      {
      }
    }
    else
    {
      setLastError(reason("mode switch to %u timed out", mode));
      driver->universal_set_value<int8_t>(op_mode_index, 0x0, mode_id_);
      if (enable_diagnostics_.load())
      {
        this->diag_collector_->addf(joint_name_ + "_cia402_set_mode", "Mode switch timed out: %d", mode);
      }
    }
  }

  if (!switchState(State402::Operation_Enable))
    return false;

  return okay;
}

bool Motor402::switchState(const State402::InternalState& target)
{
  std::chrono::steady_clock::time_point abstime = std::chrono::steady_clock::now() + state_switch_timeout_;
  State402::InternalState state = state_handler_.getState();
  target_state_ = target;
  while (state != target_state_)
  {
    std::unique_lock lock(cw_mutex_);
    State402::InternalState next = State402::Unknown;
    bool success = Command402::setTransition(control_word_, state, target_state_, &next);
    if (!success)
    {
      setLastError(reason("no transition from state %d to %d", static_cast<int>(state),
                        static_cast<int>(target_state_.load())));
      return false;
    }
    else if (enable_diagnostics_.load() && success)
    {
      this->diag_collector_->addf(joint_name_ + "_cia402_set_state", "State switched to: %d", next);
    }
    lock.unlock();
    if (state != next && !state_handler_.waitForNewState(abstime, state))
    {
      setLastError(reason("state transition %d -> %d timed out", static_cast<int>(state), static_cast<int>(next)));
      if (enable_diagnostics_.load())
      {
        this->diag_collector_->addf(joint_name_ + "_cia402_set_state", "State transition timed out: %d -> %d", state,
                                    next);
      }
      return false;
    }
  }
  return state == target;
}

bool Motor402::readState()
{
  if (this->driver == nullptr || this->status_word_entry_index == 0 || op_mode_display_index == 0)
  {
    return false;
  }

  try
  {
    uint16_t old_sw,
        sw = driver->universal_get_value<uint16_t>(status_word_entry_index, 0x0);  // TODO: added error handling
    old_sw = status_word_.exchange(sw);

    state_handler_.read(sw);

    std::unique_lock lock(mode_mutex_);
    uint16_t new_mode;
    new_mode = driver->universal_get_value<int8_t>(op_mode_display_index, 0x0);
    // RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Mode %hhi",new_mode);

    if (selected_mode_ && selected_mode_->mode_id_ == new_mode)
    {
      if (!selected_mode_->read(sw))
      {
        setLastError("mode handler has error");
      }
    }
    if (new_mode != mode_id_)
    {
      mode_id_ = new_mode;
      mode_cond_.notify_all();
    }
    if (selected_mode_ && selected_mode_->mode_id_ != new_mode)
    {
      setLastError(reason("mode mismatch: handler is %u, device reports %u",
                          static_cast<unsigned>(selected_mode_->mode_id_), static_cast<unsigned>(new_mode)));
    }

    // communication worked well
    has_communication_failure_ = false;
  }
  catch (std::runtime_error& e)
  {
    // communication was unsuccessful
    has_communication_failure_ = true;
  }

  return true;
}
void Motor402::handleRead()
{
  readState();
}
void Motor402::handleWrite()
{
  if (this->driver == nullptr || this->control_word_entry_index == 0)
  {
    return;
  }

  std::scoped_lock lock(cw_mutex_);
  control_word_ |= (1 << Command402::CW_Halt);
  if (state_handler_.getState() == State402::Operation_Enable)
  {
    std::scoped_lock lock(mode_mutex_);
    Mode::OpModeAccesser cwa(control_word_);
    bool okay = false;
    if (selected_mode_ && selected_mode_->mode_id_ == mode_id_)
    {
      okay = selected_mode_->write(cwa);
    }
    else
    {
      cwa = 0;
    }
    if (okay)
    {
      control_word_ &= ~(1 << Command402::CW_Halt);
    }
  }
  if (start_fault_reset_.exchange(false))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("canopen_402_driver"), "%s: fault reset", joint_name_.c_str());
    this->driver->universal_set_value<uint16_t>(control_word_entry_index, 0x0,
                                                control_word_ & ~(1 << Command402::CW_Fault_Reset));
  }
  else
  {
    // RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Control Word %s",
    // std::bitset<16>{control_word_}.to_string());
    this->driver->universal_set_value<uint16_t>(control_word_entry_index, 0x0, control_word_);
  }
}
void Motor402::handleDiag()
{
  uint16_t sw = status_word_;
  State402::InternalState state = state_handler_.getState();
  uint16_t mode = getMode();
  this->diag_collector_->addf(joint_name_ + "_cia402_mode", "%i", mode);

  // Observes only. Clearing initialized_ here let the 1 Hz diagnostics tick drive the control
  // logic and re-init already-enabled drives. initialized_ is owned by handleInit(); live
  // health is isFaulty().
  switch (state)
  {
    case State402::Not_Ready_To_Switch_On:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Not ready to switch on");
      this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Not ready to switch on");
      break;
    case State402::Switch_On_Disabled:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Switch on disabled");
      this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Switch on disabled");
      break;
    case State402::Ready_To_Switch_On:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Ready to switch on");
      this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Ready to switch on");
      break;
    case State402::Switched_On:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Switched on");
      this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Switched on");
      break;
    case State402::Operation_Enable:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Operation enabled");
      this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Operation enabled");
      break;
    case State402::Quick_Stop_Active:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Quick stop active");
      this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Quick stop active");
      break;
    case State402::Fault:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Fault");
      this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Fault");
      break;
    case State402::Fault_Reaction_Active:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Fault reaction active");
      this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Fault reaction active");
      break;
    case State402::Unknown:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Unknown state");
      this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unknown state");
      break;
  }

  if (sw & (1 << State402::SW_Warning))
  {
    this->diag_collector_->addf(joint_name_ + "_cia402_state", "Warning bit is set");
    this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Warning bit is set");
  }
  if (sw & (1 << State402::SW_Internal_limit))
  {
    this->diag_collector_->addf(joint_name_ + "_cia402_state", "Internal limit active");
    this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Internal limit active");
  }

  this->diag_collector_->addf(joint_name_ + "_cia402_is_initialized", "%i", (int)initialized_);
  this->diag_collector_->addf(joint_name_ + "_cia402_has_communication_failure", "%i", (int)has_communication_failure_);

  if (has_communication_failure_)
  {
    this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "A communication failure occurred");
  }

  if (!initialized_)
  {
    this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Motor is not initialized");
  }

  // Reported by the hardware interface's RPDO watchdog. Deliberately checked after the
  // state machine: the drive looks perfectly healthy in CiA402 terms while this is set.
  // The key is always published; the summary level only changes for a confirmed fault,
  // because consumers treat any non-OK motor status as "system not ok".
  {
    std::lock_guard<std::mutex> lock(motion_status_mutex_);
    this->diag_collector_->addf(joint_name_ + "_rpdo_watchdog", "%s",
                                motion_status_.empty() ? "ok" : motion_status_.c_str());
    if (motion_error_)
    {
      this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                                     "RPDO config fault: " + motion_status_);
    }
  }
}

bool Motor402::handleInit()
{
  if (channel_ == 1)
  {
    status_word_entry_index = 0x6041;
    control_word_entry_index = 0x6040;
    op_mode_index = 0x6060;
    op_mode_display_index = 0x6061;
    supported_drive_modes_index = 0x6502;
    speed_feedback_index = 0x606C;
    position_feedback_index = 0x6064;

    for (std::unordered_map<uint16_t, AllocFuncType>::iterator it = mode_allocators1_.begin();
         it != mode_allocators1_.end(); ++it)
    {
      (it->second)();
    }
  }
  else if (channel_ == 2)
  {
    status_word_entry_index = 0x6841;
    control_word_entry_index = 0x6840;
    op_mode_index = 0x6860;
    op_mode_display_index = 0x6861;
    supported_drive_modes_index = 0x6D02;
    speed_feedback_index = 0x686C;
    position_feedback_index = 0x6864;

    for (std::unordered_map<uint16_t, AllocFuncType>::iterator it = mode_allocators2_.begin();
         it != mode_allocators2_.end(); ++it)
    {
      (it->second)();
    }
  }
  else if (channel_ == 3)
  {
    status_word_entry_index = 0x7041;
    control_word_entry_index = 0x7040;
    op_mode_index = 0x7060;
    op_mode_display_index = 0x7061;
    supported_drive_modes_index = 0x7502;
    speed_feedback_index = 0x706C;
    position_feedback_index = 0x7064;

    for (std::unordered_map<uint16_t, AllocFuncType>::iterator it = mode_allocators3_.begin();
         it != mode_allocators3_.end(); ++it)
    {
      (it->second)();
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("canopen_402_driver"),
                        "Init: Illegal channel " << (int)channel_ << " for motor at joint " << joint_name_);
    return false;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("canopen_402_driver"), "%s: init - read state", joint_name_.c_str());
  if (!readState())
  {
    setLastError("init: could not read status word");
    return false;
  }
  {
    std::scoped_lock lock(cw_mutex_);
    // switchState() below returns immediately when already in the target state, so clearing
    // the control word here would leave it at 0. handleWrite() then sends 0x0100 - Halt with
    // Enable Voltage cleared, i.e. "Disable voltage" - dropping the drive straight back out.
    if (state_handler_.getState() != State402::Operation_Enable)
    {
      control_word_ = 0;
    }
    start_fault_reset_ = true;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("canopen_402_driver"), "%s: init - enable", joint_name_.c_str());
  if (!switchState(State402::Operation_Enable))
  {
    setLastError("init: could not reach Operation_Enable");
    return false;
  }

  // ! Homing mode on initialize deactivated
  // ModeSharedPtr m = allocMode(MotorBase::Homing);
  // if (!m)
  // {
  //   std::cout << "Homeing mode not supported" << std::endl;
  //   return true;  // homing not supported
  // }

  // HomingMode* homing = dynamic_cast<HomingMode*>(m.get());

  // if (!homing)
  // {
  //   std::cout << "Homing mode has incorrect handler" << std::endl;
  //   return false;
  // }
  // RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Switch to homing");
  // if (!switchMode(MotorBase::Homing))
  // {
  //   std::cout << "Could not enter homing mode" << std::endl;
  //   return false;
  // }
  // RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Execute homing");
  // if (!homing->executeHoming())
  // {
  //   std::cout << "Homing failed" << std::endl;
  //   return false;
  // }
  RCLCPP_DEBUG(rclcpp::get_logger("canopen_402_driver"), "%s: init - switch to no mode", joint_name_.c_str());
  if (!switchMode(MotorBase::No_Mode))
  {
    setLastError("init: could not enter No_Mode");
    return false;
  }

  initialized_ = true;
  return true;
}
bool Motor402::handleShutdown()
{
  switchMode(MotorBase::No_Mode);
  return switchState(State402::Switch_On_Disabled);
}
bool Motor402::handleHalt()
{
  State402::InternalState state = state_handler_.getState();
  std::scoped_lock lock(cw_mutex_);

  // do not demand quickstop in case of fault
  if (state == State402::Fault_Reaction_Active || state == State402::Fault)
    return false;

  if (state != State402::Operation_Enable)
  {
    target_state_ = state;
  }
  else
  {
    target_state_ = State402::Quick_Stop_Active;
    if (!Command402::setTransition(control_word_, state, State402::Quick_Stop_Active, 0))
    {
      RCLCPP_WARN(rclcpp::get_logger("canopen_402_driver"), "%s: could not quick stop", joint_name_.c_str());
      return false;
    }
  }
  return true;
}
bool Motor402::handleRecover()
{
  if (is_homing_) return false;  // don't interfere with homing
  return recoverInternal();
}

bool Motor402::recoverInternal()
{
  start_fault_reset_ = true;
  {
    std::scoped_lock lock(mode_mutex_);
    if (selected_mode_ && !selected_mode_->start())
    {
      setLastError("recover: could not restart mode");
      return false;
    }
  }
  if (!switchState(State402::Operation_Enable))
  {
    setLastError("recover: could not reach Operation_Enable");
    return false;
  }
  return true;
}

bool Motor402::handleHoming()
{
  if (homing_method_ == 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("canopen_402_driver"), "Homing method not configured");
    return false;
  }

  is_homing_ = true;
  struct HomingGuard { std::atomic<bool>& flag; ~HomingGuard() { flag = false; } } guard{is_homing_};

  // Determine CANopen indices based on channel
  uint16_t homing_method_index;
  uint16_t home_offset_index;
  uint16_t homing_speed_index;
  if (channel_ == 1)
  {
    homing_method_index = 0x6098;
    home_offset_index = 0x607C;
    homing_speed_index = 0x6099;
  }
  else if (channel_ == 2)
  {
    homing_method_index = 0x6898;
    home_offset_index = 0x687C;
    homing_speed_index = 0x6899;
  }
  else if (channel_ == 3)
  {
    homing_method_index = 0x7098;
    home_offset_index = 0x707C;
    homing_speed_index = 0x7099;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("canopen_402_driver"), "Invalid channel for homing");
    return false;
  }

  // Recover to operation enable state (halt is done by coordinator for all motors).
  // Uses recoverInternal() directly: handleRecover() refuses to run once is_homing_ is set
  // (set above, to keep other callers like MotorManager out), which would otherwise make
  // this bootstrap step fail every time.
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Homing: Enabling motor %s", joint_name_.c_str());
  if (!recoverInternal())
  {
    RCLCPP_ERROR(rclcpp::get_logger("canopen_402_driver"), "Could not enable motor for homing");
    return false;
  }

  // Write homing method
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Homing: Setting homing method %d for %s",
              homing_method_, joint_name_.c_str());
  try
  {
    driver->universal_set_value<int8_t>(homing_method_index, 0x0, homing_method_);
  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("canopen_402_driver"), "Failed to set homing method: %s", e.what());
    return false;
  }

  // Write homing speed (search-for-switch and search-for-zero), if configured
  if (homing_speed_ > 0.0)
  {
    uint32_t speed_dev = static_cast<uint32_t>(std::fabs(homing_speed_ * scale_vel_to_dev_));
    RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Homing: Setting homing speed %.4f rad/s (%u dev) for %s",
                homing_speed_, speed_dev, joint_name_.c_str());
    try
    {
      driver->universal_set_value<uint32_t>(homing_speed_index, 0x1, speed_dev);
      driver->universal_set_value<uint32_t>(homing_speed_index, 0x2, speed_dev);
    }
    catch (std::exception& e)
    {
      RCLCPP_WARN(rclcpp::get_logger("canopen_402_driver"), "Failed to set homing speed: %s", e.what());
      // Continue anyway - drive will use its previously configured/default homing speed
    }
  }

  // Switch to homing mode
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Homing: Switching to homing mode for %s", joint_name_.c_str());
  ModeSharedPtr m = allocMode(MotorBase::Homing);
  if (!m)
  {
    RCLCPP_ERROR(rclcpp::get_logger("canopen_402_driver"), "Homing mode not supported");
    return false;
  }

  HomingMode* homing = dynamic_cast<HomingMode*>(m.get());
  if (!homing)
  {
    RCLCPP_ERROR(rclcpp::get_logger("canopen_402_driver"), "Homing mode has incorrect handler");
    return false;
  }

  if (!switchMode(MotorBase::Homing))
  {
    RCLCPP_ERROR(rclcpp::get_logger("canopen_402_driver"), "Could not enter homing mode");
    return false;
  }

  // Execute homing
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Homing: Executing homing for %s", joint_name_.c_str());
  if (!homing->executeHoming())
  {
    RCLCPP_ERROR(rclcpp::get_logger("canopen_402_driver"), "Homing execution failed");
    switchMode(MotorBase::No_Mode);
    return false;
  }

  // Set home offset
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Homing: Setting home offset %.4f for %s",
              home_offset_, joint_name_.c_str());
  try
  {
    int32_t offset_dev = static_cast<int32_t>(home_offset_ * scale_pos_to_dev_);
    driver->universal_set_value<int32_t>(home_offset_index, 0x0, offset_dev);
  }
  catch (std::exception& e)
  {
    RCLCPP_WARN(rclcpp::get_logger("canopen_402_driver"), "Failed to set home offset: %s", e.what());
    // Continue anyway - homing succeeded
  }

  // Switch back to no mode, then to default operation mode
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Homing: Returning to normal operation for %s",
              joint_name_.c_str());
  if (!switchMode(MotorBase::No_Mode))
  {
    RCLCPP_WARN(rclcpp::get_logger("canopen_402_driver"), "Could not exit homing mode");
  }

  if (default_operation_mode_ != 0)
  {
    if (!switchMode(default_operation_mode_))
    {
      RCLCPP_WARN(rclcpp::get_logger("canopen_402_driver"), "Could not switch to default operation mode");
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Homing: Complete for %s", joint_name_.c_str());
  return true;
}

bool Motor402::isFaulty()
{
  State402::InternalState state = state_handler_.getState();
  if (state != State402::Operation_Enable && state != State402::Quick_Stop_Active)
  {
    return true;
  }
  return false;
}

bool Motor402::hasCommunicationFailure()
{
  return has_communication_failure_;
}

bool Motor402::isInitialized()
{
  return initialized_;
}

bool Motor402::isHalted()
{
  State402::InternalState state = state_handler_.getState();
  if (state == State402::Quick_Stop_Active)
  {
    return true;
  }
  return false;
}