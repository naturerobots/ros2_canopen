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

#include "canopen_pre_mapped_driver/motor.hpp"
using namespace ros2_canopen;

bool Motor402::setTarget(double val)
{
  if (state_handler_.getState() == State402::Operation_Enable) {
    auto target = val * scale_vel_to_dev_;
    return selected_mode_ && selected_mode_->setTarget(target);
  }
  return false;
}

uint16_t Motor402::getMode()
{
  return selected_mode_->mode_id_;
}

bool Motor402::readState()
{
  if (this->driver == nullptr || this->status_word_entry_index == 0 || op_mode_display_index == 0) {
    return false;
  }

  try {
    uint16_t sw = driver->universal_get_value<uint16_t>(status_word_entry_index, 0x0);    // TODO: added error handling
    status_word_.exchange(sw);

    state_handler_.read(sw);

    uint16_t new_mode;
    new_mode = driver->universal_get_value<int8_t>(op_mode_display_index, 0x0);
    // RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Mode %hhi",new_mode);

    if (selected_mode_ && selected_mode_->mode_id_ == new_mode) {
      if (!selected_mode_->read(sw)) {
        RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Mode handler has error.");
      }
    }
    if (new_mode != mode_id_) {
      mode_id_ = new_mode;
      mode_cond_.notify_all();
    }
    if (selected_mode_ && selected_mode_->mode_id_ != new_mode) {
      RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Mode does not match.");
    }

    // communication worked well
    has_communication_failure_ = false;
  } catch (std::runtime_error & e) {
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
  if (this->driver == nullptr || this->control_word_entry_index == 0) {
    return;
  }
  if (state_handler_.getState() == State402::Operation_Enable) {
  } else {
  }
}
void Motor402::handleDiag()
{
  uint16_t sw = status_word_;
  State402::InternalState state = state_handler_.getState();
  uint16_t mode = getMode();
  this->diag_collector_->addf(joint_name_ + "_cia402_mode", "%i", mode);

  switch (state) {
    case State402::Not_Ready_To_Switch_On:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Not ready to switch on");
      this->diag_collector_->summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Not ready to switch on");
      initialized_ = false;
      break;
    case State402::Switch_On_Disabled:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Switch on disabled");
      this->diag_collector_->summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Switch on disabled");
      initialized_ = false;
      break;
    case State402::Ready_To_Switch_On:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Ready to switch on");
      this->diag_collector_->summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "Ready to switch on");
      initialized_ = false;
      break;
    case State402::Switched_On:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Switched on");
      this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Switched on");
      initialized_ = false;
      break;
    case State402::Operation_Enable:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Operation enabled");
      this->diag_collector_->summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "Operation enabled");
      break;
    case State402::Quick_Stop_Active:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Quick stop active");
      this->diag_collector_->summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Quick stop active");
      break;
    case State402::Fault:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Fault");
      this->diag_collector_->summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Fault");
      break;
    case State402::Fault_Reaction_Active:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Fault reaction active");
      this->diag_collector_->summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Fault reaction active");
      break;
    case State402::Unknown:
      this->diag_collector_->addf(joint_name_ + "_cia402_state", "Unknown state");
      this->diag_collector_->summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "Unknown state");
      initialized_ = false;
      break;
  }

  if (sw & (1 << State402::SW_Warning)) {
    this->diag_collector_->addf(joint_name_ + "_cia402_state", "Warning bit is set");
    this->diag_collector_->summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Warning bit is set");
  }
  if (sw & (1 << State402::SW_Internal_limit)) {
    this->diag_collector_->addf(joint_name_ + "_cia402_state", "Internal limit active");
    this->diag_collector_->summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "Internal limit active");
  }

  this->diag_collector_->addf(joint_name_ + "_cia402_is_initialized", "%i", (int)initialized_);
  this->diag_collector_->addf(
    joint_name_ + "_cia402_has_communication_failure", "%i",
    (int)has_communication_failure_);

  if (has_communication_failure_) {
    this->diag_collector_->summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "A communication failure occurred");
  }

  if (!initialized_) {
    this->diag_collector_->summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "Motor is not initialized");
  }
}

bool Motor402::handleInit()
{

  status_word_entry_index = 0x6041;
  control_word_entry_index = 0x6040;
  op_mode_index = 0x6060;
  op_mode_display_index = 0x6061;
  supported_drive_modes_index = 0x6502;
  speed_feedback_index = 0x606C;
  position_feedback_index = 0x6064;

  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Read State");
  if (!readState()) {
    std::cout << "Could not read motor state" << std::endl;
    return false;
  }
  start_fault_reset_ = true;
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Enable");
  initialized_ = true;
  return true;
}
bool Motor402::handleShutdown()
{
  return true;
}

bool Motor402::handleHalt()
{
  State402::InternalState state = state_handler_.getState();

  // do not demand quickstop in case of fault
  if (state == State402::Fault_Reaction_Active || state == State402::Fault) {
    return false;
  }
  return true;
}
bool Motor402::handleRecover()
{
  start_fault_reset_ = true;
  {
    if (selected_mode_ && !selected_mode_->start()) {
      std::cout << "Could not restart mode." << std::endl;
      return false;
    }
  }
  return true;
}

bool Motor402::isFaulty()
{
  State402::InternalState state = state_handler_.getState();
  if (state != State402::Operation_Enable && state != State402::Quick_Stop_Active) {
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
  if (state == State402::Quick_Stop_Active) {
    return true;
  }
  return false;
}
