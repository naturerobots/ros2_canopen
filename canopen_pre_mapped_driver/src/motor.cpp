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

bool PreMappedMotor::setTarget(double val, uint8_t rollover)
{
  if (target_helper_) {
    return target_helper_->setTarget(val, rollover);
  }
  return false;
}

bool PreMappedMotor::readState()
{
  if (this->driver == nullptr || this->status_word_entry_index == 0 || op_mode_display_index == 0) {
    return false;
  }
  try {
    uint16_t sw = driver->universal_get_value<uint16_t>(status_word_entry_index, 0x0);    // TODO: added error handling
    status_word_.exchange(sw);
    // communication worked well
    has_communication_failure_ = false;
  } catch (std::runtime_error & e) {
    // communication was unsuccessful
    has_communication_failure_ = true;
  }

  return true;
}

void PreMappedMotor::handleRead()
{
  readState();
}

void PreMappedMotor::handleWrite()
{
  if (this->driver == nullptr) {
    return;
  }
  target_helper_->write();
}
void PreMappedMotor::handleDiag()
{
  this->diag_collector_->addf(
    joint_name_ + "_pre_mapped_motor_is_initialized", "%i",
    (int)initialized_);
  this->diag_collector_->addf(
    joint_name_ + "_pre_mapped_motor_has_communication_failure", "%i",
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

bool PreMappedMotor::handleInit()
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
bool PreMappedMotor::handleShutdown()
{
  return true;
}

bool PreMappedMotor::handleHalt()
{
  RCLCPP_WARN_ONCE(
    rclcpp::get_logger("canopen_402_driver"),
    "Handle halt is not implemented in PreMappedMotor. Nothing will happen.");
  return true;
}

bool PreMappedMotor::handleRecover()
{
  start_fault_reset_ = true;
  RCLCPP_WARN_ONCE(
    rclcpp::get_logger("canopen_402_driver"),
    "Handle recover is not implemented in PreMappedMotor. Nothing will happen.");
  return true;
}

bool PreMappedMotor::isFaulty()
{
  RCLCPP_WARN_ONCE(
    rclcpp::get_logger("canopen_402_driver"),
    "Handle 'isFaulty' is not implemented in PreMappedMotor. No fault state is tracked.");
  return false;
}

bool PreMappedMotor::hasCommunicationFailure()
{
  return has_communication_failure_;
}

bool PreMappedMotor::isInitialized()
{
  return initialized_;
}

bool PreMappedMotor::isHalted()
{
  RCLCPP_WARN_ONCE(
    rclcpp::get_logger("canopen_402_driver"),
    "Handle 'isHalted' is not implemented in PreMappedMotor. No halt state is tracked.");
  return false;
}
