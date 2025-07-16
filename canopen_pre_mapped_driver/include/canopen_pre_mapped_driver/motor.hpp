//    Copyright 2023 Christoph Hellmann Santos
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

#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <algorithm>
#include <atomic>
#include <bitset>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include "rclcpp/rclcpp.hpp"

#include "canopen_pre_mapped_driver/mode_forward_helper.hpp"
#include "canopen_base_driver/diagnostic_collector.hpp"
#include "canopen_base_driver/lely_driver_bridge.hpp"

namespace ros2_canopen
{
class PreMappedMotor
{
public:
  PreMappedMotor(
    std::shared_ptr<LelyDriverBridge> driver, ros2_canopen::State402::InternalState switching_state,
    std::string joint_name, double scale_pos_to_dev, double scale_pos_from_dev,
    double scale_vel_to_dev,
    double scale_vel_from_dev, uint16_t default_operation_mode, uint8_t channel)
  : joint_name_(joint_name)
    , scale_pos_to_dev_(scale_pos_to_dev)
    , scale_pos_from_dev_(scale_pos_from_dev)
    , scale_vel_to_dev_(scale_vel_to_dev)
    , scale_vel_from_dev_(scale_vel_from_dev)
    , default_operation_mode_(default_operation_mode)
    , switching_state_(switching_state)
    , monitor_mode_(true)
    , state_switch_timeout_(1)
    , initialized_(false)
    , has_communication_failure_(false)
  {
    this->driver = driver;
  }

  virtual bool setTarget(double val);
  // virtual bool enterModeAndWait(uint16_t mode);
  // virtual bool isModeSupported(uint16_t mode);
  virtual uint16_t getMode();
  bool readState();

  const std::string & getJointName() const
  {
    return joint_name_;
  }

  uint16_t getDefaultOperationMode() const
  {
    return default_operation_mode_;
  }

  void setDriver(std::shared_ptr<LelyDriverBridge> driver)
  {
    this->driver = driver;
  }

  /**
   * @brief Updates the device diagnostic information
   *
   * This function updates the diagnostic information of the device by updating the diagnostic
   * status message
   * @ref diagnostic_status_ and publishing it.
   */
  void handleDiag();
  /**
   * @brief Initialise the drive
   *
   * This function initialises the drive. This means, it first
   * attempts to bring the device to operational state (CIA402)
   * and then executes the chosen homing method.
   *
   */
  bool handleInit();
  /**
   * @brief Read objects of the drive
   *
   * This function should be called regularly. It reads the status word
   * from the device and translates it into the devices state.
   *
   */
  void handleRead();
  /**
   * @brief Writes objects to the drive
   *
   * This function should be called regularly. It writes the new command
   * word to the drive
   *
   */
  void handleWrite();
  /**
   * @brief Shutdowns the drive
   *
   * This function shuts down the drive by bringing it into
   * SwitchOn disabled state.
   *
   */
  bool handleShutdown();
  /**
   * @brief Executes a quickstop
   *
   * The function executes a quickstop.
   *
   */
  bool handleHalt();

  /**
   * @brief Recovers the device from fault
   *
   * This function tries to reset faults and
   * put the device back to operational state.
   *
   */
  bool handleRecover();

  /**
   * @brief Checks if there is a Fault
   *
   */
  bool isFaulty();

  /**
   * @brief Checks if there has been a communication failure recently.
   *
   * Will return true if communication failed, and false if everything is ok
   *
   */
  bool hasCommunicationFailure();

  /**
   * @brief Returns whether the motor is initialized
   *
   */
  bool isInitialized();

  /**
   * @brief Returns if the motor is initialized
   *
   */
  bool isHalted();

  double get_speed()
  {
    if (speed_feedback_index != 0) {
      try {
        double value =
          this->driver->universal_get_value<int32_t>(
          speed_feedback_index,
          0) * scale_vel_from_dev_;
        has_communication_failure_ = false;
        return value;
      } catch (std::runtime_error & e) {
        // communication was unsuccessful
        has_communication_failure_ = true;
      }
    }
    return 0.0;
  }

  double get_position()
  {
    if (position_feedback_index != 0) {
      try {
        double value =
          this->driver->universal_get_value<int32_t>(
          position_feedback_index,
          0) * scale_pos_from_dev_;
        has_communication_failure_ = false;
        return value;
      } catch (std::runtime_error & e) {
        // communication was unsuccessful
        has_communication_failure_ = true;
      }
    }
    return 0.0;
  }

  void set_diagnostic_status_msgs(std::shared_ptr<DiagnosticsCollector> status, bool enable)
  {
    this->enable_diagnostics_.store(enable);
    this->diag_collector_ = status;
  }

private:
  std::string joint_name_;

  double scale_pos_to_dev_;
  double scale_pos_from_dev_;
  double scale_vel_to_dev_;
  double scale_vel_from_dev_;

  // default operation mode to set
  uint16_t default_operation_mode_;

  std::atomic<uint16_t> status_word_;
  uint16_t control_word_;
  std::atomic<bool> start_fault_reset_;
  std::atomic<State402::InternalState> target_state_;

  State402 state_handler_;

  ModeSharedPtr selected_mode_;
  uint16_t mode_id_;
  std::condition_variable mode_cond_;
  const State402::InternalState switching_state_;
  const bool monitor_mode_;
  const std::chrono::seconds state_switch_timeout_;

  //  !need to patch, that second channel is supported
  std::shared_ptr<LelyDriverBridge> driver;
  uint16_t status_word_entry_index = 0;
  uint16_t control_word_entry_index = 0;
  uint16_t op_mode_display_index = 0;
  uint16_t op_mode_index = 0;
  uint16_t supported_drive_modes_index = 0;
  uint16_t speed_feedback_index = 0;
  uint16_t position_feedback_index = 0;

  // whill switch to true when initialization procedure has finished
  bool initialized_ = false;
  bool has_communication_failure_ = false;

  // Diagnostic components
  std::atomic<bool> enable_diagnostics_;
  std::shared_ptr<DiagnosticsCollector> diag_collector_;
};

}  // namespace ros2_canopen

#endif
