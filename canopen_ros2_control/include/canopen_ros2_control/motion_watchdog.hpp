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

#ifndef CANOPEN_ROS2_CONTROL__MOTION_WATCHDOG_HPP_
#define CANOPEN_ROS2_CONTROL__MOTION_WATCHDOG_HPP_

#include "canopen_402_driver/cia402_driver.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace canopen_ros2_control
{

/**
 * @brief Detects joints that are commanded but do not move, and repairs the cause.
 *
 * A CiA402 drive can report "Operation enabled" and "drive follows command value" while
 * standing perfectly still: when the RPDO carrying the setpoint (0x60FF) has the invalid
 * bit (bit 31) set in its COB-ID, the drive discards every setpoint frame, so the command
 * it actually holds really is zero. Nothing in the CiA402 state machine reveals this, and
 * the joint stays dead until someone notices.
 *
 * This class compares the commands issued by the hardware interface against the reported
 * speed. When a joint is asked to move and does not, it re-reads the drive's RPDO COB-IDs
 * over SDO and clears the invalid bit. An NMT reset does not help here: the master's
 * boot-up configuration download is what failed to apply in the first place.
 *
 * SDO transfers block, so the repair runs on its own worker thread. update() only advances
 * timers and queues work, and is safe to call from the control loop.
 */
class MotionWatchdog
{
public:
  MotionWatchdog() = default;
  ~MotionWatchdog();

  void setEnabled(bool enabled)
  {
    enabled_ = enabled;
  }
  bool isEnabled() const
  {
    return enabled_;
  }

  /// Starts the repair worker thread. Safe to call more than once.
  void start();

  /// Stops and joins the repair worker thread. Safe to call more than once.
  void stop();

  /**
   * @brief Feed one channel's command and feedback. Non-blocking.
   *
   * @param command velocity setpoint being issued, in joint units. Pass 0 when the active
   *                mode is not a velocity mode, which disables the check for that channel.
   * @param speed speed reported by the drive, in joint units
   * @param drive_ready whether the drive reports that it can act on the setpoint
   */
  void update(const std::shared_ptr<ros2_canopen::Cia402Driver>& driver, const std::string& joint_name,
              uint16_t node_id, uint8_t channel, double command, double speed, bool drive_ready,
              const rclcpp::Time& now);

private:
  struct JointState
  {
    bool armed = false;  // a motion-demanding command is currently pending
    rclcpp::Time nonzero_cmd_since;
    bool stalled = false;         // watchdog has tripped
    bool latched = false;         // repair attempts exhausted, stop retrying
    bool repair_pending = false;  // queued or in flight, do not queue twice
    int repair_attempts = 0;
    rclcpp::Time last_attempt_time;
  };

  struct RepairRequest
  {
    std::shared_ptr<ros2_canopen::Cia402Driver> driver;
    uint16_t node_id;
    uint8_t channel;
    std::string joint_name;
  };

  void worker();

  /// Clears the invalid bit on every disabled RPDO of this node. Returns the number
  /// repaired, or -1 if the drive could not be queried over SDO.
  int repairRpdoCobIds(const std::shared_ptr<ros2_canopen::Cia402Driver>& driver, uint16_t node_id,
                       const std::string& joint_name);

  static constexpr double kTimeout = 0.5;         // s of unsatisfied command before tripping
  static constexpr double kCmdThreshold = 0.05;   // rad/s, command magnitude demanding motion
  static constexpr double kSpeedThreshold = 0.02; // rad/s, below this counts as "not moving"
  static constexpr double kRetryPeriod = 2.0;     // s between repair attempts
  static constexpr int kMaxAttempts = 3;
  static constexpr uint8_t kNumRpdos = 4;  // bus.yml configures 4 RPDOs per node

  bool enabled_ = true;

  std::map<std::string, JointState> joints_;
  std::mutex joints_mutex_;

  std::thread worker_thread_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<RepairRequest> queue_;
  std::atomic<bool> shutdown_{ false };
};

}  // namespace canopen_ros2_control

#endif  // CANOPEN_ROS2_CONTROL__MOTION_WATCHDOG_HPP_
