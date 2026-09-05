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
 * @brief Detects joints that are commanded hard but do not move, and repairs the cause.
 *
 * A CiA402 drive can report "Operation enabled" and "drive follows command value" while
 * standing perfectly still: when the RPDO carrying the setpoint (0x60FF) has the invalid
 * bit (bit 31) set in its COB-ID, the drive discards every setpoint frame, so the command
 * it actually holds really is zero. Nothing in the CiA402 state machine reveals this, and
 * the joint stays dead until somebody notices.
 *
 * Detection deliberately keys on two things at once, because "non-zero command and no
 * motion" on its own is a perfectly normal steady state - a steering axis closes a position
 * loop, so it is commanded a velocity permanently, and holds still against friction:
 *
 *  - the command must be *large*: the observed failure saturated the controller near
 *    2.2 rad/s because the position error never closed, while holding an angle produces
 *    well under 0.05 rad/s;
 *  - the *encoder* must be frozen. Position is ground truth here. The drive's velocity
 *    feedback is quantised (about 0.0016 rad/s per count on the steering axes) and reads
 *    zero for slow but real movement, whereas the broken joint did not advance by a single
 *    count in 20 s.
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
   * @param position position reported by the drive, in joint units
   * @param drive_ready whether the drive reports that it can act on the setpoint
   */
  void update(const std::shared_ptr<ros2_canopen::Cia402Driver>& driver, const std::string& joint_name,
              uint16_t node_id, uint8_t channel, double command, double position, bool drive_ready,
              const rclcpp::Time& now);

private:
  struct JointState
  {
    bool armed = false;  // a large command is currently pending
    rclcpp::Time armed_at;
    double armed_position = 0.0;
    bool repair_pending = false;  // queued or in flight, do not queue twice
    bool reported = false;        // a status has been pushed to the diagnostics
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

  // Measured on lero03 sn01, robot stationary: idle/holding commands peak at 0.51 rad/s on the
  // steering axes, but they oscillate, and a single sample below the gate disarms the check.
  // The longest *continuous* stretch above 0.15 rad/s was 1.49 s, half the window below - while
  // a drive wheel rolling at 0.1 m/s holds a steady 0.32 rad/s (0.1 / 0.317 m wheel radius),
  // twice the gate. So this separates real demand from holding jitter, and still catches a dead
  // drive wheel down to about 0.05 m/s. Re-measure before changing it: the steering axes idle
  // *above* the level a slowly rolling wheel commands, so the margin comes from duration, not
  // from amplitude.
  static constexpr double kCmdThreshold = 0.15;     // rad/s, a command that must produce motion
  static constexpr double kPositionEpsilon = 0.02;  // rad, movement that counts as "it is alive"
  static constexpr double kTimeout = 3.0;           // s frozen under a sustained command
  static constexpr double kRetryPeriod = 5.0;       // s between repair attempts
  static constexpr int kMaxAttempts = 2;
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
