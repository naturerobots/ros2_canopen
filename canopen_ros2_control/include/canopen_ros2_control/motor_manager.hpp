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

#ifndef CANOPEN_ROS2_CONTROL__MOTOR_MANAGER_HPP_
#define CANOPEN_ROS2_CONTROL__MOTOR_MANAGER_HPP_

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "canopen_402_driver/cia402_driver.hpp"
#include "canopen_core/device_container.hpp"

namespace canopen_ros2_control
{

/// Motor state, derived from PDO-cached data only - evaluating it costs no bus traffic.
enum class MotorHealth
{
  Operational,
  CommunicationFailure,  //!< Node stopped answering.
  Uninitialized,         //!< Init sequence has not completed.
  Faulty,                //!< Reachable but not in Operation_Enable (e-stop, over-current, ...).
  WrongMode,             //!< Enabled, but the operation mode did not take.
  Homing,                //!< Motor is executing homing procedure, do not interfere.
};

const char* to_string(MotorHealth health);

/// One motor channel, resolved once when the manager is configured.
struct ManagedMotor
{
  uint16_t node_id = 0;
  uint8_t channel = 0;
  std::string joint_name;
  std::shared_ptr<ros2_canopen::Cia402Driver> driver;

  MotorHealth health = MotorHealth::Uninitialized;
  //!< Set when the full init sequence must run again, e.g. after an NMT reset.
  bool needs_init = true;
  uint64_t consecutive_failures = 0;

  // Kept so a recurring problem can be reconstructed from the log without per-attempt output.
  uint64_t total_failures = 0;   //!< Failed attempts since startup.
  uint64_t episodes = 0;         //!< Times this motor left the operational state.
  std::chrono::steady_clock::time_point episode_start{};
  std::chrono::steady_clock::time_point last_transition_log{};
};

/// State shared by all channels of one CANopen node.
struct ManagedNode
{
  //!< A drive with a disabled or mis-addressed PDO reports itself enabled but never moves.
  bool pdo_check_needed = true;
  std::chrono::steady_clock::time_point last_pdo_check{};
  std::chrono::steady_clock::time_point last_nmt_reset{};
};

/**
 * Owns initialization, fault recovery, operation mode switching, PDO repair and NMT resets on
 * one dedicated thread, free to block on CAN round trips without stalling the update loop.
 *
 * The real-time side only needs is_operational(); while it reads false, every motor must stop.
 *
 * Motors are serviced strictly one after another, and nothing here gives up - an e-stop may
 * stay engaged for a long time - so there is no retry ceiling in this class.
 */
class MotorManager
{
public:
  MotorManager();
  ~MotorManager();

  MotorManager(const MotorManager&) = delete;
  MotorManager& operator=(const MotorManager&) = delete;

  /// Builds the flat motor list from the registered drivers. Generates no bus traffic.
  void configure(const std::map<uint16_t, std::shared_ptr<ros2_canopen::CanopenDriverInterface>>& drivers);

  /// Starts the manager thread. Returns immediately, even with the e-stop engaged.
  void start();

  /// Stops the manager thread and waits for it to finish its current step.
  void stop();

  /// True only while every motor is enabled, in a usable mode and communicating.
  /// Lock-free: safe to call from the write() thread every cycle.
  bool is_operational() const { return operational_.load(std::memory_order_acquire); }

  /// Joint names in service order, for logging and diagnostics.
  std::vector<std::string> joint_names() const;

  /// Reports a node rebooted from outside (e.g. NMT reset via the command interface), so PDOs
  /// are re-verified and init re-run. Lock-free: safe to call from write().
  void notify_external_reset() { external_reset_pending_.store(true, std::memory_order_release); }

private:
  void run();

  /// Refreshes every motor's health. Cheap: cached PDO data only, no blocking calls.
  /// Returns true when all motors are operational.
  bool updateHealth();

  MotorHealth checkHealth(const ManagedMotor& motor) const;

  /// Performs at most one repair step for one motor. May block on CAN round trips.
  void serviceMotor(ManagedMotor& motor);

  /// Verifies and repairs the node's PDO COB-IDs. Returns false if the node must be retried.
  bool ensurePdoConfig(ManagedNode& node, const ManagedMotor& motor);

  /// Reboots the node and marks all of its motors for a full re-initialization.
  void resetNode(ManagedNode& node, const ManagedMotor& motor, const char* reason);

  /// Rewrites RPDO/TPDO COB-IDs that are disabled or mis-addressed.
  /// Returns the number repaired, or a negative count if any repair failed.
  int repairPdoConfig(const std::shared_ptr<ros2_canopen::Cia402Driver>& driver, uint16_t node_id);

  /// Consumes a pending external reset by re-running initialization for every node.
  void applyExternalReset();

  /// "faulty (2): front_left_wheel (x12), rear_left_wheel (x3)" for the throttled summary.
  std::string describeUnhealthy() const;

  /// Logs one line per health change, throttled once a motor starts flapping.
  void logTransition(ManagedMotor& motor, MotorHealth next, std::chrono::steady_clock::time_point now);

  std::vector<ManagedMotor> motors_;
  std::map<uint16_t, ManagedNode> nodes_;

  std::atomic<bool> operational_{ false };
  std::atomic<bool> running_{ false };
  std::atomic<bool> external_reset_pending_{ false };
  std::unique_ptr<std::thread> thread_;

  //!< Episodes per motor logged in full before transition logging is throttled.
  static constexpr uint64_t kVerboseEpisodes = 3;
  //!< Throttle for transition logging once a motor flaps, and for the unhealthy digest.
  static constexpr std::chrono::seconds kTransitionLogInterval{ 30 };
  static constexpr std::chrono::seconds kDigestInterval{ 10 };
  //!< Health sampling while healthy; matched to the SYNC period.
  static constexpr std::chrono::milliseconds kHealthPollPeriod{ 20 };
  //!< Pause between repair passes, so a long e-stop does not spin the CPU.
  static constexpr std::chrono::milliseconds kRepairPassPeriod{ 100 };
  //!< Rate limits only, never retry limits.
  static constexpr std::chrono::seconds kNmtResetCooldown{ 10 };
  static constexpr std::chrono::seconds kPdoRecheckInterval{ 10 };
  static constexpr uint64_t kPdoRecheckFailureThreshold = 5;
  //!< Failed mode switches before the motor is re-initialized, which re-runs the mode allocators.
  static constexpr uint64_t kModeRetriesBeforeReinit = 10;
};

}  // namespace canopen_ros2_control

#endif  // CANOPEN_ROS2_CONTROL__MOTOR_MANAGER_HPP_
