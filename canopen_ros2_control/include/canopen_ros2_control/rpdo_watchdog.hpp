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

#ifndef CANOPEN_ROS2_CONTROL__RPDO_WATCHDOG_HPP_
#define CANOPEN_ROS2_CONTROL__RPDO_WATCHDOG_HPP_

#include "canopen_402_driver/cia402_driver.hpp"
#include "rclcpp/rclcpp.hpp"

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace canopen_ros2_control
{

/**
 * @brief Continuously monitors RPDO configuration and repairs disabled RPDOs.
 *
 * CiA402 drives can silently drop setpoints when an RPDO's COB-ID has bit 31 set (invalid).
 * The drive reports "Operation enabled" but ignores every setpoint frame. This watchdog
 * periodically reads RPDO COB-IDs via SDO and clears bit 31 if found set.
 *
 * Unlike motion-based detection, this catches the fault directly at the source, before
 * any symptom appears, and works even when wheels are being dragged by external forces.
 */
class RpdoWatchdog
{
public:
  RpdoWatchdog() = default;
  ~RpdoWatchdog();

  void setEnabled(bool enabled)
  {
    enabled_ = enabled;
  }
  bool isEnabled() const
  {
    return enabled_;
  }

  /**
   * @brief Register a driver to be monitored. Call before start().
   *
   * @param driver The CiA402 driver
   * @param node_id CANopen node ID
   * @param channels Motor channels on this node (for diagnostic reporting)
   * @param joint_names Joint names per channel (for logging)
   */
  void registerDriver(const std::shared_ptr<ros2_canopen::Cia402Driver>& driver, uint16_t node_id,
                      const std::vector<uint8_t>& channels, const std::vector<std::string>& joint_names);

  /// Starts the monitoring thread. Safe to call more than once.
  void start();

  /// Stops and joins the monitoring thread. Safe to call more than once.
  void stop();

private:
  struct RegisteredNode
  {
    std::shared_ptr<ros2_canopen::Cia402Driver> driver;
    uint16_t node_id;
    std::vector<uint8_t> channels;
    std::vector<std::string> joint_names;
    bool had_fault = false;  // tracks if we've reported a fault for this node
  };

  void worker();

  /// Checks and repairs RPDO COB-IDs for a node. Returns number of RPDOs repaired.
  int checkAndRepairNode(RegisteredNode& node);

  static constexpr uint8_t kNumRpdos = 4;          // bus.yml configures 4 RPDOs per node
  static constexpr int kPollIntervalMs = 100;      // ms between SDO reads (spread load)
  static constexpr int kCyclePauseMs = 1000;       // ms pause after checking all nodes

  bool enabled_ = true;

  std::vector<RegisteredNode> nodes_;
  std::mutex nodes_mutex_;

  std::thread worker_thread_;
  std::atomic<bool> shutdown_{ false };
};

}  // namespace canopen_ros2_control

#endif  // CANOPEN_ROS2_CONTROL__RPDO_WATCHDOG_HPP_
