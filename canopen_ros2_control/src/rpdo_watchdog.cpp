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

#include "canopen_ros2_control/rpdo_watchdog.hpp"

#include <chrono>

namespace
{
auto const kLogger = rclcpp::get_logger("RpdoWatchdog");
}  // namespace

namespace canopen_ros2_control
{

RpdoWatchdog::~RpdoWatchdog()
{
  stop();
}

void RpdoWatchdog::registerDriver(const std::shared_ptr<ros2_canopen::Cia402Driver>& driver, uint16_t node_id,
                                   const std::vector<uint8_t>& channels, const std::vector<std::string>& joint_names)
{
  std::lock_guard<std::mutex> lock(nodes_mutex_);
  nodes_.push_back({ driver, node_id, channels, joint_names, false });
}

void RpdoWatchdog::start()
{
  if (worker_thread_.joinable())
  {
    return;
  }
  shutdown_.store(false);
  worker_thread_ = std::thread(&RpdoWatchdog::worker, this);
}

void RpdoWatchdog::stop()
{
  shutdown_.store(true);
  if (worker_thread_.joinable())
  {
    worker_thread_.join();
  }
}

int RpdoWatchdog::checkAndRepairNode(RegisteredNode& node)
{
  int repaired = 0;
  int disabled_count = 0;

  // RPDO communication parameters live at 0x1400+n, sub-index 1 holds the COB-ID.
  // Bit 31 set means "PDO does not exist / is invalid": the drive silently discards
  // every frame on that COB-ID, so the setpoint never arrives.
  for (uint8_t n = 0; n < kNumRpdos; ++n)
  {
    if (shutdown_.load())
    {
      return repaired;
    }

    const uint16_t index = static_cast<uint16_t>(0x1400 + n);

    ros2_canopen::COData data;
    data.index_ = index;
    data.subindex_ = 1;
    data.data_ = 0;

    if (!node.driver->sdo_read(data))
    {
      continue;  // RPDO not implemented or SDO failed
    }

    const uint32_t cobid = data.data_;
    if ((cobid & 0x80000000u) == 0)
    {
      continue;  // already valid
    }

    disabled_count++;

    // Repair: clear bit 31
    ros2_canopen::COData fix;
    fix.index_ = index;
    fix.subindex_ = 1;
    fix.data_ = cobid & ~0x80000000u;

    if (node.driver->sdo_write(fix))
    {
      repaired++;
      RCLCPP_WARN(kLogger, "node %u RPDO%u was disabled (0x%08X), re-enabled (0x%08X)",
                  node.node_id, static_cast<unsigned>(n + 1), cobid, fix.data_);
    }
    else
    {
      RCLCPP_ERROR(kLogger, "node %u RPDO%u is disabled (0x%08X) and could not be repaired",
                   node.node_id, static_cast<unsigned>(n + 1), cobid);
    }

    // Small delay between SDO operations to avoid bus congestion
    std::this_thread::sleep_for(std::chrono::milliseconds(kPollIntervalMs));
  }

  // Update diagnostic status on all motors of this node
  if (disabled_count > 0 && !node.had_fault)
  {
    std::string status = "repaired " + std::to_string(repaired) + "/" +
                         std::to_string(disabled_count) + " disabled RPDO(s)";
    for (size_t i = 0; i < node.channels.size(); ++i)
    {
      node.driver->set_motor_motion_status(node.channels[i], status, false);
    }
    if (repaired == disabled_count)
    {
      RCLCPP_INFO(kLogger, "node %u: all disabled RPDOs repaired", node.node_id);
    }
    node.had_fault = true;
  }
  else if (disabled_count == 0 && node.had_fault)
  {
    // Clear diagnostic status - all RPDOs now valid
    for (size_t i = 0; i < node.channels.size(); ++i)
    {
      node.driver->set_motor_motion_status(node.channels[i], "", false);
    }
    RCLCPP_INFO(kLogger, "node %u: RPDO config now valid", node.node_id);
    node.had_fault = false;
  }

  return repaired;
}

void RpdoWatchdog::worker()
{
  RCLCPP_INFO(kLogger, "RPDO watchdog started, monitoring %zu node(s)", nodes_.size());

  while (!shutdown_.load())
  {
    if (!enabled_)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(kCyclePauseMs));
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(nodes_mutex_);
      for (auto& node : nodes_)
      {
        if (shutdown_.load())
        {
          return;
        }
        checkAndRepairNode(node);
      }
    }

    // Pause between full cycles to avoid constant SDO traffic
    std::this_thread::sleep_for(std::chrono::milliseconds(kCyclePauseMs));
  }

  RCLCPP_INFO(kLogger, "RPDO watchdog stopped");
}

}  // namespace canopen_ros2_control
