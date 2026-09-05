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

#include "canopen_ros2_control/motion_watchdog.hpp"

#include <cmath>

namespace
{
auto const kLogger = rclcpp::get_logger("MotionWatchdog");
}

namespace canopen_ros2_control
{

MotionWatchdog::~MotionWatchdog()
{
  stop();
}

void MotionWatchdog::start()
{
  if (worker_thread_.joinable())
  {
    return;
  }
  shutdown_.store(false);
  worker_thread_ = std::thread(&MotionWatchdog::worker, this);
}

void MotionWatchdog::stop()
{
  shutdown_.store(true);
  queue_cv_.notify_all();
  if (worker_thread_.joinable())
  {
    worker_thread_.join();
  }
}

void MotionWatchdog::update(const std::shared_ptr<ros2_canopen::Cia402Driver>& driver, const std::string& joint_name,
                            uint16_t node_id, uint8_t channel, double command, double speed, bool drive_ready,
                            const rclcpp::Time& now)
{
  if (!enabled_ || !driver)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(joints_mutex_);
  auto& joint = joints_[joint_name];

  const bool motion_demanded = std::abs(command) > kCmdThreshold;
  const bool moving = std::abs(speed) > kSpeedThreshold;

  if (!motion_demanded || !drive_ready || moving)
  {
    // Clear the trip once the joint moved or the demand went away. A drive that merely
    // dropped out of Operation enabled keeps its history, so the retry budget is not
    // silently reset while it cycles through fault recovery.
    if (moving || !motion_demanded)
    {
      if (joint.stalled)
      {
        RCLCPP_INFO(kLogger, "%s is moving again after %d repair attempt(s)", joint_name.c_str(),
                    joint.repair_attempts);
        driver->set_motor_motion_fault(channel, false, "");
      }
      joint.stalled = false;
      joint.latched = false;
      joint.repair_attempts = 0;
    }
    joint.armed = false;
    return;
  }

  // From here on: the joint is asked to move, the drive reports it can, nothing happens.
  if (!joint.armed)
  {
    joint.armed = true;
    joint.nonzero_cmd_since = now;
    return;
  }

  if ((now - joint.nonzero_cmd_since).seconds() < kTimeout)
  {
    return;
  }

  if (!joint.stalled)
  {
    joint.stalled = true;
    RCLCPP_ERROR(kLogger, "%s commanded %.4f for %.2f s but is not moving", joint_name.c_str(), command, kTimeout);
    driver->set_motor_motion_fault(channel, true, "commanded but not moving, checking RPDO configuration");
  }

  if (joint.latched || joint.repair_pending)
  {
    return;
  }

  if (joint.repair_attempts > 0 && (now - joint.last_attempt_time).seconds() < kRetryPeriod)
  {
    return;  // cooling down between attempts
  }

  if (joint.repair_attempts >= kMaxAttempts)
  {
    joint.latched = true;
    RCLCPP_ERROR(kLogger,
                 "%s still not moving after %d repair attempts - giving up. The RPDO configuration is valid, so "
                 "this is a mechanical blockage or a drive-internal protection.",
                 joint_name.c_str(), joint.repair_attempts);
    driver->set_motor_motion_fault(channel, true, "not moving, RPDO config valid - mechanical or drive-level stall");
    return;
  }

  joint.repair_attempts++;
  joint.last_attempt_time = now;
  joint.repair_pending = true;

  {
    std::lock_guard<std::mutex> queue_lock(queue_mutex_);
    queue_.push_back({ driver, node_id, channel, joint_name });
  }
  queue_cv_.notify_one();
}

int MotionWatchdog::repairRpdoCobIds(const std::shared_ptr<ros2_canopen::Cia402Driver>& driver, uint16_t node_id,
                                     const std::string& joint_name)
{
  int repaired = 0;
  bool any_read_ok = false;

  // RPDO communication parameters live at 0x1400+n, sub-index 1 holds the COB-ID. Bit 31 set
  // means "PDO does not exist / is invalid": the drive silently discards every frame on that
  // COB-ID, so the setpoint never arrives and 0x60FF stays at zero.
  for (uint8_t n = 0; n < kNumRpdos; ++n)
  {
    const uint16_t index = static_cast<uint16_t>(0x1400 + n);

    ros2_canopen::COData data;
    data.index_ = index;
    data.subindex_ = 1;
    data.data_ = 0;

    if (!driver->sdo_read(data))
    {
      continue;  // RPDO not implemented on this device, or the transfer failed
    }
    any_read_ok = true;

    const uint32_t cobid = data.data_;
    if ((cobid & 0x80000000u) == 0)
    {
      continue;  // already valid
    }

    ros2_canopen::COData fix;
    fix.index_ = index;
    fix.subindex_ = 1;
    fix.data_ = cobid & ~0x80000000u;

    if (driver->sdo_write(fix))
    {
      repaired++;
      RCLCPP_WARN(kLogger, "node %u RPDO%u (%s) was disabled, COB-ID 0x%08X -> 0x%08X", node_id,
                  static_cast<unsigned>(n + 1), joint_name.c_str(), cobid, fix.data_);
    }
    else
    {
      RCLCPP_ERROR(kLogger, "node %u RPDO%u (%s) is disabled (0x%08X) and could not be repaired", node_id,
                   static_cast<unsigned>(n + 1), joint_name.c_str(), cobid);
    }
  }

  return any_read_ok ? repaired : -1;
}

void MotionWatchdog::worker()
{
  while (true)
  {
    RepairRequest req;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return shutdown_.load() || !queue_.empty(); });
      if (shutdown_.load())
      {
        return;
      }
      req = queue_.front();
      queue_.pop_front();
    }

    const int repaired = repairRpdoCobIds(req.driver, req.node_id, req.joint_name);

    std::string detail;
    if (repaired > 0)
    {
      detail = "re-enabled " + std::to_string(repaired) + " disabled RPDO(s)";
      RCLCPP_WARN(kLogger, "%s - %s, setpoints should reach the drive again", req.joint_name.c_str(), detail.c_str());
    }
    else if (repaired == 0)
    {
      detail = "RPDO configuration valid, drive is receiving setpoints";
    }
    else
    {
      detail = "could not read RPDO configuration over SDO";
      RCLCPP_ERROR(kLogger, "%s - %s", req.joint_name.c_str(), detail.c_str());
    }

    std::lock_guard<std::mutex> lock(joints_mutex_);
    auto& joint = joints_[req.joint_name];
    joint.repair_pending = false;
    if (joint.stalled)
    {
      req.driver->set_motor_motion_fault(req.channel, true, "not moving: " + detail);
    }
  }
}

}  // namespace canopen_ros2_control
