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

// The watchdog reports through a diagnostics key, never by raising the summary level.
// Consumers of the aggregated motor diagnostics - lero_teleop's scanreco remote among them -
// treat *any* non-OK motor status as "system not ok" and drop the robot out of manual mode.
// A suspicion is not worth that, and this cannot tell a wheel driven against an obstacle
// apart from a dead drive. Pass true here only for a fault that is confirmed and permanent.
constexpr bool kRaiseDiagnosticError = false;
}  // namespace

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
                            uint16_t node_id, uint8_t channel, double command, double position, bool drive_ready,
                            const rclcpp::Time& now)
{
  if (!enabled_ || !driver)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(joints_mutex_);
  auto& joint = joints_[joint_name];

  // A steering axis is commanded a velocity permanently to hold its angle, and that holding
  // command is noisy. A single sample below the gate disarms, so only a demand sustained past
  // the gate for the whole window counts as "motion was expected here".
  if (std::abs(command) <= kCmdThreshold || !drive_ready)
  {
    joint.armed = false;
    return;
  }

  if (!joint.armed)
  {
    joint.armed = true;
    joint.armed_at = now;
    joint.armed_position = position;
    return;
  }

  // Any real movement proves the drive is receiving its setpoints. Restart the window.
  if (std::abs(position - joint.armed_position) > kPositionEpsilon)
  {
    joint.armed_at = now;
    joint.armed_position = position;
    if (joint.reported || joint.repair_attempts > 0)
    {
      RCLCPP_INFO(kLogger, "%s is moving again", joint_name.c_str());
      driver->set_motor_motion_status(channel, "", false);
      joint.reported = false;
      joint.repair_attempts = 0;
    }
    return;
  }

  if ((now - joint.armed_at).seconds() < kTimeout)
  {
    return;
  }

  if (joint.repair_pending || joint.repair_attempts >= kMaxAttempts)
  {
    return;
  }

  if (joint.repair_attempts > 0 && (now - joint.last_attempt_time).seconds() < kRetryPeriod)
  {
    return;  // cooling down between attempts
  }

  RCLCPP_WARN(kLogger, "%s commanded %.3f rad/s for %.1f s without moving - checking RPDO configuration",
              joint_name.c_str(), command, kTimeout);

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

    std::string status;
    if (repaired > 0)
    {
      status = "re-enabled " + std::to_string(repaired) + " disabled RPDO(s)";
      RCLCPP_WARN(kLogger, "%s - %s, setpoints should reach the drive again", req.joint_name.c_str(), status.c_str());
    }
    else if (repaired == 0)
    {
      // The drive is receiving its setpoints, so this is not the RPDO failure. Most likely a
      // blocked wheel or a drive-internal protection - reported, but not treated as a fault.
      status = "commanded without moving, RPDO config valid";
      RCLCPP_WARN(kLogger, "%s - %s (mechanical blockage or drive-level protection?)", req.joint_name.c_str(),
                  status.c_str());
    }
    else
    {
      status = "commanded without moving, RPDO config unreadable";
      RCLCPP_ERROR(kLogger, "%s - could not read the RPDO configuration over SDO", req.joint_name.c_str());
    }

    std::lock_guard<std::mutex> lock(joints_mutex_);
    auto& joint = joints_[req.joint_name];
    joint.repair_pending = false;
    joint.reported = true;
    req.driver->set_motor_motion_status(req.channel, status, kRaiseDiagnosticError);
  }
}

}  // namespace canopen_ros2_control
