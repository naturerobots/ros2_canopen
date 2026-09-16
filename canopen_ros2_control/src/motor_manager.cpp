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

#include "canopen_ros2_control/motor_manager.hpp"

#include <cstdio>

namespace
{
auto const kLogger = rclcpp::get_logger("Cia402MotorManager");
/// Paces throttled log messages from the manager thread.
rclcpp::Clock kLogClock(RCL_STEADY_TIME);
}

namespace canopen_ros2_control
{

const char* to_string(MotorHealth health)
{
  switch (health)
  {
    case MotorHealth::Operational:
      return "operational";
    case MotorHealth::CommunicationFailure:
      return "communication failure";
    case MotorHealth::Uninitialized:
      return "uninitialized";
    case MotorHealth::Faulty:
      return "faulty";
    case MotorHealth::WrongMode:
      return "wrong operation mode";
  }
  return "unknown";
}

MotorManager::MotorManager() = default;

MotorManager::~MotorManager()
{
  stop();
}

void MotorManager::configure(const std::map<uint16_t, std::shared_ptr<ros2_canopen::CanopenDriverInterface>>& drivers)
{
  motors_.clear();
  nodes_.clear();

  for (const auto& entry : drivers)
  {
    const uint16_t node_id = entry.first;
    auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(entry.second);

    nodes_.emplace(node_id, ManagedNode{});

    for (const auto channel : driver->get_available_motor_channels())
    {
      ManagedMotor motor;
      motor.node_id = node_id;
      motor.channel = channel;
      motor.joint_name = driver->get_motor_joint_name(channel);
      motor.driver = driver;
      motors_.push_back(std::move(motor));
    }
  }

  std::string list;
  for (const auto& motor : motors_)
  {
    char id[16];
    std::snprintf(id, sizeof(id), " (0x%X.%u)", motor.node_id, static_cast<unsigned>(motor.channel));
    list += (list.empty() ? "" : ", ") + motor.joint_name + id;
  }
  RCLCPP_INFO(kLogger, "Managing %zu motor(s) on %zu node(s): %s", motors_.size(), nodes_.size(), list.c_str());
}

std::vector<std::string> MotorManager::joint_names() const
{
  std::vector<std::string> names;
  names.reserve(motors_.size());
  for (const auto& motor : motors_)
  {
    names.push_back(motor.joint_name);
  }
  return names;
}

void MotorManager::start()
{
  if (running_.load(std::memory_order_acquire))
  {
    return;
  }
  // Arm the cooldown now, so a transient startup hiccup cannot reboot nodes immediately.
  const auto now = std::chrono::steady_clock::now();
  for (auto& node : nodes_)
  {
    node.second.last_nmt_reset = now;
  }

  operational_.store(false, std::memory_order_release);
  running_.store(true, std::memory_order_release);
  thread_ = std::make_unique<std::thread>(&MotorManager::run, this);
}

void MotorManager::stop()
{
  if (!running_.exchange(false, std::memory_order_acq_rel))
  {
    return;
  }
  if (thread_ && thread_->joinable())
  {
    // May block for up to one state switch timeout if a repair step is in flight.
    thread_->join();
  }
  thread_.reset();
  operational_.store(false, std::memory_order_release);
}

void MotorManager::run()
{
  RCLCPP_INFO(kLogger, "Motor manager thread started");

  while (running_.load(std::memory_order_acquire))
  {
    if (external_reset_pending_.exchange(false, std::memory_order_acq_rel))
    {
      applyExternalReset();
    }

    // Publish health before any repair work, so a motor dropping out stops the robot within
    // one poll period instead of at the end of a pass.
    const bool all_operational = updateHealth();
    operational_.store(all_operational, std::memory_order_release);

    if (all_operational)
    {
      std::this_thread::sleep_for(kHealthPollPeriod);
      continue;
    }

    // One throttled summary while anything is wrong, instead of per-motor chatter. Individual
    // changes are already logged once each by updateHealth().
    RCLCPP_WARN_THROTTLE(kLogger, kLogClock, std::chrono::milliseconds(kDigestInterval).count(), "Waiting for %s",
                         describeUnhealthy().c_str());

    // One motor after another, never several at once. Health is re-published after each so
    // write() does not wait for the whole pass.
    for (auto& motor : motors_)
    {
      if (!running_.load(std::memory_order_acquire))
      {
        break;
      }
      serviceMotor(motor);
      operational_.store(updateHealth(), std::memory_order_release);
    }

    std::this_thread::sleep_for(kRepairPassPeriod);
  }

  operational_.store(false, std::memory_order_release);
  RCLCPP_INFO(kLogger, "Motor manager thread stopped");
}

std::string MotorManager::describeUnhealthy() const
{
  std::map<MotorHealth, std::vector<std::string>> by_health;
  for (const auto& motor : motors_)
  {
    if (motor.health != MotorHealth::Operational)
    {
      by_health[motor.health].push_back(motor.total_failures > 0 ?
                                            motor.joint_name + " (x" + std::to_string(motor.total_failures) + ")" :
                                            motor.joint_name);
    }
  }

  std::string out;
  for (const auto& entry : by_health)
  {
    out += (out.empty() ? "" : "; ");
    out += std::string(to_string(entry.first)) + " (" + std::to_string(entry.second.size()) + "): ";
    for (size_t i = 0; i < entry.second.size(); ++i)
    {
      out += (i ? ", " : "") + entry.second[i];
    }
  }
  return out;
}

void MotorManager::applyExternalReset()
{
  RCLCPP_WARN(kLogger, "External NMT reset requested, re-initializing all nodes");

  for (auto& node : nodes_)
  {
    node.second.pdo_check_needed = true;
  }
  for (auto& motor : motors_)
  {
    motor.needs_init = true;
    motor.consecutive_failures = 0;
  }
}

bool MotorManager::updateHealth()
{
  // Would otherwise trivially report "all operational" and let write() command nothing.
  if (motors_.empty())
  {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  bool all_operational = true;

  for (auto& motor : motors_)
  {
    const MotorHealth health = checkHealth(motor);

    if (health != motor.health)
    {
      if (motor.health == MotorHealth::Operational)
      {
        motor.episodes++;
        motor.episode_start = now;
      }
      logTransition(motor, health, now);
      motor.health = health;
    }

    if (health != MotorHealth::Operational)
    {
      all_operational = false;
    }
  }

  return all_operational;
}

void MotorManager::logTransition(ManagedMotor& motor, MotorHealth next, std::chrono::steady_clock::time_point now)
{
  // The first few episodes of each motor are logged in full. After that a flapping motor is
  // throttled, but every line carries the running totals so the history stays reconstructable.
  if (motor.episodes > kVerboseEpisodes && now - motor.last_transition_log < kTransitionLogInterval)
  {
    return;
  }
  motor.last_transition_log = now;

  if (next == MotorHealth::Operational)
  {
    const double seconds = std::chrono::duration<double>(now - motor.episode_start).count();
    RCLCPP_INFO(kLogger, "%s: %s -> operational after %.1fs, %lu attempt(s) [episode %lu, %lu failures total]",
                motor.joint_name.c_str(), to_string(motor.health), seconds, motor.consecutive_failures,
                motor.episodes, motor.total_failures);
    return;
  }

  // Carry the drive's own reason, otherwise a recurring fault is only diagnosable at debug level.
  const std::string why = motor.driver->get_motor_last_error(motor.channel);
  RCLCPP_WARN(kLogger, "%s: %s -> %s [episode %lu, %lu failures total]%s%s", motor.joint_name.c_str(),
              to_string(motor.health), to_string(next), motor.episodes, motor.total_failures,
              why.empty() ? "" : " - ", why.c_str());
}

MotorHealth MotorManager::checkHealth(const ManagedMotor& motor) const
{
  // Order matters: a silent node says nothing useful about its CiA402 state, and a motor that
  // never ran init has no meaningful mode.
  if (motor.driver->has_motor_communication_failure(motor.channel))
  {
    return MotorHealth::CommunicationFailure;
  }
  if (motor.needs_init || !motor.driver->is_motor_initialized(motor.channel))
  {
    return MotorHealth::Uninitialized;
  }
  if (motor.driver->is_motor_faulty(motor.channel))
  {
    return MotorHealth::Faulty;
  }
  if (motor.driver->get_mode(motor.channel) == ros2_canopen::MotorBase::No_Mode)
  {
    return MotorHealth::WrongMode;
  }
  return MotorHealth::Operational;
}

void MotorManager::serviceMotor(ManagedMotor& motor)
{
  auto& node = nodes_[motor.node_id];

  switch (motor.health)
  {
    case MotorHealth::Operational:
      motor.consecutive_failures = 0;
      return;

    case MotorHealth::CommunicationFailure:
    {
      // Not on the bus. Nothing local can fix it, so kick it now and then and keep waiting.
      if (std::chrono::steady_clock::now() - node.last_nmt_reset >= kNmtResetCooldown)
      {
        resetNode(node, motor, "node is not communicating");
      }
      return;
    }

    case MotorHealth::Uninitialized:
    {
      // A drive with broken PDOs initializes cleanly and then never moves, so re-check after a
      // boot or NMT reset, and again when init keeps failing.
      const auto now = std::chrono::steady_clock::now();
      if (motor.consecutive_failures >= kPdoRecheckFailureThreshold && now - node.last_pdo_check >= kPdoRecheckInterval)
      {
        node.pdo_check_needed = true;
      }
      if (node.pdo_check_needed && !ensurePdoConfig(node, motor))
      {
        motor.consecutive_failures++;
        return;
      }

      if (!motor.driver->init_motor(motor.channel))
      {
        motor.consecutive_failures++;
        motor.total_failures++;
        return;
      }

      // Init leaves the drive in No_Mode; the mode is applied separately and retried via
      // the WrongMode branch if it does not take.
      motor.needs_init = false;
      motor.consecutive_failures = 0;

      motor.driver->set_default_operation_mode(motor.channel);
      return;
    }

    case MotorHealth::Faulty:
    {
      // E-stop, over-current, unpowered drive. The node answers, it just refuses to enable, so
      // an NMT reset would achieve nothing. Retried forever.
      // A drive that rebooted underneath us still reports a sane status word while its PDOs are
      // broken, so re-check those too - a few SDO reads, rate limited.
      const auto now = std::chrono::steady_clock::now();
      if (motor.consecutive_failures >= kPdoRecheckFailureThreshold && now - node.last_pdo_check >= kPdoRecheckInterval)
      {
        ensurePdoConfig(node, motor);
      }

      if (!motor.driver->recover_motor(motor.channel))
      {
        motor.consecutive_failures++;
        motor.total_failures++;
        return;
      }
      motor.consecutive_failures = 0;
      return;
    }

    case MotorHealth::WrongMode:
    {
      if (!motor.driver->set_default_operation_mode(motor.channel))
      {
        motor.consecutive_failures++;
        motor.total_failures++;
        // Re-running init re-registers the mode handlers and re-reads the drive's supported
        // modes, which a bare mode switch never does.
        if (motor.consecutive_failures % kModeRetriesBeforeReinit == 0)
        {
          RCLCPP_WARN(kLogger, "%s: mode switch failed %lu times, re-initializing", motor.joint_name.c_str(),
                      motor.consecutive_failures);
          motor.needs_init = true;
        }
        return;
      }
      motor.consecutive_failures = 0;
      return;
    }
  }
}

bool MotorManager::ensurePdoConfig(ManagedNode& node, const ManagedMotor& motor)
{
  node.last_pdo_check = std::chrono::steady_clock::now();

  const int result = repairPdoConfig(motor.driver, motor.node_id);
  if (result < 0)
  {
    RCLCPP_WARN(kLogger, "Node 0x%X: PDO repair failed for %d entry(s), retrying", motor.node_id, -result);
    return false;
  }

  if (result > 0)
  {
    RCLCPP_INFO(kLogger, "Node 0x%X: repaired %d PDO(s)", motor.node_id, result);
  }
  node.pdo_check_needed = false;
  return true;
}

void MotorManager::resetNode(ManagedNode& node, const ManagedMotor& motor, const char* reason)
{
  RCLCPP_WARN(kLogger, "Node 0x%X: NMT reset (%s)", motor.node_id, reason);

  motor.driver->reset_node_nmt_command();
  node.last_nmt_reset = std::chrono::steady_clock::now();
  node.pdo_check_needed = true;

  // The node loses its object dictionary on reboot, so every motor on it must re-init.
  for (auto& other : motors_)
  {
    if (other.node_id == motor.node_id)
    {
      other.needs_init = true;
      other.consecutive_failures = 0;
    }
  }
}

int MotorManager::repairPdoConfig(const std::shared_ptr<ros2_canopen::Cia402Driver>& driver, uint16_t node_id)
{
  int repaired = 0;
  int failed = 0;

  // Standard CiA301 COB-ID bases for "auto" configuration
  // RPDO: 0x200, 0x300, 0x400, 0x500 + node_id
  // TPDO: 0x180, 0x280, 0x380, 0x480 + node_id
  static constexpr uint16_t kRpdoBases[4] = { 0x200, 0x300, 0x400, 0x500 };
  static constexpr uint16_t kTpdoBases[4] = { 0x180, 0x280, 0x380, 0x480 };
  static constexpr uint8_t kNumPdos = 4;

  struct PdoGroup
  {
    uint16_t index_base;
    const uint16_t* cobid_bases;
    const char* name;
  };
  static const PdoGroup kGroups[2] = {
    { 0x1400, kRpdoBases, "RPDO" },
    { 0x1800, kTpdoBases, "TPDO" },
  };

  for (const auto& group : kGroups)
  {
    for (uint8_t n = 0; n < kNumPdos; ++n)
    {
      const uint16_t index = static_cast<uint16_t>(group.index_base + n);
      const uint32_t expected_cobid = group.cobid_bases[n] + node_id;

      ros2_canopen::COData data;
      data.index_ = index;
      data.subindex_ = 1;
      data.data_ = 0;

      if (!driver->sdo_read(data))
      {
        continue;  // PDO not implemented or SDO failed
      }

      const uint32_t current_cobid = data.data_;
      // Bit 31 disables the PDO; the low 11 bits are the COB-ID.
      if ((current_cobid & 0x80000000u) == 0 && (current_cobid & 0x7FFu) == expected_cobid)
      {
        continue;
      }

      ros2_canopen::COData fix;
      fix.index_ = index;
      fix.subindex_ = 1;
      fix.data_ = expected_cobid;  // Write expected COB-ID with bit 31 clear

      if (driver->sdo_write(fix))
      {
        repaired++;
        RCLCPP_WARN(kLogger, "Node 0x%X %s%u: fixed COB-ID 0x%08X -> 0x%08X", node_id, group.name,
                    static_cast<unsigned>(n + 1), current_cobid, expected_cobid);
      }
      else
      {
        failed++;
        RCLCPP_ERROR(kLogger, "Node 0x%X %s%u: failed to fix COB-ID 0x%08X", node_id, group.name,
                     static_cast<unsigned>(n + 1), current_cobid);
      }
    }
  }

  // Negative tells the caller to retry.
  return failed > 0 ? -failed : repaired;
}

}  // namespace canopen_ros2_control
