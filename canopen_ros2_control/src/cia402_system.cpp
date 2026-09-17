// Copyright (c) 2024, Malte kleine Piening, Nature Robots GmbH
// Copyright (c) 2022, StoglRobotics
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2022-08-01
 *
 */
//----------------------------------------------------------------------

#include "canopen_ros2_control/cia402_system.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <thread>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <tuple>
#include <vector>

namespace
{
auto const kLogger = rclcpp::get_logger("Cia402System");
}

namespace canopen_ros2_control
{

Cia402System::Cia402System() : CanopenSystem()
{
}

Cia402System::~Cia402System()
{
  // Before ~CanopenSystem() destroys the device container the manager thread uses.
  motor_manager_.stop();
  // Wait for any in-progress homing to finish
  while (homing_active_count_.load() > 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (service_executor_)
  {
    service_executor_->cancel();
  }
  if (service_spin_thread_ && service_spin_thread_->joinable())
  {
    service_spin_thread_->join();
  }
}

hardware_interface::CallbackReturn Cia402System::on_init(const hardware_interface::HardwareInfo& info)
{
  if (CanopenSystem::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Parse joint parameters for position offset opt-in
  for (const auto& joint : info.joints)
  {
    auto it = joint.parameters.find("enable_position_offset");
    if (it != joint.parameters.end() && (it->second == "true" || it->second == "1"))
    {
      offset_enabled_joints_.insert(joint.name);
      RCLCPP_INFO(kLogger, "Position offset enabled for joint: %s", joint.name.c_str());
    }
  }

  // Read offset file path from hardware_parameters (optional, has default)
  auto it = info.hardware_parameters.find("position_offset_file");
  if (it != info.hardware_parameters.end() && !it->second.empty())
  {
    offset_file_path_ = it->second;
  }
  // Expand ~ to home directory
  if (!offset_file_path_.empty() && offset_file_path_[0] == '~')
  {
    const char* home = std::getenv("HOME");
    if (home)
    {
      offset_file_path_ = std::string(home) + offset_file_path_.substr(1);
    }
  }
  RCLCPP_INFO(kLogger, "Position offset file: %s", offset_file_path_.c_str());

  // Read cold start threshold from hardware_parameters (optional, default 0.1 rad)
  // If raw position changed more than this threshold since last save, assume cold start
  // (motor lost power and position reset). Otherwise assume warm restart (motor kept position).
  auto threshold_it = info.hardware_parameters.find("cold_start_threshold");
  if (threshold_it != info.hardware_parameters.end() && !threshold_it->second.empty())
  {
    cold_start_threshold_ = std::stod(threshold_it->second);
  }
  RCLCPP_INFO(kLogger, "Cold start threshold: %.3f rad", cold_start_threshold_);

  return CallbackReturn::SUCCESS;
}

void Cia402System::initDeviceContainer()
{
  std::string tmp_master_bin =
      (info_.hardware_parameters["master_bin"] == "\"\"") ? "" : info_.hardware_parameters["master_bin"];

  device_container_->init(info_.hardware_parameters["can_interface_name"], info_.hardware_parameters["master_config"],
                          info_.hardware_parameters["bus_config"], tmp_master_bin);
  auto drivers = device_container_->get_registered_drivers();

  RCLCPP_INFO(kLogger, "Number of registered drivers: '%lu'", device_container_->count_drivers());
  for (auto it = drivers.begin(); it != drivers.end(); it++)
  {
    auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(it->second);

    // initialize data for each node
    canopen_data_[it->first] = CanopenNodeData();

    // initialize data for each node
    canopen_data_[it->first] = CanopenNodeData();

    auto nmt_state_cb = [&](canopen::NmtState nmt_state, uint8_t id) {
      canopen_data_[id].nmt_state.set_state(nmt_state);
    };
    // register callback
    driver->register_nmt_state_cb(nmt_state_cb);

    auto rpdo_cb = [&](ros2_canopen::COData data, uint8_t id) { canopen_data_[id].rpdo_data.set_data(data); };
    // register callback
    driver->register_rpdo_cb(rpdo_cb);

    RCLCPP_INFO(kLogger, "\nRegistered driver:\n    name: '%s'\n    node_id: '0x%X'",
                it->second->get_node_base_interface()->get_name(), it->first);
  }

  RCLCPP_INFO(device_container_->get_logger(), "Initialisation successful.");
}

hardware_interface::CallbackReturn Cia402System::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // ponytail: SingleThreadedExecutor reduces thread contention with controller_manager
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  device_container_ = std::make_shared<ros2_canopen::DeviceContainer>(executor_);
  executor_->add_node(device_container_);

  // threads - spin must start before init as init needs executor callbacks
  spin_thread_ = std::make_unique<std::thread>(&Cia402System::spin, this);
  init_thread_ = std::make_unique<std::thread>(&Cia402System::initDeviceContainer, this);

  // actually wait for init phase to end
  if (init_thread_->joinable())
  {
    init_thread_->join();

    // Note: configure() is already called inside CanopenDriver::init()
    // No need to call it separately here
  }
  else
  {
    RCLCPP_ERROR(kLogger, "Could not join init thread!");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Cia402System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // underlying base class export first
  state_interfaces = CanopenSystem::export_state_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    // {
    //   // skip adding motor canopen interfaces
    //   RCLCPP_WARN_STREAM(kLogger, "skipping joint " << i);
    //   continue;
    // }
    // const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));

    // actual position
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motor_data_[info_.joints[i].name].actual_position));
    // actual speed
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motor_data_[info_.joints[i].name].actual_speed));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Cia402System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // underlying base class export first
  command_interfaces = CanopenSystem::export_command_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    // {
    //   // skip adding canopen interfaces
    //   continue;
    // }

    // const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));

    // target
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motor_data_[info_.joints[i].name].target_position));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motor_data_[info_.joints[i].name].target_velocity));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &motor_data_[info_.joints[i].name].target_torque));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Cia402System::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  // Nothing here touches the bus, so activation returns immediately even with the e-stop
  // engaged. Init, fault recovery and PDO repair all happen on the manager thread.
  motor_manager_.configure(device_container_->get_registered_drivers());

  for (const auto& joint_name : motor_manager_.joint_names())
  {
    position_offsets_[joint_name] = 0.0;
  }

  // Offsets are initialized from read(), once the drives are powered and operational.
  offsets_initialized_ = false;

  // Create service node and reset home service
  service_node_ = std::make_shared<rclcpp::Node>("cia402_system_services");
  reset_position_home_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "~/reset_position_home",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      auto drivers = device_container_->get_registered_drivers();
      for (auto it = drivers.begin(); it != drivers.end(); ++it)
      {
        auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(it->second);
        for (auto channel : driver->get_available_motor_channels())
        {
          std::string joint = driver->get_motor_joint_name(channel);
          double raw = driver->get_position(channel);
          std::scoped_lock lock(offset_mutex_);
          // Only reset offset for enabled joints
          if (offset_enabled_joints_.count(joint) == 0) continue;
          position_offsets_[joint] = -raw;
        }
      }
      savePositionOffsets();
      RCLCPP_INFO(kLogger, "Position home reset complete");
      response->success = true;
      response->message = "Position home reset complete";
    });

  adjust_position_offset_service_ = service_node_->create_service<canopen_ros2_control::srv::AdjustPositionOffset>(
    "~/adjust_position_offset",
    [this](const std::shared_ptr<canopen_ros2_control::srv::AdjustPositionOffset::Request> request,
           std::shared_ptr<canopen_ros2_control::srv::AdjustPositionOffset::Response> response) {
      const std::string& joint = request->joint_name;
      double new_offset;
      {
        std::scoped_lock lock(offset_mutex_);
        // Check if joint exists and has offset enabled
        if (offset_enabled_joints_.count(joint) == 0)
        {
          response->success = false;
          response->message = "Joint not found or offset not enabled: " + joint;
          RCLCPP_WARN(kLogger, "Adjust offset failed: %s", response->message.c_str());
          return;
        }
        position_offsets_[joint] += request->offset_delta;
        new_offset = position_offsets_[joint];
      }
      savePositionOffsets();

      RCLCPP_INFO(kLogger, "Adjusted offset for %s by %.4f, new offset: %.4f",
                  joint.c_str(), request->offset_delta, new_offset);
      response->success = true;
      response->message = "Offset adjusted successfully";
    });

  home_joint_service_ = service_node_->create_service<canopen_ros2_control::srv::HomeJoint>(
    "~/home_joint",
    [this](const std::shared_ptr<canopen_ros2_control::srv::HomeJoint::Request> request,
           std::shared_ptr<canopen_ros2_control::srv::HomeJoint::Response> response) {
      const std::string& joint = request->joint_name;

      // Check if this specific joint is already homing
      {
        std::scoped_lock lock(homing_mutex_);
        if (homing_joints_.count(joint) > 0)
        {
          response->success = false;
          response->message = "Joint " + joint + " is already homing";
          RCLCPP_WARN(kLogger, "Homing rejected: %s", response->message.c_str());
          return;
        }
      }

      std::shared_ptr<ros2_canopen::Cia402Driver> found_driver;
      uint8_t found_channel = 0;
      auto drivers = device_container_->get_registered_drivers();
      for (auto it = drivers.begin(); it != drivers.end() && !found_driver; ++it)
      {
        auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(it->second);
        for (auto channel : driver->get_available_motor_channels())
        {
          if (driver->get_motor_joint_name(channel) == joint)
          {
            found_driver = driver;
            found_channel = channel;
            break;
          }
        }
      }

      if (!found_driver || !found_driver->is_homing_enabled(found_channel))
      {
        response->success = false;
        response->message = "Joint not found or homing not enabled: " + joint;
        RCLCPP_WARN(kLogger, "Homing failed: %s", response->message.c_str());
        return;
      }

      // Runs inline on this executor thread. MultiThreadedExecutor allows concurrent calls.
      RCLCPP_INFO(kLogger, "Homing: starting for %s (blocking until it finishes)", joint.c_str());
      HomingResult result = runHomingSequence(found_driver, found_channel, joint);

      response->success = result.success;
      response->message = result.message;
      RCLCPP_INFO(kLogger, "Homing: %s for %s - %s", result.success ? "succeeded" : "failed",
                  joint.c_str(), result.message.c_str());
    });

  // MultiThreadedExecutor allows concurrent homing of multiple joints via separate service calls.
  // Deliberately NOT executor_->add_node(service_node_) - see the member comment in cia402_system.hpp.
  service_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  service_executor_->add_node(service_node_);
  service_spin_thread_ = std::make_unique<std::thread>([this]() { service_executor_->spin(); });

  last_offset_save_time_ = rclcpp::Clock().now();

  motor_manager_.start();

  return CanopenSystem::on_activate(previous_state);
}

hardware_interface::CallbackReturn Cia402System::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  // Stop the manager first, or it would fight the halt below by re-enabling motors.
  motor_manager_.stop();

  // Wait for any in-progress homing operations to finish before tearing down
  if (homing_active_count_.load() > 0)
  {
    RCLCPP_WARN(kLogger, "Waiting for %d in-progress homing operation(s) to finish before deactivating",
                homing_active_count_.load());
    while (homing_active_count_.load() > 0)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  auto drivers = device_container_->get_registered_drivers();
  for (auto it = drivers.begin(); it != drivers.end(); ++it)
  {
    auto motion_controller_driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(it->second);

    for (auto motor_channel : motion_controller_driver->get_available_motor_channels())
    {
      if (!motion_controller_driver->halt_motor(motor_channel))
      {
        RCLCPP_ERROR_STREAM(kLogger, "Failed to deactivate motor for joint "
                                         << motion_controller_driver->get_motor_joint_name(motor_channel));
        return CallbackReturn::FAILURE;
      }
    }
  }
  return CanopenSystem::on_deactivate(previous_state);
}

hardware_interface::return_type Cia402System::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  auto ret_val = CanopenSystem::read(time, period);

  auto drivers = device_container_->get_registered_drivers();
  bool any_enabled_joint_has_com_failure = false;

  for (auto it = drivers.begin(); it != drivers.end(); ++it)
  {
    auto motion_controller_driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(it->second);

    bool com_failure = false;
    for (auto motor_channel : motion_controller_driver->get_available_motor_channels())
    {
      std::string joint_name = motion_controller_driver->get_motor_joint_name(motor_channel);
      double raw_position = motion_controller_driver->get_position(motor_channel);

      double offset = 0.0;
      bool offset_enabled = false;
      {
        std::scoped_lock lock(offset_mutex_);
        offset_enabled = offset_enabled_joints_.count(joint_name) > 0;
        if (offset_enabled)
        {
          offset = position_offsets_[joint_name];
        }
      }
      motor_data_[joint_name].actual_position = raw_position + offset;
      motor_data_[joint_name].actual_speed = motion_controller_driver->get_speed(motor_channel);

      // check for communication failure
      if (motion_controller_driver->has_motor_communication_failure(motor_channel))
      {
        com_failure = true;
        if (offset_enabled)
        {
          any_enabled_joint_has_com_failure = true;
        }
      }
    }

    if (com_failure)
    {
      for (auto motor_channel : motion_controller_driver->get_available_motor_channels())
      {
        motor_data_[motion_controller_driver->get_motor_joint_name(motor_channel)].actual_position = 0.0;
        motor_data_[motion_controller_driver->get_motor_joint_name(motor_channel)].actual_speed = 0.0;
      }
    }
  }

  // Initialize offsets on first successful read of all enabled joints
  // Only once the drives are powered. With the e-stop engaged CANopen answers happily while
  // the drives are dead, and offsets captured then get persisted and reused on the next boot.
  bool have_enabled_joints = false;
  {
    std::scoped_lock lock(offset_mutex_);
    have_enabled_joints = !offset_enabled_joints_.empty();
  }
  if (!offsets_initialized_ && motor_manager_.is_operational() && !any_enabled_joint_has_com_failure &&
      have_enabled_joints)
  {
    initializePositionOffsets();
    offsets_initialized_ = true;
    last_offset_save_time_ = rclcpp::Clock().now();
  }

  // Periodic save of position offsets (every 0.5s)
  // ponytail: hardcoded interval, make configurable if needed
  if (offsets_initialized_)
  {
    auto now = rclcpp::Clock().now();
    if ((now - last_offset_save_time_).seconds() >= 0.5)
    {
      savePositionOffsets();
      last_offset_save_time_ = now;
    }
  }

  return ret_val;
}

void Cia402System::stop_all_motors()
{
  // Called every cycle while the drives are down, so it stays quiet; write() logs the
  // transition once.
  auto drivers = device_container_->get_registered_drivers();
  for (auto it = drivers.begin(); it != drivers.end(); ++it)
  {
    auto motion_controller_driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(it->second);
    for (auto motor_channel : motion_controller_driver->get_available_motor_channels())
    {
      motion_controller_driver->set_target(motor_channel, 0);
    }
  }
}

hardware_interface::return_type Cia402System::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  auto drivers = device_container_->get_registered_drivers();

  // The manager thread owns init, fault recovery, mode switching and PDO repair. The only
  // decision left here is whether the drives may be commanded at all - which keeps write()
  // free of blocking CAN round trips.
  const bool operational = motor_manager_.is_operational();
  if (operational != drives_operational_)
  {
    RCLCPP_INFO(kLogger, "%s", operational ? "All motors operational, accepting commands" :
                                            "Motors not operational, stopping all motors");
    drives_operational_ = operational;
  }

  if (!operational)
  {
    stop_all_motors();
    return hardware_interface::return_type::OK;
  }

  for (auto it = drivers.begin(); it != drivers.end(); ++it)
  {
    auto motion_controller_driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(it->second);

    // do same as in proxy system first - handle nmt, tpdo, rpdo
    // reset node nmt
    if (canopen_data_[it->first].nmt_state.reset_command())
    {
      motion_controller_driver->reset_node_nmt_command();
      // The node loses its object dictionary on reboot, so PDOs and init must be redone.
      motor_manager_.notify_external_reset();
    }

    // start nmt
    if (canopen_data_[it->first].nmt_state.start_command())
    {
      motion_controller_driver->start_node_nmt_command();
    }

    // tpdo data one shot mechanism
    if (canopen_data_[it->first].tpdo_data.write_command())
    {
      canopen_data_[it->first].tpdo_data.prepare_data();
      motion_controller_driver->tpdo_transmit(canopen_data_[it->first].tpdo_data.original_data);
    }

    for (auto motor_channel : motion_controller_driver->get_available_motor_channels())
    {
      std::string joint_name = motion_controller_driver->get_motor_joint_name(motor_channel);

      // While any joint is homing, override homing joints' targets with their configured homing
      // speeds - through the exact same set_target()/PDO pathway as any other velocity command.
      // Non-homing motors are held at zero for the duration.
      if (homing_active_count_.load(std::memory_order_acquire) > 0)
      {
        double target = 0.0;
        {
          std::scoped_lock lock(homing_mutex_);
          auto it = homing_joints_.find(joint_name);
          if (it != homing_joints_.end())
          {
            target = it->second;
          }
        }
        motion_controller_driver->set_target(motor_channel, target);
        continue;
      }

      const uint16_t& mode = motion_controller_driver->get_mode(motor_channel);

      switch (mode)
      {
        case MotorBase::No_Mode:
          break;
        case MotorBase::Profiled_Position:
        case MotorBase::Cyclic_Synchronous_Position:
        case MotorBase::Interpolated_Position:
        {
          double offset = 0.0;
          {
            std::scoped_lock lock(offset_mutex_);
            if (offset_enabled_joints_.count(joint_name) > 0)
            {
              offset = position_offsets_[joint_name];
            }
          }
          motion_controller_driver->set_target(
              motor_channel,
              motor_data_[joint_name].target_position - offset);
          break;
        }
        case MotorBase::Profiled_Velocity:
        case MotorBase::Velocity:
        case MotorBase::Cyclic_Synchronous_Velocity:
          motion_controller_driver->set_target(
              motor_channel,
              motor_data_[joint_name].target_velocity);
          break;
        case MotorBase::Profiled_Torque:
        case MotorBase::Cyclic_Synchronous_Torque:
          motion_controller_driver->set_target(
              motor_channel, motor_data_[joint_name].target_torque);
          break;
        default:
          RCLCPP_INFO(kLogger, "Mode %u not supported", mode);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

bool Cia402System::loadPositionOffsets(std::map<std::string, double>& saved_raw, std::map<std::string, double>& saved_offsets)
{
  std::ifstream file(offset_file_path_);
  if (!file.is_open()) return false;

  std::string line;
  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    std::string joint_name;
    double raw, offset;
    if (iss >> joint_name >> raw >> offset)
    {
      saved_raw[joint_name] = raw;
      saved_offsets[joint_name] = offset;
    }
  }
  return !saved_raw.empty();
}

void Cia402System::savePositionOffsets()
{
  std::filesystem::path file_path(offset_file_path_);
  if (file_path.has_parent_path())
  {
    std::filesystem::create_directories(file_path.parent_path());
  }

  // Snapshot under the lock; do the file I/O without holding it.
  std::vector<std::tuple<std::string, double, double>> entries;  // joint, raw, offset
  auto drivers = device_container_->get_registered_drivers();
  for (auto it = drivers.begin(); it != drivers.end(); ++it)
  {
    auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(it->second);
    for (auto channel : driver->get_available_motor_channels())
    {
      std::string joint_name = driver->get_motor_joint_name(channel);
      double raw = driver->get_position(channel);
      std::scoped_lock lock(offset_mutex_);
      // Only save offset-enabled joints
      if (offset_enabled_joints_.count(joint_name) == 0) continue;
      entries.emplace_back(joint_name, raw, position_offsets_[joint_name]);
    }
  }

  std::string tmp_path = offset_file_path_ + ".tmp";
  std::ofstream file(tmp_path);
  if (!file.is_open())
  {
    RCLCPP_WARN(kLogger, "Failed to open position offset file for writing: %s", tmp_path.c_str());
    return;
  }
  for (const auto& entry : entries)
  {
    file << std::get<0>(entry) << " " << std::get<1>(entry) << " " << std::get<2>(entry) << "\n";
  }
  file.close();

  // ponytail: rename is atomic on POSIX, protects against power-loss corruption
  std::rename(tmp_path.c_str(), offset_file_path_.c_str());
}

void Cia402System::initializePositionOffsets()
{
  std::map<std::string, double> saved_raw, saved_offsets;

  if (!loadPositionOffsets(saved_raw, saved_offsets))
  {
    RCLCPP_INFO(kLogger, "No position offset file found, starting with zero offsets for enabled joints");
    return;
  }

  std::scoped_lock lock(offset_mutex_);
  auto drivers = device_container_->get_registered_drivers();
  for (auto it = drivers.begin(); it != drivers.end(); ++it)
  {
    auto driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(it->second);
    for (auto channel : driver->get_available_motor_channels())
    {
      std::string joint_name = driver->get_motor_joint_name(channel);

      // Only process offset-enabled joints
      if (offset_enabled_joints_.count(joint_name) == 0) continue;

      if (saved_raw.find(joint_name) == saved_raw.end())
      {
        RCLCPP_WARN(kLogger, "No saved offset for joint %s, using 0", joint_name.c_str());
        continue;
      }

      double current_raw = driver->get_position(channel);
      double prev_raw = saved_raw[joint_name];
      double prev_offset = saved_offsets[joint_name];

      // Check if motors reset (cold start) or stayed on (warm restart)
      if (std::abs(current_raw - prev_raw) < cold_start_threshold_)
      {
        // Warm restart: motors kept position, use saved offset
        position_offsets_[joint_name] = prev_offset;
        RCLCPP_INFO(kLogger, "Joint %s warm restart, offset: %.3f", joint_name.c_str(), prev_offset);
      }
      else
      {
        // Cold start: motors reset, calculate new offset
        // saved_actual = prev_raw + prev_offset
        // new_offset = saved_actual - current_raw
        double saved_actual = prev_raw + prev_offset;
        position_offsets_[joint_name] = saved_actual - current_raw;
        RCLCPP_INFO(kLogger, "Joint %s cold start, new offset: %.3f", joint_name.c_str(), position_offsets_[joint_name]);
      }
    }
  }
}

Cia402System::HomingResult Cia402System::runHomingSequence(std::shared_ptr<ros2_canopen::Cia402Driver> driver,
                                                            uint8_t channel, const std::string& joint_name)
{
  const double speed = driver->get_homing_speed(channel);
  const double home_offset = driver->get_home_offset(channel);
  const uint16_t switch_index = driver->get_home_switch_index(channel);
  const uint8_t switch_subindex = driver->get_home_switch_subindex(channel);
  const int32_t switch_active_value = driver->get_home_switch_active_value(channel);
  const double max_travel = driver->get_home_max_travel(channel);
  const double timeout_s = driver->get_homing_timeout(channel);
  const double start_position = driver->get_position(channel);

  RCLCPP_INFO(kLogger, "Homing %s: driving at %.4f rad/s, watching 0x%04X:%hhu for value %d, "
                       "max travel %.4f rad, timeout %.1f s",
              joint_name.c_str(), speed, switch_index, switch_subindex, switch_active_value, max_travel,
              timeout_s);

  // Register this joint for homing - write() will override its target with homing speed
  {
    std::scoped_lock lock(homing_mutex_);
    homing_joints_[joint_name] = speed;
  }
  ++homing_active_count_;

  bool triggered = false;
  bool aborted_for_safety = false;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(static_cast<int64_t>(timeout_s * 1000.0));
  while (std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // is_home_switch_triggered() uses the same bounded-timeout mechanism get_position() does
    // (see Motor402::isHomeSwitchTriggered()'s comment) - driver->sdo_read(COData) was used
    // here originally and hung forever on this device's BOOLEAN-typed switch object, which is
    // exactly why the travel check below exists too: the switch read must never be the only
    // thing standing between a homing request and unbounded motion.
    bool switch_triggered = false;
    if (driver->is_home_switch_triggered(channel, switch_triggered) && switch_triggered)
    {
      triggered = true;
      break;
    }

    const double traveled = std::abs(driver->get_position(channel) - start_position);
    if (traveled > max_travel)
    {
      RCLCPP_ERROR(kLogger, "Homing %s: exceeded max travel (%.4f > %.4f rad) without seeing the "
                            "home switch - aborting for safety", joint_name.c_str(), traveled, max_travel);
      aborted_for_safety = true;
      break;
    }
  }

  // Stop overriding this joint's target - remove from homing map
  {
    std::scoped_lock lock(homing_mutex_);
    homing_joints_.erase(joint_name);
  }
  --homing_active_count_;

  if (aborted_for_safety)
  {
    return { false, "Exceeded max travel without seeing the home switch" };
  }

  if (!triggered)
  {
    RCLCPP_ERROR(kLogger, "Homing %s: timed out waiting for the home switch", joint_name.c_str());
    return { false, "Timed out waiting for the home switch" };
  }

  const double raw_position = driver->get_position(channel);
  const double new_offset = home_offset - raw_position;
  {
    std::scoped_lock lock(offset_mutex_);
    position_offsets_[joint_name] = new_offset;
    offset_enabled_joints_.insert(joint_name);
  }
  savePositionOffsets();

  RCLCPP_INFO(kLogger, "Homing %s: complete - raw position %.4f, offset %.4f, now reports %.4f",
              joint_name.c_str(), raw_position, new_offset, raw_position + new_offset);

  std::ostringstream msg;
  msg << "Homing complete - raw position " << raw_position << ", offset " << new_offset
      << ", now reports " << (raw_position + new_offset);
  return { true, msg.str() };
}

}  // namespace canopen_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(canopen_ros2_control::Cia402System, hardware_interface::SystemInterface)
