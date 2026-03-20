//    Copyright 2025 Georg John, Nature Robots GmbH
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
#include "canopen_ros2_control/furo_pre_mapped_system.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <chrono>


namespace
{
auto const kLogger = rclcpp::get_logger("FuroPreMappedSystem");
}

namespace canopen_ros2_control
{

FuroPreMappedSystem::FuroPreMappedSystem()
: CanopenSystem(), motor_running_(false), rollover(0)
{
}

hardware_interface::CallbackReturn FuroPreMappedSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (CanopenSystem::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::string getStateString(canopen::NmtState nmt_state)
{
  std::string message;

  switch (nmt_state)
  {
    case canopen::NmtState::BOOTUP:
      message = "BOOTUP";
      break;
    case canopen::NmtState::PREOP:
      message = "PREOP";
      break;
    case canopen::NmtState::RESET_COMM:
      message = "RESET_COMM";
      break;
    case canopen::NmtState::RESET_NODE:
      message = "RESET_NODE";
      break;
    case canopen::NmtState::START:
      message = "START";
      break;
    case canopen::NmtState::STOP:
      message = "STOP";
      break;
    case canopen::NmtState::TOGGLE:
      message = "TOGGLE";
      break;
    default:
      message = "ERROR";
      break;
  }

  return message;
}

void FuroPreMappedSystem::initDeviceContainer()
{
  std::string tmp_master_bin = (info_.hardware_parameters["master_bin"] == "\"\"") ?
    "" :
    info_.hardware_parameters["master_bin"];

  device_container_->init(
    info_.hardware_parameters["can_interface_name"], info_.hardware_parameters["master_config"],
    info_.hardware_parameters["bus_config"], tmp_master_bin);

  auto drivers = device_container_->get_registered_drivers();
  RCLCPP_INFO(kLogger, "Number of registered drivers: '%zu'", device_container_->count_drivers());
  
  // std::shared_ptr<lely::canopen::AsyncMaster>
  auto can_master = device_container_->get_master();

  if(can_master == nullptr)
  {
    // TODO: handle this case properly
    throw std::runtime_error("can_master not initialized!");
  }

  // ----------------------
  // Register callbacks FIRST so we can see boot-up after reset
  // ----------------------
  std::mutex nmt_mtx;
  std::condition_variable nmt_cv;
  std::unordered_map<uint8_t, bool> saw_boot_or_preop;

  // Prepare tracking map + register callbacks
  for (auto it = drivers.begin(); it != drivers.end(); ++it) {
    auto proxy_driver = std::dynamic_pointer_cast<ros2_canopen::PreMappedDriver>(it->second);
    if (!proxy_driver) {
      continue;
    }

    const uint8_t node_id = static_cast<uint8_t>(it->first);
    saw_boot_or_preop[node_id] = false;

    auto nmt_state_cb = [&](canopen::NmtState nmt_state, uint8_t id) {
      const uint8_t s = static_cast<uint8_t>(nmt_state);
      std::cout << "Got message. ID: " << (int)id <<", STATE: " << getStateString(nmt_state) << "(" << (int)s << ")" << std::endl;

      if(nmt_state == canopen::NmtState::PREOP || nmt_state == canopen::NmtState::START)
      {
        std::lock_guard<std::mutex> lk(nmt_mtx);
        saw_boot_or_preop[id] = true;
        nmt_cv.notify_all();
      }

      // Keep your existing storage if you want:
      canopen_data_[id].nmt_state.set_state(nmt_state);
    };

    proxy_driver->register_nmt_state_cb(nmt_state_cb);
  }

  RCLCPP_INFO_STREAM(kLogger, "Resetting Application ...");
  can_master->Command(lely::canopen::NmtCommand::RESET_NODE, 0x00);
  // Wait for boot-up / pre-op from all tracked nodes (timeout)
  {
    std::unique_lock<std::mutex> lk(nmt_mtx);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);

    bool ok = nmt_cv.wait_until(lk, deadline, [&]() {
      for (const auto &kv : saw_boot_or_preop) {
        if (!kv.second) return false;
      }
      return true;
    });

    if (!ok) {
      RCLCPP_ERROR_STREAM(kLogger, "Timeout waiting for Boot-up/Pre-op/Start after RESET_NODE");

    } else {
      RCLCPP_INFO_STREAM(kLogger, "Boot-up/Pre-op/Start observed for all nodes after RESET_NODE");
    }
  }
  std::this_thread::sleep_for(2000ms); // Keep this.
  
  RCLCPP_INFO_STREAM(kLogger, "Resetting CANopen communication ...");
  can_master->Command(lely::canopen::NmtCommand::RESET_COMM, 0x00);
  std::this_thread::sleep_for(1000ms); // Keep this.
  
  RCLCPP_INFO_STREAM(kLogger, "Configure Devices...");
  for (auto it = drivers.begin(); it != drivers.end(); it++) {
    auto pre_mapped_driver = std::dynamic_pointer_cast<ros2_canopen::PreMappedDriver>(it->second);
    if (pre_mapped_driver) {
      pre_mapped_driver->configure_device();
    }
  }

  RCLCPP_INFO_STREAM(kLogger, "CANopen reset done.");

  for (auto it = drivers.begin(); it != drivers.end(); it++) {
    auto proxy_driver = std::static_pointer_cast<ros2_canopen::PreMappedDriver>(it->second);

    canopen_data_[it->first] = CanopenNodeData();

    auto nmt_state_cb = [&](canopen::NmtState nmt_state, uint8_t id)
      {canopen_data_[id].nmt_state.set_state(nmt_state);};
    // register callback
    proxy_driver->register_nmt_state_cb(nmt_state_cb);

    auto rpdo_cb = [&](ros2_canopen::COData data, uint8_t id)
      {canopen_data_[id].rpdo_data.set_data(data);};
    // register callback
    proxy_driver->register_rpdo_cb(rpdo_cb);

    RCLCPP_INFO(
      kLogger, "\nRegistered driver:\n    name: '%s'\n    node_id: '0x%X'",  //
      it->second->get_node_base_interface()->get_name(), it->first);
  }

  RCLCPP_INFO(device_container_->get_logger(), "Initialisation successful.");
}

hardware_interface::CallbackReturn FuroPreMappedSystem::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  device_container_ = std::make_shared<ros2_canopen::DeviceContainer>(executor_);
  executor_->add_node(device_container_);

  // threads
  spin_thread_ = std::make_unique<std::thread>(&FuroPreMappedSystem::spin, this);
  init_thread_ = std::make_unique<std::thread>(&FuroPreMappedSystem::initDeviceContainer, this);

  // actually wait for init phase to end
  if (init_thread_->joinable()) {
    init_thread_->join();
  } else {
    RCLCPP_ERROR(kLogger, "Could not join init thread!");
    return CallbackReturn::ERROR;
  }
  rclcpp::sleep_for(std::chrono::milliseconds(3000));
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FuroPreMappedSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // underlying base class export first
  state_interfaces = CanopenSystem::export_state_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++) {

    auto joint_name = info_.joints[i].name;

    RCLCPP_INFO_STREAM(
      kLogger, "Exporting state interfaces for joint: " << joint_name);

    // actual position
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, hardware_interface::HW_IF_POSITION,
        &motor_data_[joint_name].actual_position));
    // actual speed
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_name, hardware_interface::HW_IF_VELOCITY,
        &motor_data_[joint_name].actual_speed));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FuroPreMappedSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // underlying base class export first
  command_interfaces = CanopenSystem::export_command_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++) {
    auto joint_name = info_.joints[i].name;

    RCLCPP_INFO_STREAM(
      kLogger, "Exporting command interfaces for joint: " << joint_name);

    // target
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_name, hardware_interface::HW_IF_VELOCITY,
        &motor_data_[joint_name].target_velocity));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn FuroPreMappedSystem::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret_val = CanopenSystem::on_activate(previous_state);
  auto can_master = device_container_->get_master();

  // can_master->Command(lely::canopen::NmtCommand::START, 0x00);
  // rclcpp::sleep_for(std::chrono::milliseconds(100));
  auto drivers = device_container_->get_registered_drivers();
  for (auto it = drivers.begin(); it != drivers.end(); ++it) {
    auto pre_mapped_driver = std::dynamic_pointer_cast<ros2_canopen::PreMappedDriver>(it->second);
    if (pre_mapped_driver != nullptr) {
      pre_mapped_driver->init_motor();
    }
  }
  motor_running_ = true; // Set the motor running flag to true on activation
  RCLCPP_INFO_STREAM(kLogger, "FuroPreMappedSystem ACTIVATED");
  return ret_val;
}

hardware_interface::CallbackReturn FuroPreMappedSystem::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  motor_running_ = false;
  auto drivers = device_container_->get_registered_drivers();
  for (auto it = drivers.begin(); it != drivers.end(); ++it) {
    auto driver = std::dynamic_pointer_cast<ros2_canopen::PreMappedDriver>(it->second);
    if (driver) {
      driver->get_node_canopen_driver_interface()->deactivate();
    }
  }
  auto ret_val = CanopenSystem::on_deactivate(previous_state);
  RCLCPP_INFO_STREAM(kLogger, "FuroPreMappedSystem DEACTIVATED");
  return ret_val;
}

hardware_interface::return_type FuroPreMappedSystem::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  auto ret_val = CanopenSystem::read(time, period);

  // read values and write them to joint interface
  
  auto drivers = device_container_->get_registered_drivers();

  for (auto it = drivers.begin(); it != drivers.end(); ++it) {
    auto pre_mapped_driver = std::dynamic_pointer_cast<ros2_canopen::PreMappedDriver>(it->second);

    if (pre_mapped_driver != nullptr && motor_running_) {
      auto joint_name = pre_mapped_driver->get_motor_joint_name();

      // get position
      // motor_data_[joint_name].actual_position =
      //   pre_mapped_driver->get_position();

      // get speed
      motor_data_[joint_name].actual_speed = pre_mapped_driver->get_speed();
      }
  }

  return ret_val;
}

hardware_interface::return_type FuroPreMappedSystem::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  auto ret_val = CanopenSystem::write(time, period);
  auto drivers = device_container_->get_registered_drivers();
  for (auto it = drivers.begin(); it != drivers.end(); ++it) {
    auto pre_mapped_driver = std::dynamic_pointer_cast<ros2_canopen::PreMappedDriver>(it->second);
    if (pre_mapped_driver != nullptr && motor_running_) {
      auto joint_name = pre_mapped_driver->get_motor_joint_name();
      double target_velocity = motor_data_[joint_name].target_velocity;
      bool success = pre_mapped_driver->set_target(target_velocity, rollover);
      if(!success)
      {
        // what to do here?

        // initDeviceContainer();
      }
    }
  }
  rollover++;
  return hardware_interface::return_type::OK;
}


} // namespace canopen_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  canopen_ros2_control::FuroPreMappedSystem,
  hardware_interface::SystemInterface)
