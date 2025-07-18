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

void FuroPreMappedSystem::initDeviceContainer()
{
  std::string tmp_master_bin = (info_.hardware_parameters["master_bin"] == "\"\"") ?
    "" :
    info_.hardware_parameters["master_bin"];

  device_container_->init(
    info_.hardware_parameters["can_interface_name"], info_.hardware_parameters["master_config"],
    info_.hardware_parameters["bus_config"], tmp_master_bin);

  auto can_master = device_container_->get_master();
  RCLCPP_INFO_STREAM(kLogger, "Resetting CANopen master...");
  can_master->Command(lely::canopen::NmtCommand::RESET_NODE, 0x00);
  rclcpp::sleep_for(std::chrono::milliseconds(2000));
  RCLCPP_INFO_STREAM(kLogger, "CANopen master reset done.");

  auto drivers = device_container_->get_registered_drivers();
  RCLCPP_INFO(kLogger, "Number of registered drivers: '%zu'", device_container_->count_drivers());

  for (auto it = drivers.begin(); it != drivers.end(); it++) {
    auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(it->second);

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
  rclcpp::sleep_for(std::chrono::milliseconds(2000));
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FuroPreMappedSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // underlying base class export first
  state_interfaces = CanopenSystem::export_state_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++) {

    auto joint_name = info_.joints[i].name;

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
    // target
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &motor_data_[info_.joints[i].name].target_velocity));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn FuroPreMappedSystem::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret_val = CanopenSystem::on_activate(previous_state);
  auto can_master = device_container_->get_master();
  can_master->Command(lely::canopen::NmtCommand::START, 0x00);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  auto drivers = device_container_->get_registered_drivers();
  for (auto it = drivers.begin(); it != drivers.end(); ++it) {
    auto driver = std::dynamic_pointer_cast<ros2_canopen::PreMappedDriver>(it->second);
    if (driver) {
      // driver->get_node_canopen_driver_interface()->activate();
    }
  }
  motor_running_ = true; // Set the motor running flag to true on activation
  rollover = 0; // Reset rollover on activation
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
  return ret_val;
}

hardware_interface::return_type FuroPreMappedSystem::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  // auto ret_val = CanopenSystem::write(time, period);
  auto drivers = device_container_->get_registered_drivers();
  for (auto it = drivers.begin(); it != drivers.end(); ++it) {
    auto pre_mapped_driver = std::dynamic_pointer_cast<ros2_canopen::PreMappedDriver>(it->second);
    if (pre_mapped_driver != nullptr && motor_running_) {
      auto joint_name = pre_mapped_driver->get_motor_joint_name();
      auto target_velocity = motor_data_[joint_name].target_velocity;
      // pre_mapped_driver->set_target(target_velocity, rollover);
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
