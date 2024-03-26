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

namespace
{
auto const kLogger = rclcpp::get_logger("Cia402System");
}

namespace canopen_ros2_control
{

Cia402System::Cia402System() : CanopenSystem()
{
}

hardware_interface::CallbackReturn Cia402System::on_init(const hardware_interface::HardwareInfo& info)
{
  if (CanopenSystem::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

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
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  device_container_ = std::make_shared<ros2_canopen::DeviceContainer>(executor_);
  executor_->add_node(device_container_);

  // threads
  spin_thread_ = std::make_unique<std::thread>(&Cia402System::spin, this);
  init_thread_ = std::make_unique<std::thread>(&Cia402System::initDeviceContainer, this);

  // actually wait for init phase to end
  if (init_thread_->joinable())
  {
    init_thread_->join();

    // TODO(livanov93): see how to handle configure once LifecycleCia402Driver is introduced
    /*
    auto drivers = device_container_->get_registered_drivers();
    for (auto it = drivers.begin(); it != drivers.end(); it++) {
        auto d = std::static_pointer_cast<ros2_canopen::LifecycleCia402Driver>(it->second);
        d->configure();
    }
    */
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
  hardware_interface::CallbackReturn return_value = hardware_interface::CallbackReturn::SUCCESS;

  auto drivers = device_container_->get_registered_drivers();
  for (auto it = drivers.begin(); it != drivers.end(); ++it)
  {
    auto motion_controller_driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(it->second);

    for (auto motor_channel : motion_controller_driver->get_available_motor_channels())
    {
      RCLCPP_INFO_STREAM(kLogger, "Init motor " << it->first << " channel " << (int)motor_channel << " joint_name: "
                                                << motion_controller_driver->get_motor_joint_name(motor_channel));
      if (!motion_controller_driver->init_motor(motor_channel))
      {
        RCLCPP_ERROR_STREAM(kLogger, "Failed to init motor "
                                         << it->first << " channel " << (int)motor_channel << " joint_name: "
                                         << motion_controller_driver->get_motor_joint_name(motor_channel));
        return_value = hardware_interface::CallbackReturn::ERROR;
      }

      RCLCPP_INFO_STREAM(kLogger, "Set operation mode for motor "
                                      << it->first << " channel " << (int)motor_channel << " joint_name: "
                                      << motion_controller_driver->get_motor_joint_name(motor_channel));
      if (!motion_controller_driver->set_default_operation_mode(motor_channel))
      {
        RCLCPP_ERROR_STREAM(kLogger, "Failed to set operation mode for motor "
                                         << it->first << " channel " << (int)motor_channel << " joint_name: "
                                         << motion_controller_driver->get_motor_joint_name(motor_channel));
        return_value = hardware_interface::CallbackReturn::ERROR;
      }
    }
  }
  hardware_interface::CallbackReturn super_return_value = CanopenSystem::on_activate(previous_state);
  return super_return_value == hardware_interface::CallbackReturn::SUCCESS ? return_value : super_return_value;
}

hardware_interface::CallbackReturn Cia402System::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
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
  // TODO(anyone): read robot states

  auto ret_val = CanopenSystem::read(time, period);

  auto drivers = device_container_->get_registered_drivers();

  for (auto it = drivers.begin(); it != drivers.end(); ++it)
  {
    auto motion_controller_driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(it->second);

    for (auto motor_channel : motion_controller_driver->get_available_motor_channels())
    {
      // get position
      motor_data_[motion_controller_driver->get_motor_joint_name(motor_channel)].actual_position =
          motion_controller_driver->get_position(motor_channel);
      // get speed
      motor_data_[motion_controller_driver->get_motor_joint_name(motor_channel)].actual_speed =
          motion_controller_driver->get_speed(motor_channel);
    }
  }

  return ret_val;
}

void Cia402System::stop_all_motors()
{
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

  // check all motors if at least one is faulty
  bool faulty = false;
  for (auto it = drivers.begin(); it != drivers.end(); ++it)
  {
    auto motion_controller_driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(it->second);
    for (auto motor_channel : motion_controller_driver->get_available_motor_channels())
    {
      if (motion_controller_driver->is_motor_faulty(motor_channel))
      {
        // instantly stop all motors and recover faulty motor
        stop_all_motors();
        motion_controller_driver->recover_motor(motor_channel);
        faulty = true;
      }
    }
  }

  // at least one motor is faulty
  if (faulty)
  {
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
      const uint16_t& mode = motion_controller_driver->get_mode(motor_channel);
      switch (mode)
      {
        case MotorBase::No_Mode:
          break;
        case MotorBase::Profiled_Position:
        case MotorBase::Cyclic_Synchronous_Position:
        case MotorBase::Interpolated_Position:
          motion_controller_driver->set_target(
              motor_channel,
              motor_data_[motion_controller_driver->get_motor_joint_name(motor_channel)].target_position);
          break;
        case MotorBase::Profiled_Velocity:
        case MotorBase::Velocity:
        case MotorBase::Cyclic_Synchronous_Velocity:
          motion_controller_driver->set_target(
              motor_channel,
              motor_data_[motion_controller_driver->get_motor_joint_name(motor_channel)].target_velocity);
          break;
        case MotorBase::Profiled_Torque:
        case MotorBase::Cyclic_Synchronous_Torque:
          motion_controller_driver->set_target(
              motor_channel, motor_data_[motion_controller_driver->get_motor_joint_name(motor_channel)].target_torque);
          break;
        default:
          RCLCPP_INFO(kLogger, "Mode %u not supported", mode);
      }
    }
  }

  return hardware_interface::return_type::OK;
}
}  // namespace canopen_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(canopen_ros2_control::Cia402System, hardware_interface::SystemInterface)
