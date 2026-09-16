//    Copyright 2022 Christoph Hellmann Santos
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#include "canopen_core/driver_node.hpp"

#include <chrono>
#include <thread>

using namespace ros2_canopen;
void CanopenDriver::init()
{
  // Each phase is skipped if it already succeeded, so that a caller retrying init() after a
  // failure resumes at the phase that actually failed instead of tripping over the
  // "already initialised/configured" guards.
  if (!node_canopen_driver_->is_initialised())
  {
    node_canopen_driver_->init();
  }
  if (!node_canopen_driver_->is_configured())
  {
    node_canopen_driver_->configure();
  }
  if (!node_canopen_driver_->is_master_set())
  {
    bool success = false;
    for (int i = 0; i < 5; i++)
    {
      try
      {
        node_canopen_driver_->demand_set_master();
        success = true;
        break;
      }
      catch (std::exception & e)
      {
        RCLCPP_WARN(
          this->get_logger(), "Failed to get demand set master result because %s. Retrying.",
          e.what());
        // The container executor can be slow to answer while the rest of the stack is
        // still coming up; back off rather than burning all five tries in a millisecond.
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }
    if (!success)
    {
      RCLCPP_WARN(this->get_logger(), "Failed to get demand set master result. Exiting...");
      throw DriverException("Failed to get demand set master result. Exiting...");
    }
  }
  if (!node_canopen_driver_->is_activated())
  {
    node_canopen_driver_->activate();
  }
}

void CanopenDriver::shutdown() { node_canopen_driver_->shutdown(); }

void CanopenDriver::configure()
{
  // Note: init() already calls node_canopen_driver_->configure()
  // This is here for interface compliance but typically not called separately
}

void CanopenDriver::set_master(
  std::shared_ptr<lely::ev::Executor> exec, std::shared_ptr<lely::canopen::AsyncMaster> master)
{
  node_canopen_driver_->set_master(exec, master);
}

void LifecycleCanopenDriver::init() { node_canopen_driver_->init(); }

void LifecycleCanopenDriver::shutdown() { node_canopen_driver_->shutdown(); }

void LifecycleCanopenDriver::configure()
{
  // For lifecycle drivers, configure is called via on_configure callback
}

void LifecycleCanopenDriver::set_master(
  std::shared_ptr<lely::ev::Executor> exec, std::shared_ptr<lely::canopen::AsyncMaster> master)
{
  node_canopen_driver_->set_master(exec, master);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenDriver::on_configure(const rclcpp_lifecycle::State & state)
{
  node_canopen_driver_->configure();
  bool success = false;
  for (int i = 0; i < 5; i++)
  {
    try
    {
      node_canopen_driver_->demand_set_master();
      success = true;
      break;
    }
    catch (std::exception & e)
    {
      RCLCPP_WARN(
        this->get_logger(), "Failed to get demand set master result becauss %s. Retrying.",
        e.what());
    }
  }
  if (!success)
  {
    RCLCPP_WARN(this->get_logger(), "Failed to get demand set master result. Exiting...");
    throw DriverException("Failed to get demand set master result. Exiting...");
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenDriver::on_activate(const rclcpp_lifecycle::State & state)
{
  node_canopen_driver_->activate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenDriver::on_deactivate(const rclcpp_lifecycle::State & state)
{
  node_canopen_driver_->deactivate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenDriver::on_cleanup(const rclcpp_lifecycle::State & state)
{
  node_canopen_driver_->cleanup();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenDriver::on_shutdown(const rclcpp_lifecycle::State & state)
{
  node_canopen_driver_->shutdown();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
