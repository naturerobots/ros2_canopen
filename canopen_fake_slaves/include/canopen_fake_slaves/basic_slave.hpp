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

#ifndef BASIC_SLAVE_HPP
#define BASIC_SLAVE_HPP
#include <lely/coapp/slave.hpp>
#include <lely/ev/co_task.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>

#include <memory>
#include <thread>

#include "canopen_fake_slaves/base_slave.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace lely;
using namespace std::chrono_literals;
namespace ros2_canopen
{
class SimpleSlave : public canopen::BasicSlave
{
public:
  using BasicSlave::BasicSlave;

protected:
  /**
   * @brief This function is called when a value is written to the local object dictionary by an SDO
   * or RPDO. Also copies the RPDO value to TPDO.
   * @param idx The index of the PDO.
   * @param subidx The subindex of the PDO.
   */
  void OnWrite(uint16_t idx, uint8_t subidx) noexcept override
  {
    uint32_t val = (*this)[idx][subidx];
    (*this)[0x4001][0] = val;
    this->TpdoEvent(0);
  }
};

class BasicSlave : public BaseSlave
{
public:
  explicit BasicSlave(const std::string & node_name, bool intra_process_comms = false)
  : BaseSlave(node_name, intra_process_comms)
  {
  }

protected:
  class ActiveCheckTask : public ev::CoTask
  {
  public:
    ActiveCheckTask(io::Context * ctx, ev::Executor * exec, BasicSlave * slave) : CoTask(*exec)
    {
      slave_ = slave;
      exec_ = exec;
      ctx_ = ctx;
    }

  protected:
    BasicSlave * slave_;
    ev::Executor * exec_;
    io::Context * ctx_;
    virtual void operator()() noexcept
    {
      if (slave_->activated.load())
      {
      }
      ctx_->shutdown();
    }
  };

  void run() override
  {
    io::IoGuard io_guard;
    io::Context ctx;
    io::Poll poll(ctx);
    ev::Loop loop(poll.get_poll());
    auto exec = loop.get_executor();
    io::Timer timer(poll, exec, CLOCK_MONOTONIC);

    // Retry loop for CAN controller initialization
    std::unique_ptr<io::CanController> ctrl;
    int retry_count = 0;
    while (true)
    {
      try
      {
        ctrl = std::make_unique<io::CanController>(can_interface_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Successfully connected to CAN interface '%s'",
            can_interface_name_.c_str());
        break;
      }
      catch (const std::system_error & e)
      {
        retry_count++;
        if (retry_count > can_interface_retry_count_)
        {
          RCLCPP_ERROR(this->get_logger(),
              "Failed to connect to CAN interface '%s' after %d attempts: %s",
              can_interface_name_.c_str(), retry_count, e.what());
          return;
        }
        RCLCPP_WARN(this->get_logger(),
            "CAN interface '%s' not available (%s). Retrying in %d ms... (attempt %d/%d)",
            can_interface_name_.c_str(), e.what(), can_interface_retry_delay_ms_,
            retry_count, can_interface_retry_count_);
        std::this_thread::sleep_for(std::chrono::milliseconds(can_interface_retry_delay_ms_));
      }
    }

    io::CanChannel chan(poll, exec);
    chan.open(*ctrl);

    auto sigset_ = lely::io::SignalSet(poll, exec);
    // Watch for Ctrl+C or process termination.
    sigset_.insert(SIGHUP);
    sigset_.insert(SIGINT);
    sigset_.insert(SIGTERM);

    sigset_.submit_wait(
      [&](int /*signo*/)
      {
        // If the signal is raised again, terminate immediately.
        sigset_.clear();

        // Perform a clean shutdown.
        ctx.shutdown();
      });

    SimpleSlave slave(timer, chan, slave_config_.c_str(), "", node_id_);
    slave.Reset();
    ActiveCheckTask checktask(&ctx, &exec, this);

    // timer.submit_wait()
    RCLCPP_INFO(this->get_logger(), "Created slave for node_id %i.", node_id_);
    loop.run();
    ctx.shutdown();
    RCLCPP_INFO(this->get_logger(), "Stopped CANopen Event Loop.");
    rclcpp::shutdown();
  }
};
}  // namespace ros2_canopen

#endif
