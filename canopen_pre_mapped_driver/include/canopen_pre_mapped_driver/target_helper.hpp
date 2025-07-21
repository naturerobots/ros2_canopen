//    Copyright 2023 Christoph Hellmann Santos
//    Copyright 2014-2022 Authors of ros_canopen
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

#ifndef TARGET_HELPER_HPP
#define TARGET_HELPER_HPP

#include <atomic>
#include <boost/numeric/conversion/cast.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "canopen_base_driver/lely_driver_bridge.hpp"

namespace ros2_canopen
{

class TargetHelper
{
  int32_t target_;
  uint8_t rollover_;  // rollover counter
  uint8_t start_after_;
  uint8_t target_counter_;
  std::atomic<bool> has_target_;
  std::shared_ptr<LelyDriverBridge> driver_;

public:
  TargetHelper(std::shared_ptr<LelyDriverBridge> driver)
  {
    this->driver_ = driver;
    rollover_ = 0;
    start_after_ = 32;  // start after 2 cycles
    target_counter_ = 0;
    has_target_ = false;
  }

  virtual bool write()
  {
    if (has_target_ && this->driver_) {
      // Is this the correct way to write the target in combination with the rollover counter?
      driver_->universal_set_value<int32_t>(0x60FF, 0, this->target_);
      driver_->universal_set_value<uint8_t>(0x382A, 0, this->rollover_);
      // RCLCPP_INFO(
      //   rclcpp::get_logger("canopen_402_target"),
      //   "Target command set to %f with rollover counter %d", this->target_, this->rollover_);
      rollover_++;
      return true;
    } else {
      return false;
    }
  }

  virtual bool setTarget(const double & val, uint8_t rollover)
  {
    if (std::isnan(val)) {
      RCLCPP_ERROR(rclcpp::get_logger("canopen_402_target"), "Target command is not a number");
      return false;
    }

    using boost::numeric_cast;
    using boost::numeric::negative_overflow;
    using boost::numeric::positive_overflow;
    try {
      target_ = numeric_cast<int32_t>(val);
    } catch (negative_overflow &) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("canopen_402_target"),
        "Target command " << val << " does not fit into target, clamping to min limit");
      target_ = std::numeric_limits<double>::min();
    } catch (positive_overflow &) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("canopen_402_target"),
        "Target command " << val << " does not fit into target, clamping to max limit");
      target_ = std::numeric_limits<double>::max();
    } catch (...) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("canopen_402_target"),
        "Was not able to cast command " << val);
      return false;
    }
    if (target_counter_ < start_after_) {
      target_counter_++;
      target_ = 0;  // do not send target command before start_after_ cycles
    }

    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("canopen_402_target"),
      "Setting target command to " << target_);
    // rollover_ = rollover;
    has_target_ = true;
    return true;
  }

  virtual bool start()
  {
    has_target_ = false;
    return true;
  }
};
}  // namespace ros2_canopen

#endif  // TARGET_HELPER_HPP
