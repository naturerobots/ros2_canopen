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
  double target_;
  std::atomic<bool> has_target_;
  std::shared_ptr<LelyDriverBridge> driver;

public:
  TargetHelper(std::shared_ptr<LelyDriverBridge> driver)
  {
    this->driver = driver;
    has_target_ = false;
  }

  virtual bool write(uint8_t rollover)
  {
    if (has_target_) {
      // Is this the correct way to write the target in combination with the rollover counter?
      driver->universal_set_value<int32_t>(0x60FF, 0, this->target_);
      driver->universal_set_value<uint8_t>(0x382A, 0, rollover);
      RCLCPP_INFO(
        rclcpp::get_logger("canopen_402_target"),
        "Target command set to %f with rollover counter %d", this->target_, rollover);
      return true;
    } else {
      return false;
    }
  }

  virtual bool setTarget(const double & val)
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
      std::cout << "canopen_402 Command " << val
                << " does not fit into target, clamping to min limit" << std::endl;
      target_ = std::numeric_limits<double>::min();
    } catch (positive_overflow &) {
      std::cout << "canopen_402 Command " << val
                << " does not fit into target, clamping to max limit" << std::endl;
      target_ = std::numeric_limits<double>::max();
    } catch (...) {
      std::cout << "canopen_402 Was not able to cast command " << val << std::endl;
      return false;
    }

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
