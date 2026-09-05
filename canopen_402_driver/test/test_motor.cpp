// Copyright 2026 ROS-Industrial
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

#include "canopen_402_driver/motor.hpp"

#include <cstdint>

#include "gtest/gtest.h"

using namespace ros2_canopen;

TEST(Motor402, rejects_mode_queries_without_driver)
{
  Motor402 motor(std::shared_ptr<LelyDriverBridge>{}, State402::Operation_Enable, "test_joint", 1.0, 1.0, 1.0,
                 1.0, MotorBase::Profiled_Position, 1);

  EXPECT_FALSE(motor.isModeSupported(MotorBase::Profiled_Position));
  EXPECT_FALSE(motor.isModeSupported(MotorBase::No_Mode));
  EXPECT_FALSE(motor.isModeSupported(33));
  EXPECT_FALSE(motor.isModeSupported(UINT16_MAX));
}
