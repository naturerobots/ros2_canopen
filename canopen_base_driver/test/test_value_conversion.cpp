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

#include <gtest/gtest.h>

#include "canopen_base_driver/value_conversion.hpp"

using ros2_canopen::DriverException;
using ros2_canopen::ValueConversion;

TEST(ValueConversion, LinearByDefault)
{
  const ValueConversion conversion(YAML::Load("{}"), "position", 917.0, 1.0 / 917.0, 880.0);
  EXPECT_FALSE(conversion.is_table());
  EXPECT_DOUBLE_EQ(conversion.to_dev(0.0), 880.0);
  EXPECT_DOUBLE_EQ(conversion.to_dev(0.5), 880.0 + 458.5);
  EXPECT_NEAR(conversion.from_dev(880.0 - 917.0), -1.0, 1e-12);
}

TEST(ValueConversion, TableInterpolatesAndExtrapolates)
{
  // Same table both ways (YAML alias), unsorted on purpose, nonlinear, non-zero center.
  const ValueConversion conversion(YAML::Load("{position_conversion: table,"
                                              " position_table_to_dev: &t [[1600, 0.8], [160, -0.8], [880, 0.0], "
                                              "[1300, 0.5]],"
                                              " position_table_from_dev: *t}"),
                                   "position", 0.0, 0.0);
  EXPECT_TRUE(conversion.is_table());
  EXPECT_DOUBLE_EQ(conversion.to_dev(0.0), 880.0);
  EXPECT_DOUBLE_EQ(conversion.to_dev(0.25), 1090.0);
  EXPECT_DOUBLE_EQ(conversion.from_dev(1090.0), 0.25);
  EXPECT_DOUBLE_EQ(conversion.from_dev(1450.0), 0.65);
  EXPECT_DOUBLE_EQ(conversion.to_dev(-0.4), 520.0);
  EXPECT_NEAR(conversion.from_dev(1900.0), 1.1, 1e-9);  // outer segment [1300, 1600] extrapolated
  EXPECT_NEAR(conversion.to_dev(-1.0), -20.0, 1e-9);    // outer segment [160, 880] extrapolated
}

TEST(ValueConversion, SeparateTablesPerDirection)
{
  // Command and feedback with different resolution, e.g. from different sensors.
  const ValueConversion conversion(YAML::Load("{velocity_conversion: table,"
                                              " velocity_table_to_dev: [[-1000, -1.0], [1000, 1.0]],"
                                              " velocity_table_from_dev: [[-10, -1.0], [10, 1.0]]}"),
                                   "velocity", 0.0, 0.0);
  EXPECT_DOUBLE_EQ(conversion.to_dev(0.5), 500.0);
  EXPECT_DOUBLE_EQ(conversion.from_dev(5.0), 0.5);
}

TEST(ValueConversion, TableMayBeDecreasing)
{
  const ValueConversion conversion(YAML::Load("{velocity_conversion: table,"
                                              " velocity_table_to_dev: &t [[0, 1.0], [100, -1.0]],"
                                              " velocity_table_from_dev: *t}"),
                                   "velocity", 0.0, 0.0);
  EXPECT_DOUBLE_EQ(conversion.to_dev(0.0), 50.0);
  EXPECT_DOUBLE_EQ(conversion.from_dev(25.0), 0.5);
}

TEST(ValueConversion, RejectsInvalidConfig)
{
  const auto make = [](const std::string& yaml) { return ValueConversion(YAML::Load(yaml), "position", 1.0, 1.0); };
  EXPECT_THROW(ValueConversion(YAML::Load("{}"), "position", 0.0, 0.0), DriverException);
  EXPECT_THROW(make("{position_conversion: cubic}"), DriverException);
  EXPECT_THROW(make("{position_conversion: table}"), DriverException);
  // One direction missing.
  EXPECT_THROW(make("{position_conversion: table, position_table_to_dev: [[0, 0.0], [10, 1.0]]}"), DriverException);
  EXPECT_THROW(make("{position_conversion: table, position_table_from_dev: [[0, 0.0], [10, 1.0]]}"), DriverException);
  // Too few points, not monotonic, duplicate raw.
  EXPECT_THROW(make("{position_conversion: table, position_table_to_dev: [[0, 0.0]],"
                    " position_table_from_dev: [[0, 0.0], [10, 1.0]]}"),
               DriverException);
  EXPECT_THROW(make("{position_conversion: table, position_table_to_dev: [[0, 0.0], [10, 1.0]],"
                    " position_table_from_dev: [[0, 0.0], [10, 1.0], [20, 0.5]]}"),
               DriverException);
  EXPECT_THROW(make("{position_conversion: table, position_table_to_dev: [[0, 0.0], [0, 1.0]],"
                    " position_table_from_dev: [[0, 0.0], [10, 1.0]]}"),
               DriverException);
}
