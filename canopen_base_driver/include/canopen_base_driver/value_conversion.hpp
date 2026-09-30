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

#ifndef CANOPEN_BASE_DRIVER__VALUE_CONVERSION_HPP_
#define CANOPEN_BASE_DRIVER__VALUE_CONVERSION_HPP_

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "canopen_core/driver_error.hpp"

namespace ros2_canopen
{

/**
 * @brief Converts a joint value (position, velocity, ...) to and from the raw device value.
 *
 * For a value named <name> (e.g. "position" or "velocity"), selected by the bus.yml key
 * `<name>_conversion`:
 *  - `linear` (default): raw = offset + value * scale_to_dev, value = (raw - offset) * scale_from_dev.
 *  - `table`: piecewise linear interpolation, e.g. for a linear actuator driving a steering linkage.
 *    Like the two linear scales, each direction has its own table (e.g. command and feedback from
 *    different sensors): `<name>_table_to_dev` for value -> raw and `<name>_table_from_dev` for
 *    raw -> value, both as `[[raw, value], ...]`. The points may be listed in any order but must be
 *    strictly monotonic in both columns. Outside a table its outer segments are extrapolated.
 */
class ValueConversion
{
public:
  ValueConversion() = default;

  ValueConversion(const YAML::Node& config, const std::string& name, double scale_to_dev, double scale_from_dev,
                  double offset = 0.0)
    : scale_to_dev_(scale_to_dev), scale_from_dev_(scale_from_dev), offset_(offset)
  {
    const std::string mode_key = name + "_conversion";
    const std::string mode = config[mode_key] ? config[mode_key].as<std::string>() : "linear";
    if (mode == "linear")
    {
      if (scale_to_dev_ == 0.0 || scale_from_dev_ == 0.0)
      {
        throw DriverException(mode_key + " linear needs non-zero " + name + " scales");
      }
      return;
    }
    if (mode != "table")
    {
      throw DriverException(mode_key + " must be 'linear' or 'table', got '" + mode + "'");
    }

    raw_to_value_ = read_table(config, name + "_table_from_dev", name, false);
    value_to_raw_ = read_table(config, name + "_table_to_dev", name, true);
  }

  double to_dev(double value) const
  {
    return is_table() ? interpolate(value_to_raw_, value) : offset_ + value * scale_to_dev_;
  }

  double from_dev(double raw) const
  {
    return is_table() ? interpolate(raw_to_value_, raw) : (raw - offset_) * scale_from_dev_;
  }

  bool is_table() const
  {
    return !raw_to_value_.empty();
  }

private:
  // Reads [[raw, value], ...] and returns it as (x, y) points sorted by x, with x = value if swap, else raw.
  static std::vector<std::pair<double, double>> read_table(const YAML::Node& config, const std::string& key,
                                                           const std::string& name, bool swap)
  {
    const YAML::Node table = config[key];
    if (!table || !table.IsSequence() || table.size() < 2)
    {
      throw DriverException(name + "_conversion table needs " + key + " [[raw, " + name + "], ...] with >= 2 points");
    }
    std::vector<std::pair<double, double>> points;
    for (const auto& point : table)
    {
      if (!point.IsSequence() || point.size() != 2)
      {
        throw DriverException(key + " points must be [raw, " + name + "]");
      }
      const double raw = point[0].as<double>();
      const double value = point[1].as<double>();
      points.emplace_back(swap ? value : raw, swap ? raw : value);
    }
    std::sort(points.begin(), points.end());
    const double direction = points[1].second - points[0].second;
    for (size_t i = 1; i < points.size(); ++i)
    {
      const double dx = points[i].first - points[i - 1].first;
      const double dy = points[i].second - points[i - 1].second;
      if (dx <= 0.0 || dy * direction <= 0.0)
      {
        throw DriverException(key + " must be strictly monotonic in raw and " + name);
      }
    }
    return points;
  }

  // Piecewise linear over points sorted by x, extrapolating with the outer segments.
  static double interpolate(const std::vector<std::pair<double, double>>& points, double x)
  {
    auto upper = std::upper_bound(points.begin(), points.end(), x,
                                  [](double v, const auto& point) { return v < point.first; });
    upper = std::clamp(upper, points.begin() + 1, points.end() - 1);
    const auto& [x0, y0] = *(upper - 1);
    const auto& [x1, y1] = *upper;
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
  }

  double scale_to_dev_ = 1.0;
  double scale_from_dev_ = 1.0;
  double offset_ = 0.0;
  std::vector<std::pair<double, double>> raw_to_value_;  // <name>_table_from_dev, sorted by raw
  std::vector<std::pair<double, double>> value_to_raw_;  // <name>_table_to_dev swapped, sorted by value
};

}  // namespace ros2_canopen

#endif  // CANOPEN_BASE_DRIVER__VALUE_CONVERSION_HPP_
