// Copyright 2026 rainbroken
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

#ifndef ITERA_LIO__SENSOR__LIDAR_PARSER_HPP_
#define ITERA_LIO__SENSOR__LIDAR_PARSER_HPP_

#include <cstddef>

#include "itera_lio/sensor/lidar_frame.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace itera_lio::sensor
{

enum class LidarParseStatus
{
  kSuccess,
  kMissingField,
  kMalformedCloud,
  kNoValidPoints,
};

struct LidarParseReport
{
  LidarParseStatus status{LidarParseStatus::kMalformedCloud};
  std::size_t input_points{0U};
  std::size_t output_points{0U};
  std::size_t rejected_points{0U};
};

LidarParseReport parsePointCloud2(
  const sensor_msgs::msg::PointCloud2 & cloud,
  LidarFrame & frame);

}  // namespace itera_lio::sensor

#endif  // ITERA_LIO__SENSOR__LIDAR_PARSER_HPP_
