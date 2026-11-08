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

#include "itera_lio/sensor/lidar_parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "sensor_msgs/msg/point_field.hpp"

namespace itera_lio::sensor
{
namespace
{

const sensor_msgs::msg::PointField * findField(
  const sensor_msgs::msg::PointCloud2 & cloud, const char * name)
{
  const auto field = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [name](const auto & value) {return value.name == name;});
  return field == cloud.fields.end() ? nullptr : &(*field);
}

template<typename T>
bool readValue(const std::uint8_t * data, std::size_t size, std::size_t offset, T & value)
{
  if (offset > size || sizeof(T) > size - offset) {
    return false;
  }
  std::memcpy(&value, data + offset, sizeof(T));
  return true;
}

bool readScalar(
  const std::uint8_t * data, std::size_t size,
  const sensor_msgs::msg::PointField & field, double & value)
{
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::FLOAT32: {
        float raw;
        if (!readValue(data, size, field.offset, raw)) {return false;}
        value = raw;
        return true;
      }
    case sensor_msgs::msg::PointField::FLOAT64:
      return readValue(data, size, field.offset, value);
    case sensor_msgs::msg::PointField::UINT8: {
        std::uint8_t raw;
        if (!readValue(data, size, field.offset, raw)) {return false;}
        value = raw;
        return true;
      }
    case sensor_msgs::msg::PointField::UINT16: {
        std::uint16_t raw;
        if (!readValue(data, size, field.offset, raw)) {return false;}
        value = raw;
        return true;
      }
    case sensor_msgs::msg::PointField::UINT32: {
        std::uint32_t raw;
        if (!readValue(data, size, field.offset, raw)) {return false;}
        value = raw;
        return true;
      }
    default:
      return false;
  }
}

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace

LidarParseReport parsePointCloud2(
  const sensor_msgs::msg::PointCloud2 & cloud,
  LidarFrame & frame)
{
  LidarParseReport report;
  frame = LidarFrame{};
  const std::size_t point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
  report.input_points = point_count;
  const auto * x_field = findField(cloud, "x");
  const auto * y_field = findField(cloud, "y");
  const auto * z_field = findField(cloud, "z");
  const auto * time_field = findField(cloud, "offset_time");
  // Livox uses `reflectivity`; accept the conventional `intensity` name too.
  const sensor_msgs::msg::PointField * intensity_field = findField(cloud, "reflectivity");
  if (intensity_field == nullptr) {
    intensity_field = findField(cloud, "intensity");
  }
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr || time_field == nullptr) {
    report.status = LidarParseStatus::kMissingField;
    return report;
  }
  if (cloud.is_bigendian || cloud.point_step == 0U ||
    cloud.row_step < cloud.point_step * cloud.width ||
    (point_count > 0U &&
    cloud.data.size() < static_cast<std::size_t>(cloud.row_step) * cloud.height))
  {
    report.status = LidarParseStatus::kMalformedCloud;
    return report;
  }

  frame.scan_begin_time = stampSeconds(cloud.header.stamp);
  frame.points.reserve(point_count);
  double max_relative_time = 0.0;
  for (std::size_t row = 0U; row < cloud.height; ++row) {
    for (std::size_t column = 0U; column < cloud.width; ++column) {
      const std::size_t base = row * cloud.row_step + column * cloud.point_step;
      if (base > cloud.data.size() || cloud.point_step > cloud.data.size() - base) {
        ++report.rejected_points;
        continue;
      }
      const std::uint8_t * point_data = cloud.data.data() + base;
      const std::size_t point_size = cloud.point_step;
      double x;
      double y;
      double z;
      double offset_time;
      if (!readScalar(point_data, point_size, *x_field, x) ||
        !readScalar(point_data, point_size, *y_field, y) ||
        !readScalar(point_data, point_size, *z_field, z) ||
        !readScalar(point_data, point_size, *time_field, offset_time))
      {
        ++report.rejected_points;
        continue;
      }
      const double relative_time = offset_time * 1e-9;
      LidarPoint point;
      point.position = Eigen::Vector3d(x, y, z);
      if (intensity_field != nullptr) {
        double intensity;
        if (!readScalar(point_data, point_size, *intensity_field, intensity) ||
          !std::isfinite(intensity))
        {
          ++report.rejected_points;
          continue;
        }
        point.intensity = static_cast<float>(intensity);
      }
      point.relative_time = relative_time;
      if (!point.isFinite()) {
        ++report.rejected_points;
        continue;
      }
      max_relative_time = std::max(max_relative_time, relative_time);
      frame.points.push_back(point);
    }
  }
  report.output_points = frame.points.size();
  if (frame.points.empty()) {
    report.status = LidarParseStatus::kNoValidPoints;
    return report;
  }
  frame.scan_end_time = frame.scan_begin_time + max_relative_time;
  report.status = LidarParseStatus::kSuccess;
  return report;
}

}  // namespace itera_lio::sensor
