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

#include <gtest/gtest.h>

#include <cstring>
#include <memory>

#include "itera_lio/sensor/lidar_parser.hpp"

namespace
{

sensor_msgs::msg::PointCloud2 makeCloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp.sec = 10;
  cloud.width = 2U;
  cloud.height = 1U;
  cloud.point_step = 20U;
  cloud.row_step = 40U;
  auto field = [](const char * name, std::uint32_t offset, std::uint8_t datatype) {
      sensor_msgs::msg::PointField result;
      result.name = name;
      result.offset = offset;
      result.datatype = datatype;
      result.count = 1U;
      return result;
    };
  cloud.fields = {
    field("x", 0U, sensor_msgs::msg::PointField::FLOAT32),
    field("y", 4U, sensor_msgs::msg::PointField::FLOAT32),
    field("z", 8U, sensor_msgs::msg::PointField::FLOAT32),
    field("reflectivity", 12U, sensor_msgs::msg::PointField::UINT8),
    field("offset_time", 16U, sensor_msgs::msg::PointField::UINT32)};
  cloud.data.assign(40U, 0U);
  const float x0 = 1.0F;
  const float y0 = 2.0F;
  const float z0 = 3.0F;
  const std::uint32_t t0 = 0U;
  const float x1 = 4.0F;
  const float y1 = 5.0F;
  const float z1 = 6.0F;
  const std::uint32_t t1 = 1000000U;
  const std::uint8_t intensity0 = 5U;
  const std::uint8_t intensity1 = 159U;
  std::memcpy(cloud.data.data(), &x0, sizeof(x0));
  std::memcpy(cloud.data.data() + 4U, &y0, sizeof(y0));
  std::memcpy(cloud.data.data() + 8U, &z0, sizeof(z0));
  std::memcpy(cloud.data.data() + 12U, &intensity0, sizeof(intensity0));
  std::memcpy(cloud.data.data() + 16U, &t0, sizeof(t0));
  std::memcpy(cloud.data.data() + 20U, &x1, sizeof(x1));
  std::memcpy(cloud.data.data() + 24U, &y1, sizeof(y1));
  std::memcpy(cloud.data.data() + 28U, &z1, sizeof(z1));
  std::memcpy(cloud.data.data() + 32U, &intensity1, sizeof(intensity1));
  std::memcpy(cloud.data.data() + 36U, &t1, sizeof(t1));
  return cloud;
}

TEST(LidarParser, ConvertsOffsetTimeToRelativeSeconds)
{
  const auto cloud = makeCloud();
  itera_lio::sensor::LidarFrame frame;
  const auto report = itera_lio::sensor::parsePointCloud2(cloud, frame);
  EXPECT_EQ(report.status, itera_lio::sensor::LidarParseStatus::kSuccess);
  ASSERT_EQ(frame.points.size(), 2U);
  EXPECT_DOUBLE_EQ(frame.scan_begin_time, 10.0);
  EXPECT_DOUBLE_EQ(frame.scan_end_time, 10.001);
  EXPECT_DOUBLE_EQ(frame.points[1].relative_time, 0.001);
  EXPECT_FLOAT_EQ(frame.points[0].intensity, 5.0F);
  EXPECT_FLOAT_EQ(frame.points[1].intensity, 159.0F);
}

TEST(LidarParser, RejectsMissingOffsetTime)
{
  auto cloud = makeCloud();
  cloud.fields.pop_back();
  itera_lio::sensor::LidarFrame frame;
  EXPECT_EQ(
    itera_lio::sensor::parsePointCloud2(cloud, frame).status,
    itera_lio::sensor::LidarParseStatus::kMissingField);
}

}  // namespace
