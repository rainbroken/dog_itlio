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

#include <chrono>
#include <stdexcept>

#include "gtest/gtest.h"
#include "itera_lio/sensor/sensor_buffer.hpp"

namespace
{

itera_lio::sensor::ImuData makeImu(const double seconds)
{
  itera_lio::sensor::ImuData sample;
  sample.timestamp = seconds;
  return sample;
}

itera_lio::sensor::LidarFrame makeLidar(const double seconds)
{
  itera_lio::sensor::LidarFrame frame;
  frame.scan_begin_time = seconds;
  frame.scan_end_time = seconds;
  frame.points.push_back({});
  return frame;
}

TEST(SensorBuffer, RejectsNonPositiveCapacity)
{
  EXPECT_THROW(itera_lio::sensor::SensorBuffer(0, 1), std::invalid_argument);
  EXPECT_THROW(itera_lio::sensor::SensorBuffer(1, 0), std::invalid_argument);
}

TEST(SensorBuffer, RejectsTimestampRegression)
{
  itera_lio::sensor::SensorBuffer buffer(2, 2);
  EXPECT_EQ(buffer.pushImu(makeImu(2)), itera_lio::sensor::PushResult::kAccepted);
  EXPECT_EQ(
    buffer.pushImu(makeImu(1)),
    itera_lio::sensor::PushResult::kRejectedTimestampRegression);

  const auto stats = buffer.stats();
  EXPECT_EQ(stats.imu_received, 2U);
  EXPECT_EQ(stats.imu_timestamp_regressions, 1U);
  EXPECT_EQ(stats.imu_buffer_size, 1U);
}

TEST(SensorBuffer, DropsOldestMessageAtCapacity)
{
  itera_lio::sensor::SensorBuffer buffer(1, 2, 0.0);
  EXPECT_EQ(buffer.pushLidar(makeLidar(1)), itera_lio::sensor::PushResult::kAccepted);
  EXPECT_EQ(buffer.pushLidar(makeLidar(2)), itera_lio::sensor::PushResult::kAccepted);
  EXPECT_EQ(buffer.pushImu(makeImu(2)), itera_lio::sensor::PushResult::kAccepted);

  itera_lio::sensor::BufferedMeasurement measurement;
  ASSERT_TRUE(buffer.waitAndPopNext(measurement, std::chrono::milliseconds(1)));
  EXPECT_DOUBLE_EQ(measurement.lidar.scan_begin_time, 2.0);
  EXPECT_EQ(buffer.stats().lidar_capacity_drops, 1U);
}

TEST(SensorBuffer, PopsMessagesInTimestampOrder)
{
  itera_lio::sensor::SensorBuffer buffer(2, 2, 0.0);
  EXPECT_EQ(buffer.pushLidar(makeLidar(2)), itera_lio::sensor::PushResult::kAccepted);
  EXPECT_EQ(buffer.pushImu(makeImu(1)), itera_lio::sensor::PushResult::kAccepted);

  itera_lio::sensor::BufferedMeasurement first;
  ASSERT_TRUE(buffer.waitAndPopNext(first, std::chrono::milliseconds(1)));
  EXPECT_EQ(first.type, itera_lio::sensor::MeasurementType::kImu);

  itera_lio::sensor::BufferedMeasurement second;
  ASSERT_EQ(buffer.pushImu(makeImu(2)), itera_lio::sensor::PushResult::kAccepted);
  ASSERT_TRUE(buffer.waitAndPopNext(second, std::chrono::milliseconds(1)));
  EXPECT_EQ(second.type, itera_lio::sensor::MeasurementType::kLidar);
}

TEST(SensorBuffer, HoldsLidarUntilImuLookaheadIsAvailable)
{
  itera_lio::sensor::SensorBuffer buffer(2, 10, 0.005);
  auto lidar = makeLidar(1.0);
  lidar.scan_end_time = 1.1;
  ASSERT_EQ(buffer.pushLidar(std::move(lidar)), itera_lio::sensor::PushResult::kAccepted);
  ASSERT_EQ(buffer.pushImu(makeImu(1.102)), itera_lio::sensor::PushResult::kAccepted);

  itera_lio::sensor::BufferedMeasurement measurement;
  ASSERT_TRUE(buffer.waitAndPopNext(measurement, std::chrono::milliseconds(1)));
  EXPECT_EQ(measurement.type, itera_lio::sensor::MeasurementType::kImu);
  EXPECT_DOUBLE_EQ(measurement.imu.timestamp, 1.102);

  ASSERT_EQ(buffer.pushImu(makeImu(1.105)), itera_lio::sensor::PushResult::kAccepted);
  ASSERT_TRUE(buffer.waitAndPopNext(measurement, std::chrono::milliseconds(1)));
  EXPECT_EQ(measurement.type, itera_lio::sensor::MeasurementType::kLidar);
  EXPECT_DOUBLE_EQ(measurement.lidar.scan_end_time, 1.1);
}

}  // namespace
