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

#ifndef ITERA_LIO__SENSOR__SENSOR_BUFFER_HPP_
#define ITERA_LIO__SENSOR__SENSOR_BUFFER_HPP_

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

#include "itera_lio/common/measurement_types.hpp"
#include "itera_lio/sensor/imu_data.hpp"
#include "itera_lio/sensor/lidar_frame.hpp"

namespace itera_lio::sensor
{

enum class PushResult
{
  kAccepted,
  kRejectedTimestampRegression,
};

using MeasurementType = common::MeasurementType;
using BufferedMeasurement = common::Measurement;

struct SensorBufferStats
{
  std::uint64_t lidar_received{0};
  std::uint64_t imu_received{0};
  std::uint64_t lidar_timestamp_regressions{0};
  std::uint64_t imu_timestamp_regressions{0};
  std::uint64_t lidar_capacity_drops{0};
  std::uint64_t imu_capacity_drops{0};
  std::size_t lidar_buffer_size{0};
  std::size_t imu_buffer_size{0};
};

class SensorBuffer
{
public:
  SensorBuffer(
    std::size_t lidar_capacity, std::size_t imu_capacity,
    double imu_lookahead_sec = 0.0);

  PushResult pushLidar(LidarFrame frame);
  PushResult pushImu(ImuData sample);

  bool waitAndPopNext(
    BufferedMeasurement & measurement,
    std::chrono::milliseconds timeout);

  void stop();
  SensorBufferStats stats() const;

private:
  const std::size_t lidar_capacity_;
  const std::size_t imu_capacity_;
  const double imu_lookahead_sec_;

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<LidarFrame> lidar_buffer_;
  std::deque<ImuData> imu_buffer_;
  double last_lidar_timestamp_{-1.0};
  double last_imu_timestamp_{-1.0};
  double latest_imu_timestamp_seen_{-1.0};
  bool stopped_{false};
  SensorBufferStats stats_;
};

}  // namespace itera_lio::sensor

#endif  // ITERA_LIO__SENSOR__SENSOR_BUFFER_HPP_
