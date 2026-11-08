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

#include "itera_lio/sensor/sensor_buffer.hpp"

#include <stdexcept>
#include <utility>
#include <cmath>

namespace itera_lio::sensor
{

SensorBuffer::SensorBuffer(
  const std::size_t lidar_capacity, const std::size_t imu_capacity,
  const double imu_lookahead_sec)
: lidar_capacity_(lidar_capacity), imu_capacity_(imu_capacity),
  imu_lookahead_sec_(imu_lookahead_sec)
{
  if (lidar_capacity_ == 0U || imu_capacity_ == 0U ||
    !std::isfinite(imu_lookahead_sec_) || imu_lookahead_sec_ < 0.0)
  {
    throw std::invalid_argument("sensor buffer capacities must be positive");
  }
}

PushResult SensorBuffer::pushLidar(LidarFrame frame)
{
  const double timestamp = frame.scan_begin_time;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.lidar_received;
    if (last_lidar_timestamp_ >= 0.0 && timestamp < last_lidar_timestamp_) {
      ++stats_.lidar_timestamp_regressions;
      return PushResult::kRejectedTimestampRegression;
    }
    last_lidar_timestamp_ = timestamp;
    if (lidar_buffer_.size() >= lidar_capacity_) {
      lidar_buffer_.pop_front();
      ++stats_.lidar_capacity_drops;
    }
    lidar_buffer_.push_back(std::move(frame));
  }
  condition_.notify_one();
  return PushResult::kAccepted;
}

PushResult SensorBuffer::pushImu(ImuData sample)
{
  const double timestamp = sample.timestamp;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.imu_received;
    if (last_imu_timestamp_ >= 0.0 && timestamp < last_imu_timestamp_) {
      ++stats_.imu_timestamp_regressions;
      return PushResult::kRejectedTimestampRegression;
    }
    last_imu_timestamp_ = timestamp;
    latest_imu_timestamp_seen_ = timestamp;
    if (imu_buffer_.size() >= imu_capacity_) {
      imu_buffer_.pop_front();
      ++stats_.imu_capacity_drops;
    }
    imu_buffer_.push_back(std::move(sample));
  }
  condition_.notify_one();
  return PushResult::kAccepted;
}

bool SensorBuffer::waitAndPopNext(
  BufferedMeasurement & measurement,
  const std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  const auto has_ready_measurement = [this]() {
      const bool lidar_ready = !lidar_buffer_.empty() &&
        latest_imu_timestamp_seen_ >=
        lidar_buffer_.front().scan_end_time + imu_lookahead_sec_ &&
        (imu_buffer_.empty() ||
        lidar_buffer_.front().scan_begin_time <= imu_buffer_.front().timestamp);
      return stopped_ || lidar_ready || !imu_buffer_.empty();
    };
  const bool ready = condition_.wait_for(lock, timeout, has_ready_measurement);
  if (!ready || (stopped_ && lidar_buffer_.empty() && imu_buffer_.empty())) {
    return false;
  }

  const bool lidar_has_imu_lookahead = !lidar_buffer_.empty() &&
    latest_imu_timestamp_seen_ >=
    lidar_buffer_.front().scan_end_time + imu_lookahead_sec_;
  const bool take_lidar = lidar_has_imu_lookahead &&
    (imu_buffer_.empty() ||
    lidar_buffer_.front().scan_begin_time <= imu_buffer_.front().timestamp);

  if (!take_lidar && imu_buffer_.empty()) {
    return false;
  }

  if (take_lidar) {
    measurement.type = MeasurementType::kLidar;
    measurement.lidar = std::move(lidar_buffer_.front());
    lidar_buffer_.pop_front();
  } else {
    measurement.type = MeasurementType::kImu;
    measurement.imu = std::move(imu_buffer_.front());
    imu_buffer_.pop_front();
  }
  return true;
}

void SensorBuffer::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
  }
  condition_.notify_all();
}

SensorBufferStats SensorBuffer::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = stats_;
  result.lidar_buffer_size = lidar_buffer_.size();
  result.imu_buffer_size = imu_buffer_.size();
  return result;
}

}  // namespace itera_lio::sensor
