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

#ifndef ITERA_LIO__SENSOR__IMU_DATA_HPP_
#define ITERA_LIO__SENSOR__IMU_DATA_HPP_

#include <Eigen/Core>

#include <cmath>

namespace itera_lio::sensor
{

struct ImuData
{
  double timestamp{0.0};
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};

  bool isFinite() const
  {
    return std::isfinite(timestamp) && gyro.allFinite() && accel.allFinite();
  }
};

// Align an IMU timestamp to the LiDAR time axis. The offset follows the
// FAST-LIO convention: t_imu_aligned = t_imu_raw - lidar_to_imu_offset.
inline ImuData applyTimeOffset(const ImuData & sample, const double lidar_to_imu_offset)
{
  ImuData aligned = sample;
  if (std::isfinite(lidar_to_imu_offset)) {
    aligned.timestamp -= lidar_to_imu_offset;
  }
  return aligned;
}

}  // namespace itera_lio::sensor

#endif  // ITERA_LIO__SENSOR__IMU_DATA_HPP_
