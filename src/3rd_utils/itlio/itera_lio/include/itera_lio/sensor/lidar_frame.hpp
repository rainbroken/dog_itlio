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

#ifndef ITERA_LIO__SENSOR__LIDAR_FRAME_HPP_
#define ITERA_LIO__SENSOR__LIDAR_FRAME_HPP_

#include <Eigen/Core>

#include <cmath>
#include <vector>

namespace itera_lio::sensor
{

struct LidarPoint
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  float intensity{0.0F};
  double relative_time{0.0};

  bool isFinite() const
  {
    return position.allFinite() && std::isfinite(static_cast<double>(intensity)) &&
           std::isfinite(relative_time) && relative_time >= 0.0;
  }
};

struct LidarFrame
{
  double scan_begin_time{0.0};
  double scan_end_time{0.0};
  std::vector<LidarPoint> points;

  // Host steady-clock metadata only; never used for estimation or sensor time.
  double callback_start_steady_sec{0.0};
  double input_callback_ms{0.0};

  bool isValid() const
  {
    if (!std::isfinite(scan_begin_time) || !std::isfinite(scan_end_time) ||
      scan_end_time < scan_begin_time || points.empty())
    {
      return false;
    }
    for (const auto & point : points) {
      if (!point.isFinite() || scan_begin_time + point.relative_time > scan_end_time + 1e-9) {
        return false;
      }
    }
    return true;
  }
};

}  // namespace itera_lio::sensor

#endif  // ITERA_LIO__SENSOR__LIDAR_FRAME_HPP_
