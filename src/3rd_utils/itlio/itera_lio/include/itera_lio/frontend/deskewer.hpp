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

#ifndef ITERA_LIO__FRONTEND__DESKEWER_HPP_
#define ITERA_LIO__FRONTEND__DESKEWER_HPP_

#include <Eigen/Core>

#include <cstddef>
#include <deque>
#include <vector>

#include "itera_lio/common/geometry.hpp"
#include "itera_lio/sensor/lidar_frame.hpp"

namespace itera_lio::frontend
{

using ImuPose = common::ImuPose;

bool deskewLidarFrame(
  const sensor::LidarFrame & input,
  const std::deque<ImuPose> & trajectory,
  const common::FixedExtrinsics & extrinsics,
  sensor::LidarFrame & output, int num_threads = 1);

// Deskews every input point once and gathers the selected points in their
// original order for the scan-matching path.
bool deskewLidarFrameAndSelect(
  const sensor::LidarFrame & input,
  const std::vector<std::size_t> & selected_indices,
  const std::deque<ImuPose> & trajectory,
  const common::FixedExtrinsics & extrinsics,
  sensor::LidarFrame & full_output,
  sensor::LidarFrame & selected_output, int num_threads = 1);

}  // namespace itera_lio::frontend

#endif  // ITERA_LIO__FRONTEND__DESKEWER_HPP_
