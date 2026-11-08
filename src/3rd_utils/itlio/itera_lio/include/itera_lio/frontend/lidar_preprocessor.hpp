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

#ifndef ITERA_LIO__FRONTEND__LIDAR_PREPROCESSOR_HPP_
#define ITERA_LIO__FRONTEND__LIDAR_PREPROCESSOR_HPP_

#include <cstddef>
#include <vector>

#include "itera_lio/sensor/lidar_frame.hpp"

namespace itera_lio::frontend
{

sensor::LidarFrame voxelSampleLidarFrame(
  const sensor::LidarFrame & input, float voxel_size);

std::vector<std::size_t> voxelSelectLidarPointIndices(
  const sensor::LidarFrame & input, float voxel_size);

// Reuse caller-owned index storage; invalid input clears previous indices.
void voxelSelectLidarPointIndices(
  const sensor::LidarFrame & input, float voxel_size,
  std::vector<std::size_t> & selected_indices);

// Precondition: input is a non-empty frame fully validated at an upstream boundary.
// Intended for output from filterLidarFrameByRange; performs no full-frame validation.
void voxelSelectLidarPointIndicesFromValidatedFrame(
  const sensor::LidarFrame & input, float voxel_size,
  std::vector<std::size_t> & selected_indices);

// Keep input points whose original order is a multiple of point_filter_num.
// A value of one keeps every point, matching FAST-LIO2 point_filter_num.
sensor::LidarFrame sampleLidarFrameByStride(
  const sensor::LidarFrame & input, std::size_t point_filter_num);

void sampleLidarFrameByStride(
  const sensor::LidarFrame & input, std::size_t point_filter_num,
  sensor::LidarFrame & output);

// Range is measured in the raw LiDAR frame before deskewing. Zero disables it.
sensor::LidarFrame filterLidarFrameByRange(
  const sensor::LidarFrame & input, double min_range);

// Reuse caller-owned storage. In-place calls are supported via a temporary.
void filterLidarFrameByRange(
  const sensor::LidarFrame & input, double min_range, sensor::LidarFrame & output);

}  // namespace itera_lio::frontend

#endif  // ITERA_LIO__FRONTEND__LIDAR_PREPROCESSOR_HPP_
