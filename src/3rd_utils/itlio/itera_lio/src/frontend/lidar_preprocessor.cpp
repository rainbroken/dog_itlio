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

#include "itera_lio/frontend/lidar_preprocessor.hpp"
#include "itera_lio/common/voxel_visit_set.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace itera_lio::frontend
{
sensor::LidarFrame voxelSampleLidarFrame(
  const sensor::LidarFrame & input, const float voxel_size)
{
  sensor::LidarFrame output;
  output.scan_begin_time = input.scan_begin_time;
  output.scan_end_time = input.scan_end_time;
  // The index selector performs the complete input validation before returning
  // any index; repeating that full traversal here adds no protection.
  const auto selected_indices = voxelSelectLidarPointIndices(input, voxel_size);
  output.points.reserve(selected_indices.size());
  for (const auto index : selected_indices) {
    output.points.push_back(input.points[index]);
  }
  return output;
}

std::vector<std::size_t> voxelSelectLidarPointIndices(
  const sensor::LidarFrame & input, const float voxel_size)
{
  std::vector<std::size_t> selected_indices;
  voxelSelectLidarPointIndices(input, voxel_size, selected_indices);
  return selected_indices;
}

void voxelSelectLidarPointIndices(
  const sensor::LidarFrame & input, const float voxel_size,
  std::vector<std::size_t> & selected_indices)
{
  selected_indices.clear();
  if (!input.isValid() || !std::isfinite(voxel_size) || voxel_size <= 0.0F) {
    return;
  }
  voxelSelectLidarPointIndicesFromValidatedFrame(input, voxel_size, selected_indices);
}

void voxelSelectLidarPointIndicesFromValidatedFrame(
  const sensor::LidarFrame & input, const float voxel_size,
  std::vector<std::size_t> & selected_indices)
{
  selected_indices.clear();
  if (input.points.empty() || !std::isfinite(voxel_size) || voxel_size <= 0.0F) {
    return;
  }
  thread_local common::VoxelVisitSet sampled;
  sampled.reset(input.points.size());
  selected_indices.reserve(input.points.size());
  for (std::size_t index = 0U; index < input.points.size(); ++index) {
    const auto & point = input.points[index];
    const common::VoxelKey key{
      static_cast<int>(std::floor(point.position.x() / voxel_size)),
      static_cast<int>(std::floor(point.position.y() / voxel_size)),
      static_cast<int>(std::floor(point.position.z() / voxel_size))};
    if (sampled.insert(key)) {
      selected_indices.push_back(index);
    }
  }
}

sensor::LidarFrame sampleLidarFrameByStride(
  const sensor::LidarFrame & input, const std::size_t point_filter_num)
{
  sensor::LidarFrame output;
  sampleLidarFrameByStride(input, point_filter_num, output);
  return output;
}

void sampleLidarFrameByStride(
  const sensor::LidarFrame & input, const std::size_t point_filter_num,
  sensor::LidarFrame & output)
{
  if (&input == &output) {
    output = sampleLidarFrameByStride(input, point_filter_num);
    return;
  }
  output.points.clear();
  output.scan_begin_time = input.scan_begin_time;
  output.scan_end_time = input.scan_end_time;
  if (!input.isValid() || point_filter_num == 0U) {
    return;
  }
  output.points.reserve(1U + (input.points.size() - 1U) / point_filter_num);
  for (std::size_t index = 0U; index < input.points.size();) {
    output.points.push_back(input.points[index]);
    // Subtract before advancing so even SIZE_MAX is a valid stride.
    if (point_filter_num >= input.points.size() - index) {break;}
    index += point_filter_num;
  }
}

sensor::LidarFrame filterLidarFrameByRange(
  const sensor::LidarFrame & input, const double min_range)
{
  sensor::LidarFrame output;
  filterLidarFrameByRange(input, min_range, output);
  return output;
}

void filterLidarFrameByRange(
  const sensor::LidarFrame & input, const double min_range, sensor::LidarFrame & output)
{
  if (&input == &output) {
    output = filterLidarFrameByRange(input, min_range);
    return;
  }
  output.points.clear();
  output.scan_begin_time = input.scan_begin_time;
  output.scan_end_time = input.scan_end_time;
  if (!std::isfinite(input.scan_begin_time) || !std::isfinite(input.scan_end_time) ||
    input.scan_end_time < input.scan_begin_time || input.points.empty() ||
    !std::isfinite(min_range) || min_range < 0.0)
  {
    return;
  }
  if (min_range == 0.0) {
    if (input.isValid()) {output = input;}
    return;
  }

  output.points.reserve(input.points.size());
  const double cutoff_squared = min_range * min_range;
  constexpr double kSafeCoordinate = 1e150;
  constexpr double kMinimumNormalRange = 1e-150;
  constexpr double kRoundoffMargin = 32.0 * std::numeric_limits<double>::epsilon();
  for (const auto & point : input.points) {
    // Preserve the all-or-nothing frame validation, including a late bad
    // point, while sharing its input traversal with range filtering.
    if (!point.isFinite() ||
      input.scan_begin_time + point.relative_time > input.scan_end_time + 1e-9)
    {
      output.points.clear();
      return;
    }
    const double x = point.position.x();
    const double y = point.position.y();
    const double z = point.position.z();
    // Outside this range use the overflow/underflow-safe original expression.
    // The uncertainty band also retains the original decision at the cutoff.
    if (min_range >= kMinimumNormalRange && min_range <= kSafeCoordinate &&
      std::abs(x) <= kSafeCoordinate && std::abs(y) <= kSafeCoordinate &&
      std::abs(z) <= kSafeCoordinate)
    {
      const double squared = x * x + y * y + z * z;
      if (std::abs(squared - cutoff_squared) > kRoundoffMargin * cutoff_squared) {
        if (squared > cutoff_squared) {output.points.push_back(point);}
        continue;
      }
    }
    const double range = std::hypot(
      std::hypot(point.position.x(), point.position.y()), point.position.z());
    if (std::isfinite(range) && range >= min_range) {
      output.points.push_back(point);
    }
  }
}

}  // namespace itera_lio::frontend
