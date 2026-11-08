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

#include "itera_lio/loop/submap_builder.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "itera_lio/common/voxel.hpp"

namespace itera_lio::loop
{

std::vector<common::MapPoint> buildLoopSubmap(
  const KeyframeCloudStore & store, const common::KeyframeId matched_id,
  const common::KeyframeId current_id, const std::size_t half_window,
  const std::size_t min_separation)
{
  std::vector<common::MapPoint> points;
  KeyframeCloud matched;
  if (!store.get(matched_id, matched) || current_id <= matched_id ||
    current_id - matched_id < min_separation)
  {
    return points;
  }
  const auto & T_W_M = matched.keyframe.pose_WI;
  const std::size_t first = matched_id > half_window ? matched_id - half_window : 0U;
  const std::size_t last = std::min(
    store.size() - 1U, matched_id +
    std::min(half_window, store.size() - 1U - matched_id));
  for (std::size_t id = first; id <= last; ++id) {
    if (id >= current_id || current_id - id < min_separation) {continue;}
    KeyframeCloud cloud;
    if (!store.get(id, cloud)) {continue;}
    const auto & T_W_I = cloud.keyframe.pose_WI;
    for (const auto & point : cloud.points_I) {
      points.push_back(
        {(T_W_M.R_WI.transpose() *
          (T_W_I.R_WI * point.position.cast<double>() + T_W_I.p_WI - T_W_M.p_WI))
          .cast<float>(), point.intensity});
    }
  }
  return points;
}

std::vector<common::MapPoint> buildCurrentLoopSubmap(
  const KeyframeCloudStore & store, const common::KeyframeId current_id,
  const std::size_t half_window)
{
  std::vector<common::MapPoint> points;
  KeyframeCloud current;
  if (!store.get(current_id, current)) {return points;}
  const auto & T_W_C = current.keyframe.pose_WI;
  const std::size_t first = current_id > half_window ? current_id - half_window : 0U;
  for (std::size_t id = first; id <= current_id; ++id) {
    KeyframeCloud cloud;
    if (!store.get(id, cloud)) {continue;}
    const auto & T_W_I = cloud.keyframe.pose_WI;
    for (const auto & point : cloud.points_I) {
      points.push_back(
        {(T_W_C.R_WI.transpose() *
          (T_W_I.R_WI * point.position.cast<double>() + T_W_I.p_WI - T_W_C.p_WI))
          .cast<float>(), point.intensity});
    }
  }
  return points;
}

std::vector<common::MapPoint> rebuildOptimizedMap(
  const KeyframeCloudStore & store,
  const std::vector<backend::Keyframe> & optimized_keyframes,
  const float voxel_size)
{
  std::vector<common::MapPoint> points;
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0F) {return points;}
  std::unordered_set<common::VoxelKey, common::VoxelKeyHash> occupied;
  for (const auto & keyframe : optimized_keyframes) {
    KeyframeCloud cloud;
    if (!store.get(keyframe.id, cloud)) {continue;}
    for (const auto & point_I : cloud.points_I) {
      common::MapPoint point_W{
        (keyframe.pose_WI.R_WI * point_I.position.cast<double>() +
        keyframe.pose_WI.p_WI).cast<float>(), point_I.intensity};
      common::VoxelKey key;
      if (common::makeVoxelKey(point_W.position, voxel_size, key) &&
        occupied.emplace(key).second)
      {
        points.push_back(point_W);
      }
    }
  }
  return points;
}

}  // namespace itera_lio::loop
