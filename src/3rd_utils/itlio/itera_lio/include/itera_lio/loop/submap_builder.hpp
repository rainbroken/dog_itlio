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

#ifndef ITERA_LIO__LOOP__SUBMAP_BUILDER_HPP_
#define ITERA_LIO__LOOP__SUBMAP_BUILDER_HPP_

#include <cstddef>
#include <vector>

#include "itera_lio/loop/keyframe_cloud_store.hpp"

namespace itera_lio::loop
{

// Express nearby historical clouds in the matched keyframe's IMU frame.
std::vector<common::MapPoint> buildLoopSubmap(
  const KeyframeCloudStore & store, common::KeyframeId matched_id,
  common::KeyframeId current_id, std::size_t half_window,
  std::size_t min_separation);

// Express a short recent history in the current keyframe's IMU frame.
std::vector<common::MapPoint> buildCurrentLoopSubmap(
  const KeyframeCloudStore & store, common::KeyframeId current_id,
  std::size_t half_window);

// Reproject the original keyframe-local clouds with optimized graph poses.
std::vector<common::MapPoint> rebuildOptimizedMap(
  const KeyframeCloudStore & store,
  const std::vector<backend::Keyframe> & optimized_keyframes,
  float voxel_size);

}  // namespace itera_lio::loop

#endif  // ITERA_LIO__LOOP__SUBMAP_BUILDER_HPP_
