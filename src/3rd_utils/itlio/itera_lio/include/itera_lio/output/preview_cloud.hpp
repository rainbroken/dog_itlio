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

#ifndef ITERA_LIO__OUTPUT__PREVIEW_CLOUD_HPP_
#define ITERA_LIO__OUTPUT__PREVIEW_CLOUD_HPP_

#include <cstddef>
#include <mutex>
#include <set>
#include <unordered_map>
#include <vector>

#include "itera_lio/common/map_types.hpp"
#include "itera_lio/common/voxel.hpp"
#include "itera_lio/sensor/lidar_frame.hpp"

namespace itera_lio::output::preview {

// Incremental RViz-only cache.  It never reads the complete mapping archive:
// callers feed archive deltas and snapshots only copy this bounded cache.
class PreviewMap {
public:
  PreviewMap(float voxel_size, std::size_t max_points);

  void addPoints(const std::vector<common::MapPoint> &points);
  bool getAllPoints(std::vector<common::MapPoint> &points) const;
  bool getAllPointsBatched(
    std::vector<common::MapPoint> &points, std::size_t batch_size) const;
  std::size_t size() const;

private:
  struct Entry {
    common::MapPoint point;
    float squared_center_distance{0.0F};
  };
  struct RankedKey {
    std::size_t rank{0U};
    common::VoxelKey key;
  };
  struct RankedKeyLess {
    bool operator()(const RankedKey &left, const RankedKey &right) const;
  };

  static RankedKey rankedKey(const common::VoxelKey &key);

  float voxel_size_;
  std::size_t max_points_;
  bool ranking_initialized_{false};
  mutable std::mutex mutex_;
  std::unordered_map<common::VoxelKey, Entry, common::VoxelKeyHash> voxels_;
  std::set<RankedKey, RankedKeyLess> ranked_voxels_;
};

sensor::LidarFrame sampleLidarFrame(const sensor::LidarFrame &, float,
                                    std::size_t);

std::vector<common::MapPoint>
sampleMapPoints(const std::vector<common::MapPoint> &, float, std::size_t);

} // namespace itera_lio::output::preview

#endif // ITERA_LIO__OUTPUT__PREVIEW_CLOUD_HPP_
