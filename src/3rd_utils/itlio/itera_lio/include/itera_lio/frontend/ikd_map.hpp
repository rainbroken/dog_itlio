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

#ifndef ITERA_LIO__FRONTEND__IKD_MAP_HPP_
#define ITERA_LIO__FRONTEND__IKD_MAP_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "itera_lio/common/map_types.hpp"
#include "itera_lio/frontend/map_interfaces.hpp"

namespace itera_lio::frontend
{

using MapPoint = common::MapPoint;
using MapBox = common::MapBox;

enum class IkdRebuildPolicy
{
  kSynchronous,
  kAsynchronous,
  kHybrid
};

struct IkdRebuildStats
{
  std::uint64_t total_count{0U};
  std::uint64_t synchronous_count{0U};
  std::uint64_t asynchronous_count{0U};
  std::uint8_t last_reason{0U};
  std::size_t last_subtree_size{0U};
  std::size_t max_subtree_size{0U};
  double last_duration_ms{0.0};
  double max_duration_ms{0.0};
  std::uint64_t search_lock_wait_count{0U};
  double search_lock_wait_total_ms{0.0};
  double search_lock_wait_max_ms{0.0};
  bool asynchronous_in_progress{false};
};

class IkdMap : public MapStore
{
public:
  explicit IkdMap(
    float map_voxel_size = 0.3F,
    IkdRebuildPolicy rebuild_policy = IkdRebuildPolicy::kHybrid,
    std::size_t asynchronous_min_subtree_points = 20000U);
  ~IkdMap();
  std::unique_ptr<ConcurrentReadGuard> prepareConcurrentRead() const override;
  bool concurrentReadPrepared() const override;

  IkdMap(const IkdMap &) = delete;
  IkdMap & operator=(const IkdMap &) = delete;

  void build(const std::vector<MapPoint> & points) override;
  common::MapInsertReport addPoints(
    const std::vector<MapPoint> & points, bool downsample) override;
  int deleteBoxes(
    const std::vector<MapBox> & boxes,
    std::vector<MapPoint> * deleted_points = nullptr) override;
  std::size_t countPoints(
    const MapBox & box,
    std::size_t max_count = std::numeric_limits<std::size_t>::max()) const override;
  bool nearestSearch(
    const MapPoint & point, int k, float max_distance,
    std::vector<MapPoint> & neighbors,
    std::vector<float> & squared_distances) const override;
  bool nearestSearchPositions(
    const Eigen::Vector3f & point, int k, float max_distance,
    std::vector<Eigen::Vector3d> & neighbors,
    std::vector<float> & squared_distances) const override;
  bool getAllPoints(std::vector<MapPoint> & points) const override;
  std::size_t size() const override;
  IkdRebuildStats rebuildStats() const;

private:
  struct Impl;
  Impl * impl_{nullptr};
};

}  // namespace itera_lio::frontend

#endif  // ITERA_LIO__FRONTEND__IKD_MAP_HPP_
