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

#ifndef ITERA_LIO__FRONTEND__GLOBAL_MAP_HPP_
#define ITERA_LIO__FRONTEND__GLOBAL_MAP_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "itera_lio/common/map_types.hpp"

namespace itera_lio::frontend
{

// Compatibility alias for callers that historically obtained MapPoint from
// the frontend map headers.  The storage type itself lives in common/domain.
using MapPoint = common::MapPoint;

struct GlobalMapStats
{
  std::size_t chunk_count{0U};
  std::size_t max_chunk_points{0U};
  std::uint64_t rehash_count{0U};
  std::uint64_t outer_rehash_count{0U};
  std::uint64_t marginalized_chunks{0U};
  std::uint64_t marginalized_points{0U};
  double last_rehash_duration_ms{0.0};
  double max_rehash_duration_ms{0.0};
};

// Representatives that changed as a result of one archive update.  A caller
// can use this delta to update a derived product without rereading the archive.
struct GlobalMapUpdate
{
  std::size_t inserted_count{0U};
  std::size_t replaced_count{0U};
  std::vector<common::MapPoint> upserted_points;
};

class GlobalMap
{
public:
  explicit GlobalMap(float voxel_size = 0.3F, std::size_t max_resident_chunks = 0U);
  ~GlobalMap();

  GlobalMap(const GlobalMap &) = delete;
  GlobalMap & operator=(const GlobalMap &) = delete;

  void build(
    const std::vector<common::MapPoint> & points,
    GlobalMapUpdate * update = nullptr);
  int addPoints(
    const std::vector<common::MapPoint> & points,
    GlobalMapUpdate * update = nullptr);
  bool forEachPoint(const std::function<void(const common::MapPoint &)> & visitor) const;
  bool getAllPoints(std::vector<common::MapPoint> & points) const;
  std::size_t size() const;
  GlobalMapStats stats() const;

private:
  struct Impl;
  Impl * impl_{nullptr};
};

}  // namespace itera_lio::frontend

#endif  // ITERA_LIO__FRONTEND__GLOBAL_MAP_HPP_
