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

#ifndef ITERA_LIO__FRONTEND__LOCAL_MAP_MANAGER_HPP_
#define ITERA_LIO__FRONTEND__LOCAL_MAP_MANAGER_HPP_

#include <Eigen/Core>

#include <vector>
#include <deque>
#include <utility>

#include "itera_lio/common/map_types.hpp"
#include "itera_lio/frontend/map_interfaces.hpp"

namespace itera_lio::frontend
{

using MapPoint = common::MapPoint;
using MapBox = common::MapBox;

struct LocalMapOptions
{
  float cube_size{100.0F};
  float move_threshold{20.0F};
  float delete_chunk_size{10.0F};
  std::size_t max_delete_boxes_per_update{1U};
  std::size_t max_delete_points_per_update{5000U};
  std::size_t max_delete_split_depth{12U};
};

class LocalMapManager
{
public:
  explicit LocalMapManager(const LocalMapOptions & options = {});

  std::vector<MapBox> update(
    const Eigen::Vector3f & position, MapStore & map,
    std::vector<MapPoint> * deleted_points = nullptr);
  Eigen::Vector3f center() const {return center_;}
  bool initialized() const {return initialized_;}
  std::size_t pendingDeleteBoxes() const {return pending_delete_boxes_.size();}
  double lastDeleteTimeMs() const {return last_delete_time_ms_;}
  std::size_t lastDeletedPointCount() const {return last_deleted_point_count_;}

private:
  LocalMapOptions options_;
  Eigen::Vector3f center_{Eigen::Vector3f::Zero()};
  bool initialized_{false};
  std::deque<std::pair<MapBox, std::size_t>> pending_delete_boxes_;
  double last_delete_time_ms_{0.0};
  std::size_t last_deleted_point_count_{0U};
};

}  // namespace itera_lio::frontend

#endif  // ITERA_LIO__FRONTEND__LOCAL_MAP_MANAGER_HPP_
