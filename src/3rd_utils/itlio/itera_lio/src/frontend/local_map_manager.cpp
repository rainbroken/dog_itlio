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

#include "itera_lio/frontend/local_map_manager.hpp"

#include <cmath>
#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>

namespace itera_lio::frontend
{

namespace
{

bool isValidBox(const MapBox & box)
{
  return (box.max.array() > box.min.array()).all();
}

void enqueueChunks(
  const MapBox & box, const float chunk_size,
  std::deque<std::pair<MapBox, std::size_t>> & pending_boxes)
{
  for (float x = box.min.x(); x < box.max.x(); x += chunk_size) {
    for (float y = box.min.y(); y < box.max.y(); y += chunk_size) {
      for (float z = box.min.z(); z < box.max.z(); z += chunk_size) {
        MapBox chunk;
        chunk.min = Eigen::Vector3f(x, y, z);
        chunk.max = Eigen::Vector3f(
          std::min(box.max.x(), x + chunk_size),
          std::min(box.max.y(), y + chunk_size),
          std::min(box.max.z(), z + chunk_size));
        pending_boxes.emplace_back(chunk, 0U);
      }
    }
  }
}

}  // namespace

LocalMapManager::LocalMapManager(const LocalMapOptions & options)
: options_(options)
{
  if (!std::isfinite(options_.cube_size) || options_.cube_size <= 0.0F ||
    !std::isfinite(options_.move_threshold) || options_.move_threshold <= 0.0F ||
    options_.move_threshold >= options_.cube_size * 0.5F ||
    !std::isfinite(options_.delete_chunk_size) || options_.delete_chunk_size <= 0.0F ||
    options_.max_delete_boxes_per_update == 0U ||
    options_.max_delete_points_per_update == 0U ||
    options_.max_delete_split_depth == 0U)
  {
    throw std::invalid_argument("invalid local map options");
  }
}

std::vector<MapBox> LocalMapManager::update(
  const Eigen::Vector3f & position, MapStore & map, std::vector<MapPoint> * deleted_points)
{
  std::vector<MapBox> deleted;
  last_delete_time_ms_ = 0.0;
  last_deleted_point_count_ = 0U;
  if (deleted_points != nullptr) {
    deleted_points->clear();
  }
  if (!position.allFinite()) {return deleted;}
  if (!initialized_) {
    center_ = position;
    initialized_ = true;
    return deleted;
  }
  const Eigen::Vector3f delta = position - center_;
  if (delta.cwiseAbs().maxCoeff() > options_.move_threshold) {
    const Eigen::Vector3f old_min = center_ - Eigen::Vector3f::Constant(options_.cube_size * 0.5F);
    const Eigen::Vector3f old_max = center_ + Eigen::Vector3f::Constant(options_.cube_size * 0.5F);
    center_ = position;
    const Eigen::Vector3f new_min = center_ - Eigen::Vector3f::Constant(options_.cube_size * 0.5F);
    const Eigen::Vector3f new_max = center_ + Eigen::Vector3f::Constant(options_.cube_size * 0.5F);
    MapBox overlap{old_min, old_max};
    for (int axis = 0; axis < 3 && isValidBox(overlap); ++axis) {
      if (new_min[axis] > overlap.min[axis]) {
        MapBox lower = overlap;
        lower.max[axis] = std::min(new_min[axis], overlap.max[axis]);
        if (isValidBox(lower)) {
          enqueueChunks(lower, options_.delete_chunk_size, pending_delete_boxes_);
        }
        overlap.min[axis] = std::max(overlap.min[axis], new_min[axis]);
      }
      if (new_max[axis] < overlap.max[axis] && isValidBox(overlap)) {
        MapBox upper = overlap;
        upper.min[axis] = std::max(new_max[axis], overlap.min[axis]);
        if (isValidBox(upper)) {
          enqueueChunks(upper, options_.delete_chunk_size, pending_delete_boxes_);
        }
        overlap.max[axis] = std::min(overlap.max[axis], new_max[axis]);
      }
    }
  }

  // Process only a bounded number of slabs. This keeps ikd-tree deletion and
  // any internal rebuild out of the critical path for a single scan.
  const Eigen::Vector3f current_min = center_ -
    Eigen::Vector3f::Constant(options_.cube_size * 0.5F);
  const Eigen::Vector3f current_max = center_ +
    Eigen::Vector3f::Constant(options_.cube_size * 0.5F);
  const auto delete_start = std::chrono::steady_clock::now();
  std::vector<MapBox> process_boxes;
  std::size_t estimated_delete_points = 0U;
  while (!pending_delete_boxes_.empty() &&
    process_boxes.size() < options_.max_delete_boxes_per_update &&
    estimated_delete_points < options_.max_delete_points_per_update)
  {
    const auto pending = pending_delete_boxes_.front();
    MapBox box = pending.first;
    const std::size_t split_depth = pending.second;
    pending_delete_boxes_.pop_front();
    bool intersects = true;
    for (int axis = 0; axis < 3; ++axis) {
      if (box.max[axis] <= current_min[axis] || box.min[axis] >= current_max[axis]) {
        intersects = false;
        break;
      }
    }
    if (!intersects) {
      const std::size_t remaining_budget =
        options_.max_delete_points_per_update - estimated_delete_points;
      const std::size_t count_limit = remaining_budget ==
        std::numeric_limits<std::size_t>::max() ? remaining_budget : remaining_budget + 1U;
      const std::size_t point_count = map.countPoints(box, count_limit);
      if (point_count > remaining_budget) {
        Eigen::Index longest_axis = 0;
        const Eigen::Vector3f extent = box.max - box.min;
        extent.maxCoeff(&longest_axis);
        if (split_depth < options_.max_delete_split_depth &&
          extent[longest_axis] > std::numeric_limits<float>::epsilon())
        {
          const float midpoint =
            box.min[longest_axis] + extent[longest_axis] * 0.5F;
          MapBox lower = box;
          MapBox upper = box;
          lower.max[longest_axis] = midpoint;
          upper.min[longest_axis] = midpoint;
          pending_delete_boxes_.emplace_front(upper, split_depth + 1U);
          pending_delete_boxes_.emplace_front(lower, split_depth + 1U);
          continue;
        }
        // A spatial box cannot always satisfy a point budget when many points
        // share the same coordinate. Defer it if this frame already has work;
        // otherwise process it once the split-depth guard is reached.
        if (!process_boxes.empty()) {
          pending_delete_boxes_.push_front(pending);
          break;
        }
      }
      process_boxes.push_back(box);
      estimated_delete_points += point_count;
      continue;
    }

    // Preserve portions outside the active cube when the platform backtracks.
    // The part inside the cube stays queued only conceptually: it is retained.
    std::vector<MapBox> outside{box};
    for (int axis = 0; axis < 3; ++axis) {
      std::vector<MapBox> next;
      for (const auto & candidate : outside) {
        if (candidate.min[axis] < current_min[axis]) {
          MapBox lower = candidate;
          lower.max[axis] = current_min[axis];
          next.push_back(lower);
        }
        if (candidate.max[axis] > current_max[axis]) {
          MapBox upper = candidate;
          upper.min[axis] = current_max[axis];
          next.push_back(upper);
        }
        MapBox middle = candidate;
        middle.min[axis] = std::max(middle.min[axis], current_min[axis]);
        middle.max[axis] = std::min(middle.max[axis], current_max[axis]);
        if (middle.min[axis] < middle.max[axis]) {
          next.push_back(middle);
        }
      }
      outside = std::move(next);
    }
    // Re-queue only the valid outside pieces. The final inside piece is
    // intentionally omitted because it is still part of the active map.
    for (auto iterator = outside.rbegin(); iterator != outside.rend(); ++iterator) {
      bool is_inside = true;
      for (int axis = 0; axis < 3; ++axis) {
        is_inside = is_inside && iterator->min[axis] >= current_min[axis] &&
          iterator->max[axis] <= current_max[axis];
      }
      if (!is_inside) {
        pending_delete_boxes_.emplace_front(*iterator, split_depth);
      }
    }
  }
  if (!process_boxes.empty()) {
    if (deleted_points != nullptr) {
      last_deleted_point_count_ = static_cast<std::size_t>(
        std::max(0, map.deleteBoxes(process_boxes, deleted_points)));
    } else {
      last_deleted_point_count_ = static_cast<std::size_t>(
        std::max(0, map.deleteBoxes(process_boxes, nullptr)));
    }
    deleted = std::move(process_boxes);
  }
  last_delete_time_ms_ = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - delete_start).count();
  return deleted;
}

}  // namespace itera_lio::frontend
