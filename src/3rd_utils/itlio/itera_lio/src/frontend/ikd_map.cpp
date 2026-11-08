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

#include "itera_lio/frontend/ikd_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "ikd_tree/ikd_Tree.h"

namespace itera_lio::frontend
{

struct IkdMap::Impl
{
  Impl(
    float voxel_size, IkdRebuildPolicy rebuild_policy,
    std::size_t asynchronous_min_subtree_points)
  : tree(
      0.5F, 0.7F, voxel_size,
      rebuild_policy != IkdRebuildPolicy::kSynchronous,
      rebuild_policy == IkdRebuildPolicy::kAsynchronous ? 1500 :
      static_cast<int>(std::min<std::size_t>(
        asynchronous_min_subtree_points,
        static_cast<std::size_t>(std::numeric_limits<int>::max())))),
    voxel_size(voxel_size) {}

  KD_TREE<ikdTree_PointType> tree;
  float voxel_size;
  bool concurrent_read_prepared{false};
  struct ReadGuard final : NeighborMap::ConcurrentReadGuard
  {
    explicit ReadGuard(Impl & owner) : owner(owner) {
      owner.tree.Begin_Concurrent_Read();
      owner.concurrent_read_prepared = true;
    }
    ~ReadGuard() override {
      owner.concurrent_read_prepared = false;
      owner.tree.End_Concurrent_Read();
    }
    Impl & owner;
  };
};

namespace
{

ikdTree_PointType toIkd(const MapPoint & point)
{
  ikdTree_PointType result(point.position.x(), point.position.y(), point.position.z());
  result.intensity = point.intensity;
  return result;
}

MapPoint fromIkd(const ikdTree_PointType & point)
{
  return MapPoint{Eigen::Vector3f(point.x, point.y, point.z), point.intensity};
}

}  // namespace

IkdMap::IkdMap(
  const float map_voxel_size, const IkdRebuildPolicy rebuild_policy,
  const std::size_t asynchronous_min_subtree_points)
: impl_(new Impl(map_voxel_size, rebuild_policy, asynchronous_min_subtree_points))
{
  if (!std::isfinite(map_voxel_size) || map_voxel_size <= 0.0F ||
    asynchronous_min_subtree_points == 0U)
  {
    delete impl_;
    impl_ = nullptr;
    throw std::invalid_argument("map voxel size must be finite and positive");
  }
}

std::size_t IkdMap::countPoints(const MapBox & box, const std::size_t max_count) const
{
  if (!(box.max.array() > box.min.array()).all() || max_count == 0U) {
    return 0U;
  }
  BoxPointType query;
  for (int axis = 0; axis < 3; ++axis) {
    query.vertex_min[axis] = box.min[axis];
    query.vertex_max[axis] = box.max[axis];
  }
  return impl_->tree.Box_Count(query, max_count);
}

IkdMap::~IkdMap()
{
  delete impl_;
}

std::unique_ptr<NeighborMap::ConcurrentReadGuard> IkdMap::prepareConcurrentRead() const
{
  if (impl_->concurrent_read_prepared) {throw std::logic_error("nested concurrent map read");}
  return std::make_unique<Impl::ReadGuard>(*impl_);
}

bool IkdMap::concurrentReadPrepared() const {return impl_->concurrent_read_prepared;}

void IkdMap::build(const std::vector<MapPoint> & points)
{
  KD_TREE<ikdTree_PointType>::PointVector converted;
  converted.reserve(points.size());
  for (const auto & point : points) {
    converted.push_back(toIkd(point));
  }
  impl_->tree.Build(converted);
}

common::MapInsertReport IkdMap::addPoints(
  const std::vector<MapPoint> & points, const bool downsample)
{
  common::MapInsertReport result;
  result.input_count = points.size();
  KD_TREE<ikdTree_PointType>::PointVector converted;
  converted.reserve(points.size());
  for (const auto & point : points) {
    converted.push_back(toIkd(point));
  }
  if (converted.empty()) {
    return result;
  }
  const auto report = impl_->tree.Add_Points_With_Report(converted, downsample);
  result.inserted_count = report.inserted_count;
  result.replaced_count = report.replaced_count;
  result.rejected_count = report.rejected_count;
  result.compacted_count = report.compacted_count;
  result.net_size_delta = report.net_size_delta;
  result.upserted_points.reserve(report.upserted_points.size());
  for (const auto & point : report.upserted_points) {
    result.upserted_points.push_back(fromIkd(point));
  }
  result.removed_points.reserve(report.removed_points.size());
  for (const auto & point : report.removed_points) {
    result.removed_points.push_back(fromIkd(point));
  }
  return result;
}

int IkdMap::deleteBoxes(
  const std::vector<MapBox> & boxes, std::vector<MapPoint> * deleted_points)
{
  std::vector<BoxPointType> converted;
  converted.reserve(boxes.size());
  for (const auto & box : boxes) {
    BoxPointType value;
    for (int axis = 0; axis < 3; ++axis) {
      value.vertex_min[axis] = box.min[axis];
      value.vertex_max[axis] = box.max[axis];
    }
    converted.push_back(value);
  }
  if (deleted_points != nullptr) {
    deleted_points->clear();
    for (const auto & box : converted) {
      KD_TREE<ikdTree_PointType>::PointVector storage;
      impl_->tree.Box_Search(box, storage);
      deleted_points->reserve(deleted_points->size() + storage.size());
      for (const auto & value : storage) {
        deleted_points->push_back(fromIkd(value));
      }
    }
  }
  return converted.empty() ? 0 : impl_->tree.Delete_Point_Boxes(converted);
}

bool IkdMap::nearestSearch(
  const MapPoint & point, const int k, const float max_distance,
  std::vector<MapPoint> & neighbors, std::vector<float> & squared_distances) const
{
  neighbors.clear();
  squared_distances.clear();
  // Nearest_Search handles a null root. The vendor size() uses trylock and
  // may return a stale zero while other query workers inspect the same root.
  if (k <= 0 || !std::isfinite(max_distance) || max_distance < 0.0F)
  {
    return false;
  }
  thread_local KD_TREE<ikdTree_PointType>::PointVector nearest_result;
  nearest_result.clear();
  nearest_result.reserve(static_cast<std::size_t>(k));
  impl_->tree.Nearest_Search(
    toIkd(point), k, nearest_result, squared_distances, max_distance);
  neighbors.reserve(nearest_result.size());
  for (const auto & value : nearest_result) {
    MapPoint point;
    point.position = Eigen::Vector3f(value.x, value.y, value.z);
    point.intensity = value.intensity;
    neighbors.push_back(point);
  }
  return !neighbors.empty() && neighbors.size() == squared_distances.size();
}

bool IkdMap::nearestSearchPositions(
  const Eigen::Vector3f & point, const int k, const float max_distance,
  std::vector<Eigen::Vector3d> & neighbors, std::vector<float> & squared_distances) const
{
  neighbors.clear();
  squared_distances.clear();
  if (k <= 0 || !point.allFinite() || !std::isfinite(max_distance) || max_distance < 0.0F)
  {
    return false;
  }
  thread_local KD_TREE<ikdTree_PointType>::PointVector nearest_result;
  nearest_result.clear();
  nearest_result.reserve(static_cast<std::size_t>(k));
  impl_->tree.Nearest_Search(
    ikdTree_PointType(point.x(), point.y(), point.z()), k, nearest_result,
    squared_distances, max_distance);
  neighbors.reserve(nearest_result.size());
  for (const auto & value : nearest_result) {
    neighbors.emplace_back(
      static_cast<double>(value.x), static_cast<double>(value.y), static_cast<double>(value.z));
  }
  return !neighbors.empty() && neighbors.size() == squared_distances.size();
}

bool IkdMap::getAllPoints(std::vector<MapPoint> & points) const
{
  points.clear();
  if (impl_ == nullptr || impl_->tree.size() <= 0) {
    return false;
  }

  BoxPointType query;
  for (int axis = 0; axis < 3; ++axis) {
    query.vertex_min[axis] = std::numeric_limits<float>::lowest();
    query.vertex_max[axis] = std::numeric_limits<float>::max();
  }

  KD_TREE<ikdTree_PointType>::PointVector storage;
  impl_->tree.Box_Search(query, storage);
  points.reserve(storage.size());
  for (const auto & value : storage) {
    MapPoint point;
    point.position = Eigen::Vector3f(value.x, value.y, value.z);
    point.intensity = value.intensity;
    points.push_back(point);
  }
  return !points.empty();
}

std::size_t IkdMap::size() const
{
  return static_cast<std::size_t>(impl_->tree.validnum());
}

IkdRebuildStats IkdMap::rebuildStats() const
{
  const RebuildStats source = impl_->tree.rebuild_stats();
  IkdRebuildStats result;
  result.total_count = source.total_count;
  result.synchronous_count = source.synchronous_count;
  result.asynchronous_count = source.asynchronous_count;
  result.last_reason = static_cast<std::uint8_t>(source.last_reason);
  result.last_subtree_size = source.last_subtree_size;
  result.max_subtree_size = source.max_subtree_size;
  result.last_duration_ms = source.last_duration_ms;
  result.max_duration_ms = source.max_duration_ms;
  result.search_lock_wait_count = source.search_lock_wait_count;
  result.search_lock_wait_total_ms = source.search_lock_wait_total_ms;
  result.search_lock_wait_max_ms = source.search_lock_wait_max_ms;
  result.asynchronous_in_progress = source.asynchronous_in_progress;
  return result;
}

}  // namespace itera_lio::frontend
