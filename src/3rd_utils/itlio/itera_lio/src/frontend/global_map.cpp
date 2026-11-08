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

#include "itera_lio/frontend/global_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace itera_lio::frontend
{

namespace
{

struct VoxelKey
{
  // The archive is an online map in a bounded robot workspace.  int32 is
  // sufficient for ordinary metric coordinates while halving the key
  // storage compared with three int64 members (and avoiding padding growth
  // in unordered_map nodes).
  std::int32_t x;
  std::int32_t y;
  std::int32_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  static std::uint64_t mix(std::uint64_t value)
  {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
  }

  std::size_t operator()(const VoxelKey & key) const
  {
    const auto x = mix(static_cast<std::uint64_t>(key.x));
    const auto y = mix(static_cast<std::uint64_t>(key.y));
    const auto z = mix(static_cast<std::uint64_t>(key.z));
    return static_cast<std::size_t>(mix(x ^ (y << 1U) ^ (z << 7U)));
  }
};

struct VoxelEntry
{
  common::MapPoint point;
  float squared_center_distance{0.0F};
};

constexpr std::int32_t kChunkVoxelsPerAxis = 64;

struct ChunkKey
{
  std::int32_t x;
  std::int32_t y;
  std::int32_t z;

  bool operator==(const ChunkKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct ChunkKeyHash
{
  std::size_t operator()(const ChunkKey & key) const
  {
    return VoxelKeyHash{}(VoxelKey{key.x, key.y, key.z});
  }
};

struct LocalVoxelKey
{
  std::uint8_t x;
  std::uint8_t y;
  std::uint8_t z;

  bool operator==(const LocalVoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct LocalVoxelKeyHash
{
  std::size_t operator()(const LocalVoxelKey & key) const
  {
    const std::uint32_t packed = static_cast<std::uint32_t>(key.x) |
      (static_cast<std::uint32_t>(key.y) << 8U) |
      (static_cast<std::uint32_t>(key.z) << 16U);
    return static_cast<std::size_t>(VoxelKeyHash::mix(packed));
  }
};

struct Chunk
{
  std::unordered_map<LocalVoxelKey, VoxelEntry, LocalVoxelKeyHash> voxels;
};

std::int32_t floorDivide(const std::int32_t value, const std::int32_t divisor)
{
  std::int32_t quotient = value / divisor;
  if (value % divisor < 0) {
    --quotient;
  }
  return quotient;
}

void makeChunkKeys(const VoxelKey & voxel, ChunkKey & chunk, LocalVoxelKey & local)
{
  chunk = ChunkKey{
    floorDivide(voxel.x, kChunkVoxelsPerAxis),
    floorDivide(voxel.y, kChunkVoxelsPerAxis),
    floorDivide(voxel.z, kChunkVoxelsPerAxis)};
  local = LocalVoxelKey{
    static_cast<std::uint8_t>(voxel.x - chunk.x * kChunkVoxelsPerAxis),
    static_cast<std::uint8_t>(voxel.y - chunk.y * kChunkVoxelsPerAxis),
    static_cast<std::uint8_t>(voxel.z - chunk.z * kChunkVoxelsPerAxis)};
}

template<typename Map>
bool insertionMayRehash(const Map & map)
{
  return map.bucket_count() <= 1U ||
         static_cast<float>(map.size() + 1U) >
         static_cast<float>(map.bucket_count()) * map.max_load_factor();
}

bool makeVoxelKey(const Eigen::Vector3f & scaled, VoxelKey & key)
{
  constexpr double kMin = static_cast<double>(std::numeric_limits<std::int32_t>::min());
  constexpr double kMax = static_cast<double>(std::numeric_limits<std::int32_t>::max());
  const Eigen::Array3d floored = scaled.cast<double>().array().floor();
  if (!floored.allFinite() || (floored < kMin).any() || (floored > kMax).any()) {
    return false;
  }
  key = VoxelKey{
    static_cast<std::int32_t>(floored.x()),
    static_cast<std::int32_t>(floored.y()),
    static_cast<std::int32_t>(floored.z())};
  return true;
}

}  // namespace

struct GlobalMap::Impl
{
  explicit Impl(const float size, const std::size_t chunk_limit)
  : voxel_size(size), max_resident_chunks(chunk_limit) {}

  float voxel_size;
  std::size_t max_resident_chunks{0U};
  std::unordered_map<ChunkKey, Chunk, ChunkKeyHash> chunks;
  std::deque<ChunkKey> chunk_order;
  std::size_t point_count{0U};
  GlobalMapStats stats;

  void recordRehash(const double duration_ms, const bool outer)
  {
    ++stats.rehash_count;
    if (outer) {
      ++stats.outer_rehash_count;
    }
    stats.last_rehash_duration_ms = duration_ms;
    stats.max_rehash_duration_ms = std::max(stats.max_rehash_duration_ms, duration_ms);
  }

  void makeRoomForChunk()
  {
    while (max_resident_chunks > 0U && chunks.size() >= max_resident_chunks &&
      !chunk_order.empty())
    {
      const ChunkKey oldest = chunk_order.front();
      chunk_order.pop_front();
      const auto found = chunks.find(oldest);
      if (found == chunks.end()) {continue;}
      const std::size_t removed = found->second.voxels.size();
      point_count -= removed;
      chunks.erase(found);
      ++stats.marginalized_chunks;
      stats.marginalized_points += removed;
    }
  }
};

GlobalMap::GlobalMap(const float voxel_size, const std::size_t max_resident_chunks)
: impl_(new Impl(voxel_size, max_resident_chunks))
{
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0F) {
    delete impl_;
    impl_ = nullptr;
    throw std::invalid_argument("global map voxel size must be finite and positive");
  }
}

GlobalMap::~GlobalMap()
{
  delete impl_;
}

void GlobalMap::build(
  const std::vector<common::MapPoint> & points, GlobalMapUpdate * update)
{
  impl_->chunks.clear();
  impl_->chunk_order.clear();
  impl_->point_count = 0U;
  impl_->stats = {};
  if (update != nullptr) {
    *update = {};
  }
  addPoints(points, update);
}

int GlobalMap::addPoints(
  const std::vector<common::MapPoint> & points, GlobalMapUpdate * update)
{
  int inserted = 0;
  // A scan can replace one archive representative more than once.  Publish
  // only its final representative in the delta; intermediate choices are not
  // part of the archive after this call returns.
  std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> update_indices;
  if (update != nullptr) {
    update_indices.reserve(points.size());
  }
  const auto record_update = [&](const VoxelKey & key, const common::MapPoint & point) {
      if (update == nullptr) {
        return;
      }
      const auto [iterator, inserted_key] = update_indices.try_emplace(
        key, update->upserted_points.size());
      if (inserted_key) {
        update->upserted_points.push_back(point);
      } else {
        update->upserted_points[iterator->second] = point;
      }
    };
  for (const auto & point : points) {
    if (!point.position.allFinite() || !std::isfinite(point.intensity)) {
      continue;
    }
    const Eigen::Array3f scaled = point.position.array() / impl_->voxel_size;
    VoxelKey key;
    if (!makeVoxelKey(scaled.matrix(), key)) {
      continue;
    }
    const Eigen::Vector3f center =
      (Eigen::Vector3f(
        static_cast<float>(key.x), static_cast<float>(key.y), static_cast<float>(key.z)) +
      Eigen::Vector3f::Constant(0.5F)) * impl_->voxel_size;
    const float squared_distance = (point.position - center).squaredNorm();
    ChunkKey chunk_key;
    LocalVoxelKey local_key;
    makeChunkKeys(key, chunk_key, local_key);
    auto chunk_iterator = impl_->chunks.find(chunk_key);
    if (chunk_iterator == impl_->chunks.end()) {
      impl_->makeRoomForChunk();
      const bool may_rehash = insertionMayRehash(impl_->chunks);
      const auto rehash_start = may_rehash ? std::chrono::steady_clock::now() :
        std::chrono::steady_clock::time_point{};
      const std::size_t buckets_before = impl_->chunks.bucket_count();
      chunk_iterator = impl_->chunks.try_emplace(chunk_key).first;
      impl_->chunk_order.push_back(chunk_key);
      if (impl_->chunks.bucket_count() != buckets_before) {
        impl_->recordRehash(
          std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - rehash_start).count(), true);
      }
    }
    auto & chunk = chunk_iterator->second.voxels;
    const auto existing = chunk.find(local_key);
    if (existing != chunk.end()) {
      if (squared_distance < existing->second.squared_center_distance) {
        existing->second = VoxelEntry{point, squared_distance};
        if (update != nullptr) {
          ++update->replaced_count;
          record_update(key, point);
        }
      }
      continue;
    }
    const bool may_rehash = insertionMayRehash(chunk);
    const auto rehash_start = may_rehash ? std::chrono::steady_clock::now() :
      std::chrono::steady_clock::time_point{};
    const std::size_t buckets_before = chunk.bucket_count();
    const auto [iterator, was_inserted] = chunk.try_emplace(
      local_key, VoxelEntry{point, squared_distance});
    (void)iterator;
    if (was_inserted) {
      ++inserted;
      ++impl_->point_count;
      impl_->stats.max_chunk_points = std::max(impl_->stats.max_chunk_points, chunk.size());
      if (update != nullptr) {
        ++update->inserted_count;
        record_update(key, point);
      }
    }
    if (chunk.bucket_count() != buckets_before) {
      impl_->recordRehash(
        std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - rehash_start).count(), false);
    }
  }
  return inserted;
}

bool GlobalMap::forEachPoint(
  const std::function<void(const common::MapPoint &)> & visitor) const
{
  if (!visitor) {
    return false;
  }
  for (const auto & chunk : impl_->chunks) {
    for (const auto & entry : chunk.second.voxels) {
      visitor(entry.second.point);
    }
  }
  return impl_->point_count > 0U;
}

bool GlobalMap::getAllPoints(std::vector<common::MapPoint> & points) const
{
  points.clear();
  points.reserve(impl_->point_count);
  for (const auto & chunk : impl_->chunks) {
    for (const auto & entry : chunk.second.voxels) {
      points.push_back(entry.second.point);
    }
  }
  return !points.empty();
}

std::size_t GlobalMap::size() const
{
  return impl_->point_count;
}

GlobalMapStats GlobalMap::stats() const
{
  GlobalMapStats result = impl_->stats;
  result.chunk_count = impl_->chunks.size();
  return result;
}

}  // namespace itera_lio::frontend
