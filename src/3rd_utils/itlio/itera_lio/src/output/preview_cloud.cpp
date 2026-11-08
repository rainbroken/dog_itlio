// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0
#include "itera_lio/output/preview_cloud.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>
namespace itera_lio::output::preview {
namespace {
struct Key {
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};
  bool operator==(const Key &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};
struct Hash {
  std::size_t operator()(const Key &key) const {
    std::size_t h = std::hash<std::int64_t>{}(key.x);
    h ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b97f4a7c15ULL + (h << 6U) +
         (h >> 2U);
    h ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b97f4a7c15ULL + (h << 6U) +
         (h >> 2U);
    return h;
  }
};
bool isFinite(const sensor::LidarPoint &point) { return point.isFinite(); }
bool isFinite(const common::MapPoint &point) {
  return point.position.allFinite() && std::isfinite(point.intensity);
}
Eigen::Vector3f positionOf(const sensor::LidarPoint &point) {
  return point.position.cast<float>();
}
const Eigen::Vector3f &positionOf(const common::MapPoint &point) {
  return point.position;
}
Key keyFor(const Eigen::Vector3f &point, const float size) {
  return {static_cast<std::int64_t>(std::floor(point.x() / size)),
          static_cast<std::int64_t>(std::floor(point.y() / size)),
          static_cast<std::int64_t>(std::floor(point.z() / size))};
}
Eigen::Vector3f centerFor(const Key &key, const float size) {
  return {(static_cast<float>(key.x) + 0.5F) * size,
          (static_cast<float>(key.y) + 0.5F) * size,
          (static_cast<float>(key.z) + 0.5F) * size};
}
template <typename Point>
std::vector<Point> sample(const std::vector<Point> &points, float size,
                          std::size_t limit) {
  if (!std::isfinite(size) || size <= 0.0F) {
    return {};
  }
  std::unordered_map<Key, Point, Hash> reps;
  reps.reserve(limit > 0U ? std::min(points.size(), limit) : points.size());
  for (const auto &point : points) {
    if (!isFinite(point)) {
      continue;
    }
    const Eigen::Vector3f position = positionOf(point);
    const auto key = keyFor(position, size);
    const auto center = centerFor(key, size);
    const auto it = reps.find(key);
    if (it == reps.end() ||
        (position - center).squaredNorm() <
            (positionOf(it->second) - center).squaredNorm()) {
      reps[key] = point;
    }
  }
  std::vector<std::pair<Key, Point>> ordered;
  ordered.reserve(reps.size());
  for (const auto &entry : reps) {
    ordered.push_back(entry);
  }
  std::sort(ordered.begin(), ordered.end(), [](const auto &a, const auto &b) {
    return std::tie(a.first.x, a.first.y, a.first.z) <
           std::tie(b.first.x, b.first.y, b.first.z);
  });
  if (limit > 0U && ordered.size() > limit) {
    ordered.resize(limit);
  }
  std::vector<Point> result;
  result.reserve(ordered.size());
  for (const auto &entry : ordered) {
    result.push_back(entry.second);
  }
  return result;
}

} // namespace

PreviewMap::PreviewMap(const float voxel_size, const std::size_t max_points)
    : voxel_size_(voxel_size), max_points_(max_points) {
  if (!std::isfinite(voxel_size_) || voxel_size_ <= 0.0F) {
    throw std::invalid_argument(
        "preview map voxel size must be finite and positive");
  }
}

bool PreviewMap::RankedKeyLess::operator()(const RankedKey &left,
                                           const RankedKey &right) const {
  return std::tie(left.rank, left.key.x, left.key.y, left.key.z) <
         std::tie(right.rank, right.key.x, right.key.y, right.key.z);
}

PreviewMap::RankedKey PreviewMap::rankedKey(const common::VoxelKey &key) {
  return {common::VoxelKeyHash{}(key), key};
}

void PreviewMap::addPoints(const std::vector<common::MapPoint> &points) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto &point : points) {
    if (!isFinite(point)) {
      continue;
    }
    common::VoxelKey key;
    if (!common::makeVoxelKey(point.position, voxel_size_, key)) {
      continue;
    }
    const float distance =
        (point.position - common::voxelCenter(key, voxel_size_)).squaredNorm();
    const auto existing = voxels_.find(key);
    if (existing != voxels_.end()) {
      if (distance < existing->second.squared_center_distance) {
        existing->second = Entry{point, distance};
      }
      continue;
    }

    if (max_points_ > 0U && voxels_.size() >= max_points_) {
      if (!ranking_initialized_) {
        for (const auto &entry : voxels_) {
          ranked_voxels_.insert(rankedKey(entry.first));
        }
        ranking_initialized_ = true;
      }
      const RankedKey candidate = rankedKey(key);
      const auto worst = std::prev(ranked_voxels_.end());
      if (!RankedKeyLess{}(candidate, *worst)) {
        continue;
      }
      voxels_.erase(worst->key);
      ranked_voxels_.erase(worst);
    }
    voxels_.emplace(key, Entry{point, distance});
    if (ranking_initialized_) {
      ranked_voxels_.insert(rankedKey(key));
    }
  }
}

bool PreviewMap::getAllPoints(std::vector<common::MapPoint> &points) const {
  std::lock_guard<std::mutex> lock(mutex_);
  points.clear();
  points.reserve(voxels_.size());
  for (const auto &entry : voxels_) {
    points.push_back(entry.second.point);
  }
  return !points.empty();
}

bool PreviewMap::getAllPointsBatched(
    std::vector<common::MapPoint> &points,
    const std::size_t batch_size) const {
  if (batch_size == 0U) {
    return false;
  }
  std::vector<common::VoxelKey> keys;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    keys.reserve(voxels_.size());
    for (const auto &entry : voxels_) {
      keys.push_back(entry.first);
    }
  }
  points.clear();
  points.reserve(keys.size());
  for (std::size_t offset = 0U; offset < keys.size(); offset += batch_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::size_t end = std::min(offset + batch_size, keys.size());
    for (std::size_t index = offset; index < end; ++index) {
      const auto entry = voxels_.find(keys[index]);
      if (entry != voxels_.end()) {
        points.push_back(entry->second.point);
      }
    }
  }
  return !points.empty();
}

std::size_t PreviewMap::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return voxels_.size();
}

sensor::LidarFrame sampleLidarFrame(const sensor::LidarFrame &scan, float size,
                                    std::size_t limit) {
  sensor::LidarFrame result = scan;
  result.points = sample(scan.points, size, limit);
  return result;
}
std::vector<common::MapPoint>
sampleMapPoints(const std::vector<common::MapPoint> &points, float size,
                std::size_t limit) {
  return sample(points, size, limit);
}

} // namespace itera_lio::output::preview
