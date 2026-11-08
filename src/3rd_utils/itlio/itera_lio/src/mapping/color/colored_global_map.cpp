// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/mapping/color/colored_global_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <list>
#include <mutex>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <utility>

namespace itera_lio::mapping::color {

namespace {

constexpr double kChunkLengthMeters = 5.0;
constexpr std::uint32_t kChunkFileMagic = 0x434C494FU;

struct ChunkKey {
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  bool operator==(const ChunkKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct ChunkKeyHash {
  std::size_t operator()(const ChunkKey &key) const {
    return common::VoxelKeyHash{}({key.x, key.y, key.z});
  }
};

struct LocalVoxelKey {
  std::int32_t x{0};
  std::int32_t y{0};
  std::int32_t z{0};

  bool operator==(const LocalVoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct LocalVoxelKeyHash {
  std::size_t operator()(const LocalVoxelKey &key) const {
    return common::VoxelKeyHash{}({key.x, key.y, key.z});
  }
};

struct Entry {
  ColoredPoint point;
  float squared_center_distance{0.0F};
};

using VoxelMap = std::unordered_map<LocalVoxelKey, Entry, LocalVoxelKeyHash>;

struct RankedKey {
  std::size_t rank{0U};
  common::VoxelKey key;
};

struct RankedKeyLess {
  bool operator()(const RankedKey &left, const RankedKey &right) const {
    return std::tie(left.rank, left.key.x, left.key.y, left.key.z) <
           std::tie(right.rank, right.key.x, right.key.y, right.key.z);
  }
};

std::int64_t floorDivide(const std::int64_t value, const std::int64_t divisor) {
  std::int64_t quotient = value / divisor;
  if (value % divisor < 0) {
    --quotient;
  }
  return quotient;
}

void splitKey(const common::VoxelKey &voxel, const std::int64_t voxels_per_axis,
              ChunkKey &chunk, LocalVoxelKey &local) {
  chunk = {floorDivide(voxel.x, voxels_per_axis),
           floorDivide(voxel.y, voxels_per_axis),
           floorDivide(voxel.z, voxels_per_axis)};
  local = {static_cast<std::int32_t>(voxel.x - chunk.x * voxels_per_axis),
           static_cast<std::int32_t>(voxel.y - chunk.y * voxels_per_axis),
           static_cast<std::int32_t>(voxel.z - chunk.z * voxels_per_axis)};
}

template <typename T> bool writeValue(std::ostream &stream, const T &value) {
  stream.write(reinterpret_cast<const char *>(&value), sizeof(T));
  return static_cast<bool>(stream);
}

template <typename T> bool readValue(std::istream &stream, T &value) {
  stream.read(reinterpret_cast<char *>(&value), sizeof(T));
  return static_cast<bool>(stream);
}

bool writeEntry(std::ostream &stream, const LocalVoxelKey &key,
                const Entry &entry) {
  return writeValue(stream, key.x) && writeValue(stream, key.y) &&
         writeValue(stream, key.z) &&
         writeValue(stream, entry.point.position.x()) &&
         writeValue(stream, entry.point.position.y()) &&
         writeValue(stream, entry.point.position.z()) &&
         writeValue(stream, entry.point.intensity) &&
         writeValue(stream, entry.point.r) &&
         writeValue(stream, entry.point.g) &&
         writeValue(stream, entry.point.b) &&
         writeValue(stream, entry.squared_center_distance);
}

bool readEntry(std::istream &stream, LocalVoxelKey &key, Entry &entry) {
  return readValue(stream, key.x) && readValue(stream, key.y) &&
         readValue(stream, key.z) &&
         readValue(stream, entry.point.position.x()) &&
         readValue(stream, entry.point.position.y()) &&
         readValue(stream, entry.point.position.z()) &&
         readValue(stream, entry.point.intensity) &&
         readValue(stream, entry.point.r) && readValue(stream, entry.point.g) &&
         readValue(stream, entry.point.b) &&
         readValue(stream, entry.squared_center_distance);
}

} // namespace

struct ColoredGlobalMap::Impl {
  struct ChunkRecord {
    std::unique_ptr<VoxelMap> voxels;
    std::size_t point_count{0U};
    bool dirty{false};
    bool spilled{false};
    std::list<ChunkKey>::iterator lru_iterator;
  };

  struct PendingPoint {
    const ColoredPoint *point{nullptr};
    common::VoxelKey voxel_key;
    LocalVoxelKey local_key;
  };

  Impl(const float size, const std::size_t point_limit,
       const std::size_t resident_chunk_limit, const std::string &spill_root)
      : voxel_size(size), max_points(point_limit),
        max_resident_chunks(resident_chunk_limit),
        voxels_per_axis(
            static_cast<std::int64_t>(std::ceil(kChunkLengthMeters / size))) {
    if (!std::isfinite(voxel_size) || voxel_size <= 0.0F ||
        voxels_per_axis <= 0 ||
        voxels_per_axis > std::numeric_limits<std::int32_t>::max() ||
        (max_points > 0U && max_resident_chunks > 0U)) {
      throw std::invalid_argument("colored global map voxel size is invalid");
    }
    if (max_resident_chunks > 0U) {
      namespace fs = std::filesystem;
      const fs::path root = spill_root.empty()
                                ? fs::temp_directory_path()
                                : fs::absolute(fs::path(spill_root));
      fs::create_directories(root);
      const auto suffix =
          std::chrono::steady_clock::now().time_since_epoch().count();
      spill_directory =
          root / (".itera_lio_colored_chunks_" + std::to_string(suffix));
      fs::create_directories(spill_directory);
    }
  }

  ~Impl() {
    if (!spill_directory.empty()) {
      std::error_code error;
      std::filesystem::remove_all(spill_directory, error);
    }
  }

  std::filesystem::path chunkPath(const ChunkKey &key) const {
    return spill_directory /
           (std::to_string(key.x) + "_" + std::to_string(key.y) + "_" +
            std::to_string(key.z) + ".bin");
  }

  void touch(const ChunkKey &key, ChunkRecord &record,
             const bool was_resident) {
    if (was_resident) {
      resident_lru.erase(record.lru_iterator);
    } else {
      ++resident_chunk_count;
    }
    resident_lru.push_front(key);
    record.lru_iterator = resident_lru.begin();
  }

  bool writeChunk(const ChunkKey &key, ChunkRecord &record) {
    if (spill_directory.empty() || record.voxels == nullptr) {
      return false;
    }
    const auto path = chunkPath(key);
    const auto temporary_path = path.string() + ".tmp";
    std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
    const std::uint64_t count =
        static_cast<std::uint64_t>(record.voxels->size());
    if (!stream || !writeValue(stream, kChunkFileMagic) ||
        !writeValue(stream, count)) {
      return false;
    }
    for (const auto &voxel : *record.voxels) {
      if (!writeEntry(stream, voxel.first, voxel.second)) {
        return false;
      }
    }
    stream.flush();
    stream.close();
    if (!stream) {
      return false;
    }
    std::error_code error;
    std::filesystem::rename(temporary_path, path, error);
    if (error) {
      std::filesystem::remove(path, error);
      error.clear();
      std::filesystem::rename(temporary_path, path, error);
    }
    if (error) {
      std::filesystem::remove(temporary_path, error);
      return false;
    }
    record.dirty = false;
    record.spilled = true;
    return true;
  }

  bool readChunk(const ChunkKey &key, VoxelMap &voxels) const {
    std::ifstream stream(chunkPath(key), std::ios::binary);
    std::uint32_t magic = 0U;
    std::uint64_t count = 0U;
    if (!stream || !readValue(stream, magic) || magic != kChunkFileMagic ||
        !readValue(stream, count)) {
      return false;
    }
    voxels.clear();
    voxels.reserve(static_cast<std::size_t>(count));
    for (std::uint64_t index = 0U; index < count; ++index) {
      LocalVoxelKey local;
      Entry entry;
      if (!readEntry(stream, local, entry)) {
        return false;
      }
      voxels.emplace(local, entry);
    }
    return voxels.size() == static_cast<std::size_t>(count);
  }

  VoxelMap &makeResident(const ChunkKey &key, ChunkRecord &record) {
    const bool was_resident = record.voxels != nullptr;
    if (record.voxels == nullptr) {
      auto loaded = std::make_unique<VoxelMap>();
      if (record.spilled && !readChunk(key, *loaded)) {
        throw std::runtime_error("failed to read spilled colored-map chunk");
      }
      record.voxels = std::move(loaded);
    }
    touch(key, record, was_resident);
    return *record.voxels;
  }

  void enforceResidentBudget() {
    while (max_resident_chunks > 0U &&
           resident_chunk_count > max_resident_chunks) {
      const ChunkKey key = resident_lru.back();
      auto iterator = chunks.find(key);
      if (iterator == chunks.end() || iterator->second.voxels == nullptr) {
        throw std::logic_error("invalid colored-map LRU state");
      }
      auto &record = iterator->second;
      if (record.dirty && !writeChunk(key, record)) {
        throw std::runtime_error("failed to spill colored-map chunk");
      }
      record.voxels.reset();
      resident_lru.pop_back();
      --resident_chunk_count;
    }
  }

  bool
  visitSpilled(const ChunkKey &key,
               const std::function<void(const ColoredPoint &)> &visitor) const {
    std::ifstream stream(chunkPath(key), std::ios::binary);
    std::uint32_t magic = 0U;
    std::uint64_t count = 0U;
    if (!stream || !readValue(stream, magic) || magic != kChunkFileMagic ||
        !readValue(stream, count)) {
      return false;
    }
    for (std::uint64_t index = 0U; index < count; ++index) {
      LocalVoxelKey local;
      Entry entry;
      if (!readEntry(stream, local, entry)) {
        return false;
      }
      visitor(entry.point);
    }
    return true;
  }

  float voxel_size;
  std::size_t max_points;
  std::size_t max_resident_chunks;
  std::int64_t voxels_per_axis;
  std::filesystem::path spill_directory;
  mutable std::mutex mutex;
  std::unordered_map<ChunkKey, ChunkRecord, ChunkKeyHash> chunks;
  std::list<ChunkKey> resident_lru;
  std::size_t resident_chunk_count{0U};
  std::size_t point_count{0U};
  std::set<RankedKey, RankedKeyLess> ranked_voxels;
};

ColoredGlobalMap::ColoredGlobalMap(const float voxel_size,
                                   const std::size_t max_points,
                                   const std::size_t max_resident_chunks,
                                   const std::string &spill_directory)
    : impl_(std::make_unique<Impl>(voxel_size, max_points, max_resident_chunks,
                                   spill_directory)) {}

ColoredGlobalMap::~ColoredGlobalMap() = default;

std::size_t ColoredGlobalMap::addPoints(const std::vector<ColoredPoint> &points,
                                        ColoredGlobalMapUpdate *update) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (update != nullptr) {
    *update = {};
  }

  std::unordered_map<ChunkKey, std::vector<Impl::PendingPoint>, ChunkKeyHash>
      grouped;
  grouped.reserve(points.size());
  for (const auto &point : points) {
    common::VoxelKey voxel_key;
    if (!point.position.allFinite() || !std::isfinite(point.intensity) ||
        !common::makeVoxelKey(point.position, impl_->voxel_size, voxel_key)) {
      continue;
    }
    ChunkKey chunk_key;
    LocalVoxelKey local_key;
    splitKey(voxel_key, impl_->voxels_per_axis, chunk_key, local_key);
    grouped[chunk_key].push_back({&point, voxel_key, local_key});
  }

  std::unordered_map<common::VoxelKey, std::size_t, common::VoxelKeyHash>
      update_indices;
  if (update != nullptr) {
    update_indices.reserve(points.size());
  }
  const auto record_update = [&](const common::VoxelKey &key,
                                 const ColoredPoint &point) {
    if (update == nullptr) {
      return;
    }
    const auto [iterator, inserted] =
        update_indices.try_emplace(key, update->upserted_points.size());
    if (inserted) {
      update->upserted_points.push_back(point);
    } else {
      update->upserted_points[iterator->second] = point;
    }
  };

  std::size_t inserted_count = 0U;
  for (const auto &group : grouped) {
    auto [chunk_iterator, new_chunk] = impl_->chunks.try_emplace(group.first);
    auto &record = chunk_iterator->second;
    (void)new_chunk;
    VoxelMap &voxels = impl_->makeResident(group.first, record);
    for (const auto &pending : group.second) {
      const Eigen::Vector3f center =
          common::voxelCenter(pending.voxel_key, impl_->voxel_size);
      const float squared_distance =
          (pending.point->position - center).squaredNorm();
      const auto existing = voxels.find(pending.local_key);
      if (existing != voxels.end()) {
        if (squared_distance < existing->second.squared_center_distance) {
          existing->second = Entry{*pending.point, squared_distance};
          record.dirty = true;
          if (update != nullptr) {
            ++update->replaced_count;
            record_update(pending.voxel_key, *pending.point);
          }
        }
        continue;
      }

      const RankedKey ranked{common::VoxelKeyHash{}(pending.voxel_key),
                             pending.voxel_key};
      if (impl_->max_points > 0U && impl_->point_count >= impl_->max_points) {
        const auto worst = std::prev(impl_->ranked_voxels.end());
        if (!RankedKeyLess{}(ranked, *worst)) {
          continue;
        }
        ChunkKey worst_chunk;
        LocalVoxelKey worst_local;
        splitKey(worst->key, impl_->voxels_per_axis, worst_chunk, worst_local);
        auto &worst_record = impl_->chunks.at(worst_chunk);
        VoxelMap &worst_voxels = impl_->makeResident(worst_chunk, worst_record);
        if (worst_voxels.erase(worst_local) != 1U) {
          throw std::logic_error("ranked colored-map voxel is missing");
        }
        --worst_record.point_count;
        worst_record.dirty = true;
        --impl_->point_count;
        impl_->ranked_voxels.erase(worst);
      }

      voxels.emplace(pending.local_key,
                     Entry{*pending.point, squared_distance});
      ++record.point_count;
      record.dirty = true;
      ++impl_->point_count;
      ++inserted_count;
      if (impl_->max_points > 0U) {
        impl_->ranked_voxels.insert(ranked);
      }
      if (update != nullptr) {
        ++update->inserted_count;
        record_update(pending.voxel_key, *pending.point);
      }
    }
    impl_->enforceResidentBudget();
  }
  return inserted_count;
}

bool ColoredGlobalMap::forEachPoint(
    const std::function<void(const ColoredPoint &)> &visitor) const {
  if (!visitor) {
    return false;
  }
  std::lock_guard<std::mutex> lock(impl_->mutex);
  for (const auto &chunk : impl_->chunks) {
    if (chunk.second.voxels != nullptr) {
      for (const auto &voxel : *chunk.second.voxels) {
        visitor(voxel.second.point);
      }
    } else if (!chunk.second.spilled ||
               !impl_->visitSpilled(chunk.first, visitor)) {
      return false;
    }
  }
  return impl_->point_count > 0U;
}

bool ColoredGlobalMap::getAllPoints(std::vector<ColoredPoint> &points) const {
  points.clear();
  points.reserve(size());
  return forEachPoint(
      [&points](const ColoredPoint &point) { points.push_back(point); });
}

bool ColoredGlobalMap::getChunkPoints(
    const std::int64_t x, const std::int64_t y, const std::int64_t z,
    const std::size_t max_points,
    std::vector<ColoredPoint> &points) const {
  points.clear();
  std::unique_lock<std::mutex> lock(impl_->mutex);
  const ChunkKey key{x, y, z};
  const auto iterator = impl_->chunks.find(key);
  if (iterator == impl_->chunks.end()) {
    return true;
  }
  const auto &record = iterator->second;
  if (max_points == 0U) {
    return false;
  }
  const std::size_t stride =
      std::max<std::size_t>(1U, (record.point_count + max_points - 1U) /
                                  max_points);
  points.reserve(std::min(record.point_count, max_points));
  std::size_t index = 0U;
  const auto append_sample = [&points, &index, stride](const ColoredPoint &point) {
    if (index++ % stride == 0U) {
      points.push_back(point);
    }
  };
  if (record.voxels != nullptr) {
    for (const auto &voxel : *record.voxels) {
      append_sample(voxel.second.point);
    }
    return true;
  }
  if (!record.spilled) {
    return false;
  }
  // Open the current spill file while holding the map lock, then read its
  // immutable file handle without blocking new colored observations.
  std::ifstream stream(impl_->chunkPath(key), std::ios::binary);
  std::uint32_t magic = 0U;
  std::uint64_t count = 0U;
  if (!stream || !readValue(stream, magic) || magic != kChunkFileMagic ||
      !readValue(stream, count)) {
    return false;
  }
  lock.unlock();
  for (std::uint64_t row = 0U; row < count; ++row) {
    LocalVoxelKey local;
    Entry entry;
    if (!readEntry(stream, local, entry)) {
      return false;
    }
    append_sample(entry.point);
  }
  return true;
}

std::size_t ColoredGlobalMap::size() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->point_count;
}

std::size_t ColoredGlobalMap::residentChunkCount() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->resident_chunk_count;
}

std::size_t ColoredGlobalMap::spilledChunkCount() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return static_cast<std::size_t>(std::count_if(
      impl_->chunks.begin(), impl_->chunks.end(), [](const auto &chunk) {
        return chunk.second.voxels == nullptr && chunk.second.spilled;
      }));
}

} // namespace itera_lio::mapping::color
