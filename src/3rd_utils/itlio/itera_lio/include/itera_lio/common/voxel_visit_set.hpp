// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0
#ifndef ITERA_LIO__COMMON__VOXEL_VISIT_SET_HPP_
#define ITERA_LIO__COMMON__VOXEL_VISIT_SET_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>
#include "itera_lio/common/voxel.hpp"

namespace itera_lio::common
{
// Scratch storage for first-point voxel selection. Open addressing avoids
// allocating and freeing an unordered_set node for every point of every scan.
// One owner/thread must use the scratch table at a time.
class VoxelVisitSet
{
public:
  void reset(const std::size_t max_points)
  {
    std::size_t capacity = 8U;
    while (capacity / 2U < max_points) {capacity *= 2U;}
    // A generation invalidates old membership without clearing the whole
    // table on every scan. Keep the high-water capacity for buffer reuse.
    if (keys_.size() < capacity) {
      keys_.resize(capacity);
      generations_.resize(capacity);
    }
    mask_ = keys_.size() - 1U;
    if (generation_ == std::numeric_limits<std::uint32_t>::max()) {
      std::fill(generations_.begin(), generations_.end(), 0U);
      generation_ = 1U;
    } else {
      ++generation_;
    }
  }

  bool insert(const VoxelKey & key)
  {
    // Membership only: bucket order never determines emitted point order.
    // Fold all three signed coordinates as unsigned before one avalanche.
    std::uint64_t hash = static_cast<std::uint64_t>(key.x) * 0x9e3779b185ebca87ULL ^
      static_cast<std::uint64_t>(key.y) * 0xc2b2ae3d27d4eb4fULL ^
      static_cast<std::uint64_t>(key.z) * 0x165667b19e3779f9ULL;
    hash ^= hash >> 33U;
    hash *= 0xff51afd7ed558ccdULL;
    hash ^= hash >> 33U;
    std::size_t slot = static_cast<std::size_t>(hash) & mask_;
    while (generations_[slot] == generation_) {
      if (keys_[slot] == key) {return false;}
      slot = (slot + 1U) & mask_;
    }
    keys_[slot] = key;
    generations_[slot] = generation_;
    return true;
  }

private:
  // Most probes encounter an unused slot. Keep generation tags compact so
  // that testing emptiness does not fetch the full 24-byte voxel key.
  std::vector<VoxelKey> keys_;
  std::vector<std::uint32_t> generations_;
  std::size_t mask_{0U};
  std::uint32_t generation_{0U};
};
}  // namespace itera_lio::common
#endif
