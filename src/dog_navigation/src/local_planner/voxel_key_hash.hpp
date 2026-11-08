#pragma once
#include <array>
#include <functional>

namespace dog_navigation {
struct VoxelKeyHash {
  size_t operator()(const std::array<int, 3> &key) const noexcept {
    size_t value = 0;
    for (int axis : key)
      value ^= std::hash<int>{}(axis) + 0x9e3779b97f4a7c15ULL + (value << 6) +
               (value >> 2);
    return value;
  }
};
} // namespace dog_navigation
