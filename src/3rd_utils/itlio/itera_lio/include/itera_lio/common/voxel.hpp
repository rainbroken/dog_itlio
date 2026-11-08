// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0

#ifndef ITERA_LIO__COMMON__VOXEL_HPP_
#define ITERA_LIO__COMMON__VOXEL_HPP_

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>

namespace itera_lio::common
{

struct VoxelKey
{
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto mix = [](std::uint64_t value) {
        value += 0x9e3779b97f4a7c15ULL;
        value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
        value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
        return value ^ (value >> 31U);
      };
    return static_cast<std::size_t>(mix(
      mix(static_cast<std::uint64_t>(key.x)) ^
      (mix(static_cast<std::uint64_t>(key.y)) << 1U) ^
      (mix(static_cast<std::uint64_t>(key.z)) << 7U)));
  }
};

inline bool makeVoxelKey(
  const Eigen::Vector3f & point, const float voxel_size, VoxelKey & key)
{
  if (!point.allFinite() || !std::isfinite(voxel_size) || voxel_size <= 0.0F) {
    return false;
  }
  const Eigen::Array3d floored = (point.cast<double>().array() / voxel_size).floor();
  constexpr double kMin = static_cast<double>(std::numeric_limits<std::int64_t>::min());
  constexpr double kMax = static_cast<double>(std::numeric_limits<std::int64_t>::max());
  if (!floored.allFinite() || (floored < kMin).any() || (floored > kMax).any()) {
    return false;
  }
  key = {static_cast<std::int64_t>(floored.x()), static_cast<std::int64_t>(floored.y()),
    static_cast<std::int64_t>(floored.z())};
  return true;
}

inline bool makeVoxelKey(
  const Eigen::Vector3d & point, const float voxel_size, VoxelKey & key)
{
  if (!point.allFinite() || !std::isfinite(voxel_size) || voxel_size <= 0.0F) {
    return false;
  }
  const Eigen::Array3d floored = (point.array() / static_cast<double>(voxel_size)).floor();
  constexpr double kMin = static_cast<double>(std::numeric_limits<std::int64_t>::min());
  constexpr double kMax = static_cast<double>(std::numeric_limits<std::int64_t>::max());
  if (!floored.allFinite() || (floored < kMin).any() || (floored > kMax).any()) {
    return false;
  }
  key = {static_cast<std::int64_t>(floored.x()), static_cast<std::int64_t>(floored.y()),
    static_cast<std::int64_t>(floored.z())};
  return true;
}

inline Eigen::Vector3f voxelCenter(const VoxelKey & key, const float voxel_size)
{
  return (Eigen::Vector3f(
    static_cast<float>(key.x), static_cast<float>(key.y), static_cast<float>(key.z)) +
    Eigen::Vector3f::Constant(0.5F)) * voxel_size;
}

}  // namespace itera_lio::common

#endif  // ITERA_LIO__COMMON__VOXEL_HPP_
