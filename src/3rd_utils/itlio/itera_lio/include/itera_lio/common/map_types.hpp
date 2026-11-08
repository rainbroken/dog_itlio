// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ITERA_LIO__COMMON__MAP_TYPES_HPP_
#define ITERA_LIO__COMMON__MAP_TYPES_HPP_

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace itera_lio::common
{

struct MapPoint
{
  Eigen::Vector3f position{Eigen::Vector3f::Zero()};
  float intensity{0.0F};
};

struct MapBox
{
  Eigen::Vector3f min{Eigen::Vector3f::Zero()};
  Eigen::Vector3f max{Eigen::Vector3f::Zero()};
};

struct MapInsertReport
{
  std::size_t input_count{0U};
  std::size_t inserted_count{0U};
  std::size_t replaced_count{0U};
  std::size_t rejected_count{0U};
  std::size_t compacted_count{0U};
  std::int64_t net_size_delta{0};
  std::vector<MapPoint> upserted_points;
  std::vector<MapPoint> removed_points;
};

}  // namespace itera_lio::common

#endif  // ITERA_LIO__COMMON__MAP_TYPES_HPP_
