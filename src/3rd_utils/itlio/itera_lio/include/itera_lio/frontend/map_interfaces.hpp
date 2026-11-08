// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ITERA_LIO__FRONTEND__MAP_INTERFACES_HPP_
#define ITERA_LIO__FRONTEND__MAP_INTERFACES_HPP_

#include <Eigen/Core>

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "itera_lio/common/map_types.hpp"

namespace itera_lio::frontend
{

class NeighborMap
{
public:
  struct ConcurrentReadGuard {virtual ~ConcurrentReadGuard() = default;};
  virtual ~NeighborMap() = default;
  // Call on the map owner thread, with no concurrent mutations/other queries.
  // Keep the guard alive until all query workers have joined.
  virtual std::unique_ptr<ConcurrentReadGuard> prepareConcurrentRead() const {return {};}
  virtual bool concurrentReadPrepared() const {return false;}

  virtual bool nearestSearch(
    const common::MapPoint & point, int k, float max_distance,
    std::vector<common::MapPoint> & neighbors,
    std::vector<float> & squared_distances) const = 0;
  virtual bool nearestSearchPositions(
    const Eigen::Vector3f & point, int k, float max_distance,
    std::vector<Eigen::Vector3d> & neighbors,
    std::vector<float> & squared_distances) const = 0;
};

class MapStore : public NeighborMap
{
public:
  ~MapStore() override = default;

  virtual void build(const std::vector<common::MapPoint> & points) = 0;
  virtual common::MapInsertReport addPoints(
    const std::vector<common::MapPoint> & points, bool downsample) = 0;
  virtual int deleteBoxes(
    const std::vector<common::MapBox> & boxes,
    std::vector<common::MapPoint> * deleted_points = nullptr) = 0;
  virtual std::size_t countPoints(
    const common::MapBox & box,
    std::size_t max_count = std::numeric_limits<std::size_t>::max()) const = 0;
  virtual bool getAllPoints(std::vector<common::MapPoint> & points) const = 0;
  virtual std::size_t size() const = 0;
};

}  // namespace itera_lio::frontend

#endif  // ITERA_LIO__FRONTEND__MAP_INTERFACES_HPP_
