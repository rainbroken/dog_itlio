// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0
#include "itera_lio/output/static_map_filter.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace itera_lio::output {
namespace {
using Key = common::VoxelKey;
using Hash = common::VoxelKeyHash;
Key keyOf(const Eigen::Vector3d &point, float size) {
  Key key;
  if (!common::makeVoxelKey(point, size, key)) {
    throw std::invalid_argument("non-finite or out-of-range filter coordinate");
  }
  return key;
}
}

void StaticMapFilterOptions::validate() const {
  if (!std::isfinite(evidence_voxel_size) || evidence_voxel_size < 0.02F ||
      !std::isfinite(max_range) || max_range <= 0 || max_range > 200 ||
      !std::isfinite(endpoint_margin) || endpoint_margin < 0 ||
      !std::isfinite(margin_per_meter) || margin_per_meter < 0 ||
      !std::isfinite(max_endpoint_margin) || max_endpoint_margin < endpoint_margin ||
      !std::isfinite(observer_voxel_size) || observer_voxel_size < 0.05 ||
      !std::isfinite(min_scan_interval) || min_scan_interval < 0 ||
      min_free_observations < 2) {
    throw std::invalid_argument("invalid static map filter options");
  }
}

StaticMapFilter::StaticMapFilter(StaticMapFilterOptions options) : options_(options) {
  options_.validate();
}

void StaticMapFilter::addCandidate(const Eigen::Vector3d &point) {
  evidence_.try_emplace(keyOf(point, options_.evidence_voxel_size));
}

void StaticMapFilter::observe(const Eigen::Vector3d &origin,
                              const std::vector<Eigen::Vector3d> &points) {
  const auto observer = keyOf(origin, static_cast<float>(options_.observer_voxel_size));
  std::unordered_set<Key, Hash> hits, misses;
  // One vote per voxel per scan, independent of point density. For free-space
  // work keep one actual ray per endpoint voxel, preferring its closest return.
  std::unordered_map<Key, Eigen::Vector3d, Hash> rays;
  for (const auto &point : points) {
    const double range = (point - origin).norm();
    if (!std::isfinite(range) || range < 1e-9) {continue;}
    const auto endpoint = keyOf(point, options_.evidence_voxel_size);
    if (range <= options_.max_range) {hits.insert(endpoint);}
    const auto found = rays.find(endpoint);
    if (found == rays.end()) {rays.emplace(endpoint, point);}
    else if (range < (found->second - origin).norm()) {found->second = point;}
  }
  for (const auto &entry : rays) {
    const Eigen::Vector3d ray = entry.second - origin;
    const double range = ray.norm();
    const double margin = std::min(options_.max_endpoint_margin,
        options_.endpoint_margin + options_.margin_per_meter * range);
    // A distant return can establish free space only within max_range. It
    // cannot establish an occupied endpoint at the artificial truncation.
    const double end_t = std::max(0.0, std::min(options_.max_range, range - margin) / range);
    Key current = keyOf(origin, options_.evidence_voxel_size);
    const Key endpoint = entry.first;
    const int step[3] = {ray.x() > 0 ? 1 : (ray.x() < 0 ? -1 : 0),
                         ray.y() > 0 ? 1 : (ray.y() < 0 ? -1 : 0),
                         ray.z() > 0 ? 1 : (ray.z() < 0 ? -1 : 0)};
    double next[3], delta[3];
    const std::int64_t indices[3] = {current.x, current.y, current.z};
    for (int axis = 0; axis < 3; ++axis) {
      if (step[axis] == 0) {
        next[axis] = delta[axis] = std::numeric_limits<double>::infinity();
      } else {
        const double boundary = (indices[axis] + (step[axis] > 0 ? 1 : 0)) *
                                static_cast<double>(options_.evidence_voxel_size);
        next[axis] = (boundary - origin[axis]) / ray[axis];
        delta[axis] = options_.evidence_voxel_size / std::abs(ray[axis]);
      }
    }
    while (!(current == endpoint)) {
      const double t = std::min({next[0], next[1], next[2]});
      if (!std::isfinite(t) || t >= end_t) {break;}
      for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(next[axis] - t) <= 1e-12 * std::max(1.0, std::abs(t))) {
          if (axis == 0) {current.x += step[axis];}
          else if (axis == 1) {current.y += step[axis];}
          else {current.z += step[axis];}
          next[axis] += delta[axis];
        }
      }
      if (!(current == endpoint) && evidence_.count(current)) {misses.insert(current);}
    }
  }
  // Occupied evidence wins within the same scan, protecting edges and occluders.
  for (const auto &key : hits) {
    misses.erase(key);
    const auto found = evidence_.find(key);
    if (found != evidence_.end()) {++found->second.hits;}
  }
  for (const auto &key : misses) {
    auto &value = evidence_.at(key);
    if (value.misses == 0) {value.first_free_observer = observer;}
    else if (!(value.first_free_observer == observer)) {value.multiple_free_observers = true;}
    ++value.misses;
  }
}

bool StaticMapFilter::rejected(const Evidence &value) const {
  // Reference: p(hit)=.70, p(miss)=.40, occupied threshold=.50. Use aggregate
  // log odds without temporal clamping: result does not depend on frame order.
  const double log_odds = value.hits * std::log(0.70 / 0.30) +
                          value.misses * std::log(0.40 / 0.60);
  return value.hits > 0 && value.misses >= options_.min_free_observations &&
         value.multiple_free_observers && log_odds < 0;
}

bool StaticMapFilter::keep(const Eigen::Vector3d &point) const {
  const auto found = evidence_.find(keyOf(point, options_.evidence_voxel_size));
  return found == evidence_.end() || !rejected(found->second);
}

std::size_t StaticMapFilter::rejectedCount() const {
  return std::count_if(evidence_.begin(), evidence_.end(),
      [this](const auto &entry) {return rejected(entry.second);});
}
bool StaticMapFilter::stable(const Eigen::Vector3d &point) const {
  const auto found = evidence_.find(keyOf(point, options_.evidence_voxel_size));
  return found != evidence_.end() && found->second.hits >= 2 && !rejected(found->second);
}
}  // namespace itera_lio::output
