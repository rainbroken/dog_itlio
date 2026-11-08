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

#ifndef ITERA_LIO__FRONTEND__SCAN_TO_MAP_HPP_
#define ITERA_LIO__FRONTEND__SCAN_TO_MAP_HPP_

#include <Eigen/Core>

#include <vector>
#include <cstdint>
#include <utility>
#include <unordered_map>

#include "itera_lio/common/geometry.hpp"
#include "itera_lio/common/observation_types.hpp"
#include "itera_lio/frontend/map_interfaces.hpp"
#include "itera_lio/frontend/plane_fitter.hpp"
#include "itera_lio/sensor/lidar_frame.hpp"

namespace itera_lio::frontend
{

struct ScanToMapOptions
{
  int knn{5};
  int num_threads{3};
  float max_neighbor_distance{3.0F};
  PlaneFitterOptions plane;
  double residual_threshold{0.5};
  // Commit only geometrically consistent observations; correction uses a wider gate.
  double map_insert_residual_threshold{0.5};
  bool enable_map_insert_quality_gate{false};
  bool require_converged_for_map_insert{false};
  double map_insert_max_mean_residual{0.05};
  double map_insert_max_rotation_correction{0.15};
  double map_insert_max_translation_correction{0.3};
  double map_insert_min_effective_ratio{0.2};
  std::size_t max_effective_points{1000U};
  // Upper bound on expensive KNN/plane-fit attempts per scan. Candidates are
  // spread across the scan so this does not bias observations toward one
  // unordered voxel-map segment.
  std::size_t max_candidate_points{2000U};
  // Geometrically balanced cap for points committed to the map. This is
  // independent from both KNN attempts and correction observations.
  std::size_t max_map_insert_points{1000U};
  bool adaptive_candidates{false};
  double reuse_max_displacement{0.05};
  // Accepted observations are selected from spatial cells in a deterministic
  // round-robin over normal bins instead of stopping at the first N points.
  double selection_voxel_size{1.0};
  int normal_azimuth_bins{8};
  int normal_elevation_bins{4};
};

using PlaneObservation = common::PlaneObservation;
using ScanToMapReport = common::RegistrationReport;

// Owned by one scan correction only; the map must remain unchanged during its lifetime.
class ScanToMapIterationCache
{
public:
  struct Entry
  {
    Eigen::Vector3d anchor;
    Plane plane;
    double farthest_distance{0.0};
  };

  // Slots are proportional candidate base indices, an injective mapping
  // to point indices within this scan. Bound dense memory independently of
  // scan size; exceptional candidate budgets retain the original hash path.
  static constexpr std::size_t kMaximumDenseEntries = 65536U;
  std::unordered_map<std::size_t, Entry> planes;
  std::vector<Entry> dense_planes;
  std::vector<std::uint8_t> dense_valid;
  bool use_dense{false};

  void reset(const std::size_t candidate_count)
  {
    planes.clear();
    use_dense = candidate_count <= kMaximumDenseEntries;
    dense_planes.resize(use_dense ? candidate_count : 0U);
    dense_valid.assign(use_dense ? candidate_count : 0U, 0U);
  }

  const Entry * findPlane(const std::size_t point_index, const std::size_t slot) const
  {
    if (use_dense) {return dense_valid[slot] ? &dense_planes[slot] : nullptr;}
    const auto found = planes.find(point_index);
    return found == planes.end() ? nullptr : &found->second;
  }

  void erasePlane(const std::size_t point_index, const std::size_t slot)
  {
    if (use_dense) {dense_valid[slot] = 0U;} else {planes.erase(point_index);}
  }

  void storePlane(const std::size_t point_index, const std::size_t slot, const Entry &entry)
  {
    if (use_dense) {
      dense_planes[slot] = entry;
      dense_valid[slot] = 1U;
    } else {
      planes[point_index] = entry;
    }
  }
  std::size_t iteration{0U};
  std::size_t candidate_budget{0U};
  std::size_t previous_effective{0U};
  std::size_t previous_accepted{0U};
};

ScanToMapReport buildScanToMapObservations(
  const sensor::LidarFrame & scan,
  const common::Pose & pose,
  const common::FixedExtrinsics & extrinsics,
  const NeighborMap & map,
  const ScanToMapOptions & options,
  ScanToMapIterationCache * cache = nullptr);

}  // namespace itera_lio::frontend

#endif  // ITERA_LIO__FRONTEND__SCAN_TO_MAP_HPP_
