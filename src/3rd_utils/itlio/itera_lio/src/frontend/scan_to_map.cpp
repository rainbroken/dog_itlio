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

#include "itera_lio/frontend/scan_to_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <unordered_map>

namespace itera_lio::frontend
{

namespace
{

struct SpatialKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;
  int normal_bin;

  bool operator==(const SpatialKey & other) const
  {
    return x == other.x && y == other.y && z == other.z && normal_bin == other.normal_bin;
  }
};

struct SpatialKeyHash
{
  std::size_t operator()(const SpatialKey & key) const
  {
    const auto mix = [](const std::int64_t value) {
        return std::hash<std::int64_t>{}(value + static_cast<std::int64_t>(0x9e3779b9));
      };
    return mix(key.x) ^ (mix(key.y) << 1U) ^ (mix(key.z) << 2U) ^
           (std::hash<int>{}(key.normal_bin) << 3U);
  }
};

int normalBin(
  const Eigen::Vector3d & input, const int azimuth_bins, const int elevation_bins)
{
  Eigen::Vector3d normal = input.normalized();
  // Plane eigenvectors have an arbitrary sign. Canonicalize the sign so the
  // same physical normal does not occupy two selection bins.
  Eigen::Index dominant = 0;
  normal.cwiseAbs().maxCoeff(&dominant);
  if (normal[dominant] < 0.0) {
    normal = -normal;
  }
  const double azimuth = std::atan2(normal.y(), normal.x());
  const double elevation = std::asin(std::clamp(normal.z(), -1.0, 1.0));
  constexpr double kPi = 3.14159265358979323846;
  const double azimuth_fraction = (azimuth + kPi) / (2.0 * kPi);
  const double elevation_fraction = (elevation + 0.5 * kPi) / kPi;
  const int azimuth_index = std::min(
    azimuth_bins - 1, static_cast<int>(azimuth_fraction * azimuth_bins));
  const int elevation_index = std::min(
    elevation_bins - 1, static_cast<int>(elevation_fraction * elevation_bins));
  return elevation_index * azimuth_bins + azimuth_index;
}

std::vector<PlaneObservation> selectDiverseObservations(
  const std::vector<PlaneObservation> & candidates, const ScanToMapOptions & options,
  const std::size_t max_points)
{
  if (candidates.size() <= max_points) {
    return candidates;
  }
  const int normal_bin_count = options.normal_azimuth_bins * options.normal_elevation_bins;
  std::unordered_map<SpatialKey, std::size_t, SpatialKeyHash> cell_representatives;
  cell_representatives.reserve(candidates.size());
  const auto center_distance = [&](const PlaneObservation & candidate) {
    const Eigen::Array3d cell =
      (candidate.point_W.array() / options.selection_voxel_size).floor();
    const Eigen::Vector3d difference = (candidate.point_W.array() -
      (cell + 0.5) * options.selection_voxel_size).matrix();
    // Compare coverage in the plane tangent, not distance along its normal:
    // a 3-D center preference can itself select one sign of range noise.
    return (difference - candidate.normal_W * candidate.normal_W.dot(difference)).squaredNorm();
  };
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    const auto & candidate = candidates[index];
    const SpatialKey key{
      static_cast<std::int64_t>(std::floor(candidate.point_W.x() / options.selection_voxel_size)),
      static_cast<std::int64_t>(std::floor(candidate.point_W.y() / options.selection_voxel_size)),
      static_cast<std::int64_t>(std::floor(candidate.point_W.z() / options.selection_voxel_size)),
      normalBin(candidate.normal_W, options.normal_azimuth_bins, options.normal_elevation_bins)};
    const auto found = cell_representatives.find(key);
    if (found == cell_representatives.end() ||
      center_distance(candidate) < center_distance(candidates[found->second]))
    {
      cell_representatives[key] = index;
    }
  }

  std::vector<std::vector<std::size_t>> bins(static_cast<std::size_t>(normal_bin_count));
  for (const auto & entry : cell_representatives) {
    const std::size_t index = entry.second;
    // The representative key already contains this exact normal bin.
    const int bin = entry.first.normal_bin;
    bins[static_cast<std::size_t>(bin)].push_back(index);
  }
  for (auto & bin : bins) {
    std::sort(
      bin.begin(), bin.end(), [&](const std::size_t lhs, const std::size_t rhs) {
        // A spatial hash gives a reproducible spread without selecting the
        // smallest innovations. Residual-ranked sampling censors the signed
        // correction signal before the robust estimator can use it.
        const auto priority = [&](const std::size_t index) {
          const auto & point = candidates[index].point_W;
          const auto mix = [](std::uint64_t value) {
            value ^= value >> 30U;
            value *= UINT64_C(0xbf58476d1ce4e5b9);
            value ^= value >> 27U;
            value *= UINT64_C(0x94d049bb133111eb);
            return value ^ (value >> 31U);
          };
          std::uint64_t hash = UINT64_C(0x9e3779b97f4a7c15);
          for (int axis = 0; axis < 3; ++axis) {
            const auto cell = static_cast<std::int64_t>(
              std::floor(point[axis] / options.selection_voxel_size));
            hash = mix(hash ^ mix(static_cast<std::uint64_t>(cell)));
          }
          return hash;
        };
        const auto a = priority(lhs), b = priority(rhs);
        return a != b ? a < b : lhs < rhs;
      });
  }

  std::vector<PlaneObservation> selected;
  selected.reserve(max_points);
  std::size_t cursor = 0U;
  while (selected.size() < max_points && cursor < cell_representatives.size()) {
    bool added = false;
    for (auto & bin : bins) {
      if (cursor >= bin.size()) {
        continue;
      }
      selected.push_back(candidates[bin[cursor]]);
      added = true;
      if (selected.size() >= max_points) {
        break;
      }
    }
    ++cursor;
    if (!added && cursor >= cell_representatives.size()) {
      break;
    }
  }
  return selected;
}

}  // namespace

ScanToMapReport buildScanToMapObservations(
  const sensor::LidarFrame & scan,
  const common::Pose & pose,
  const common::FixedExtrinsics & extrinsics,
  const NeighborMap & map,
  const ScanToMapOptions & options,
  ScanToMapIterationCache * cache)
{
  ScanToMapReport report;
  report.input_points = scan.points.size();
  if (!scan.isValid() || !pose.R_WI.allFinite() || !pose.p_WI.allFinite() ||
    !extrinsics.isValid() || options.knn < 3 || options.num_threads < 1 ||
    !std::isfinite(options.residual_threshold) || options.residual_threshold <= 0.0 ||
    options.max_effective_points == 0U || options.max_candidate_points == 0U ||
    options.max_map_insert_points == 0U ||
    !std::isfinite(options.map_insert_residual_threshold) ||
    options.map_insert_residual_threshold <= 0.0 ||
    !std::isfinite(options.reuse_max_displacement) || options.reuse_max_displacement < 0.0 ||
    !std::isfinite(options.selection_voxel_size) || options.selection_voxel_size <= 0.0 ||
    options.normal_azimuth_bins <= 0 || options.normal_elevation_bins <= 0)
  {
    report.rejected_points = report.input_points;
    return report;
  }
  const std::size_t candidate_count = std::min(
    scan.points.size(), options.max_candidate_points);
  std::size_t budget = candidate_count;
  if (cache != nullptr) {
    if (cache->iteration == 0U) {
      cache->reset(candidate_count);
      cache->candidate_budget = candidate_count;
    } else {
      if (options.adaptive_candidates &&
        cache->previous_effective >= options.max_effective_points &&
        cache->previous_accepted >= (cache->candidate_budget + 1U) / 2U)
      {
        cache->candidate_budget = std::max(
          std::min(
            candidate_count, options.max_effective_points >= candidate_count / 3U ?
            candidate_count : options.max_effective_points * 3U),
          cache->candidate_budget - cache->candidate_budget / 4U);
      }
    }
    budget = std::min(candidate_count, cache->candidate_budget);
  }
  report.candidate_attempts = budget;
  struct CandidateResult {
    PlaneObservation observation;
    bool accepted{false};
    std::size_t reused_planes{0U}, knn_queries{0U}, rejected_points{0U};
    double knn_time_ms{0.0}, plane_fit_time_ms{0.0};
  };
  // Retain bounded scratch on the owning processing thread. Bind a normal
  // reference before entering OpenMP so all workers address the owner's
  // storage, rather than resolving thread-local storage on worker threads.
  constexpr std::size_t kMaxRetainedCandidates = 16384U;
  thread_local std::vector<CandidateResult> retained_results;
  std::vector<CandidateResult> large_scan_results;
  auto & results = budget <= kMaxRetainedCandidates ? retained_results : large_scan_results;
  results.resize(budget);
  // Dense cache slots are injective base indices. The hash fallback cannot
  // support independent writes, so keep that exceptional path serial.
  const int workers = map.concurrentReadPrepared() &&
    (cache == nullptr || cache->use_dense) && budget >= 256U ?
    std::min(options.num_threads, 3) : 1;
  #pragma omp parallel num_threads(workers) if(workers > 1)
  {
    // Each worker owns these scratch buffers. Keep their capacity across
    // registration iterations; the map query clears and fills them per call.
    thread_local std::vector<float> distances;
    thread_local std::vector<Eigen::Vector3d> neighbor_positions;
    distances.reserve(static_cast<std::size_t>(options.knn));
    neighbor_positions.reserve(static_cast<std::size_t>(options.knn));
    const auto process_candidate = [&](const std::size_t attempted) {
      auto & item = results[attempted];
      // A rejected entry's old observation is never read. Reset only live
      // status/counters; accepted entries overwrite every observation field.
      item.accepted = false;
      item.reused_planes = item.knn_queries = item.rejected_points = 0U;
      item.knn_time_ms = item.plane_fit_time_ms = 0.0;
    // Use proportional indexing instead of an integer stride so the final
    // candidate is included even when scan size is not divisible by the cap.
    const std::size_t base_index = attempted * candidate_count / budget;
    const std::size_t point_index = base_index * scan.points.size() / candidate_count;
    const auto & point = scan.points[point_index];
    const Eigen::Vector3d point_I = extrinsics.R_IL * point.position + extrinsics.p_IL;
    const Eigen::Vector3d point_W = pose.R_WI * point_I + pose.p_WI;
    Plane plane;
    bool reused = false;
    if (cache != nullptr && cache->iteration > 0U) {
      const auto *found = cache->findPlane(point_index, base_index);
      if (found != nullptr) {
        const double displacement = (point_W - found->anchor).norm();
        if (std::isfinite(displacement) && displacement <= options.reuse_max_displacement &&
          found->farthest_distance + displacement < options.max_neighbor_distance &&
          std::abs(found->plane.normal.dot(point_W - found->plane.centroid)) <
          options.residual_threshold * 0.8)
        {
          plane = found->plane;
          reused = true;
          ++item.reused_planes;
        }
      }
    }
    if (!reused) {
      const auto knn_start = std::chrono::steady_clock::now();
      ++item.knn_queries;
      const bool knn_ok = map.nearestSearchPositions(
        point_W.cast<float>(), options.knn, options.max_neighbor_distance,
        neighbor_positions, distances);
      item.knn_time_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - knn_start).count();
      if (!knn_ok || neighbor_positions.size() < static_cast<std::size_t>(options.knn)) {
        if (cache != nullptr) {cache->erasePlane(point_index, base_index);}
        ++item.rejected_points;
        return;
      }
      const auto plane_start = std::chrono::steady_clock::now();
      if (!fitPlane(neighbor_positions, options.plane, plane)) {
        if (cache != nullptr) {cache->erasePlane(point_index, base_index);}
        item.plane_fit_time_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - plane_start).count();
        ++item.rejected_points;
        return;
      }
      item.plane_fit_time_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - plane_start).count();
      if (cache != nullptr && distances.size() >= static_cast<std::size_t>(options.knn)) {
        const float farthest_squared = *std::max_element(distances.begin(), distances.end());
        if (std::isfinite(farthest_squared) && farthest_squared >= 0.0F) {
          cache->storePlane(
            point_index, base_index, {point_W, plane, std::sqrt(farthest_squared)});
        }
      }
    }
    const double residual = plane.normal.dot(point_W - plane.centroid);
    if (!std::isfinite(residual) || std::abs(residual) > options.residual_threshold) {
      ++item.rejected_points;
      return;
    }
    PlaneObservation observation;
    observation.point_I = point_I;
    observation.point_W = point_W;
    observation.normal_W = plane.normal;
    observation.centroid_W = plane.centroid;
    observation.residual = residual;
    observation.intensity = point.intensity;
    observation.has_point_I = true;
    if (options.plane.enable_quality_checks) {
      const double normal_ratio = plane.eigenvalues.x() / plane.eigenvalues.y();
      observation.plane_weight = std::clamp(1.0 - normal_ratio, 0.1, 1.0);
    }
    item.observation = observation;
    item.accepted = true;
    };
    #pragma omp for schedule(static)
    for (std::size_t attempted = 0U; attempted < budget; ++attempted) {
      process_candidate(attempted);
    }
  }
  std::vector<PlaneObservation> candidates;
  candidates.reserve(budget);
  double knn_time_ms = 0.0, plane_fit_time_ms = 0.0;
  // Deterministic compaction and summation preserve candidate/selection order.
  for (const auto & item : results) {
    report.reused_planes += item.reused_planes;
    report.knn_queries += item.knn_queries;
    report.rejected_points += item.rejected_points;
    knn_time_ms += item.knn_time_ms;
    plane_fit_time_ms += item.plane_fit_time_ms;
    if (item.accepted) {candidates.push_back(item.observation);}
  }
  const auto selection_start = std::chrono::steady_clock::now();
  const std::size_t accepted_count = candidates.size();
  report.accepted_candidates = accepted_count;
  const std::size_t larger_budget = std::max(
    options.max_effective_points, options.max_map_insert_points);
  if (candidates.size() > larger_budget) {
    // Both original calls take the binning/sorting branch. Their round-robin
    // order is independent of the budget, so the shorter output is a prefix.
    // Both calls use the same deterministic spatial priority.
    auto selected = selectDiverseObservations(candidates, options, larger_budget);
    if (options.max_effective_points <= options.max_map_insert_points) {
      const auto count = std::min(options.max_effective_points, selected.size());
      report.observations.assign(selected.begin(), selected.begin() + count);
      report.insertion_candidates = std::move(selected);
    } else {
      const auto count = std::min(options.max_map_insert_points, selected.size());
      report.insertion_candidates.assign(selected.begin(), selected.begin() + count);
      report.observations = std::move(selected);
    }
  } else {
    // Preserve the original unfiltered order when a budget covers all input.
    report.observations = selectDiverseObservations(
      candidates, options, options.max_effective_points);
    report.insertion_candidates = selectDiverseObservations(
      candidates, options, options.max_map_insert_points);
  }
  report.selection_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - selection_start).count();
  report.knn_time_ms = knn_time_ms;
  report.plane_fit_time_ms = plane_fit_time_ms;
  report.effective_points = report.observations.size();
  if (cache != nullptr) {
    cache->previous_effective = report.effective_points;
    cache->previous_accepted = accepted_count;
    ++cache->iteration;
  }
  report.mean_abs_residual = report.effective_points > 0U ?
    [&]() {
    double selected_sum = 0.0;
    for (const auto & observation : report.observations) {
      selected_sum += std::abs(observation.residual);
    }
    return selected_sum / static_cast<double>(report.effective_points);
  } () : 0.0;
  return report;
}

}  // namespace itera_lio::frontend
