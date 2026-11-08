// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ITERA_LIO__LOOP__LOOP_VERIFIER_HPP_
#define ITERA_LIO__LOOP__LOOP_VERIFIER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "itera_lio/loop/loop_types.hpp"

namespace itera_lio::loop
{

struct LoopVerifierOptions
{
  std::string registration_engine{"manual_gicp"};
  std::size_t min_points{100U};
  std::size_t min_inliers{100U};
  double max_fitness{0.05};
  double max_translation_jump{20.0};
  double max_rotation_rad{1.0};
  double max_correspondence_distance{1.5};
  double inlier_distance{0.45};
  double gicp_voxel_size{0.30};
  double robust_kernel_scale{1.0};
  int num_threads{2};
  double min_inlier_ratio{0.4};
  double min_reciprocal_inlier_ratio{0.25};
  double min_geometry_eigenvalue_ratio{1e-4};
  double max_normal_eigenvalue_ratio{0.2};
  bool require_planar_normal_agreement{false};
  // Local revisit refinement must stay in the basin established by a global
  // loop. Do not use centroid initializations that can jump to another door.
  bool local_refinement{false};
  double source_voxel_size{0.35};
  double target_voxel_size{0.40};
  double max_gravity_tilt_correction_rad{0.10};
  double max_vertical_correction{1.0};
  int normal_neighbors{10};
  int max_iterations{40};
  std::size_t max_source_points{2000U};
  std::size_t max_target_points{10000U};
};

// Geometry verification is intentionally a separate boundary. A descriptor
// candidate is accepted only after robust GICP and the geometry gates below succeed.
class LoopVerifier
{
public:
  explicit LoopVerifier(const LoopVerifierOptions & options = {});

  LoopVerificationResult verify(
    const LoopCandidate & candidate,
    const KeyframeCloud & current,
    const KeyframeCloud & matched,
    const std::vector<common::MapPoint> & submap_points) const;

private:
  LoopVerifierOptions options_;
};

}  // namespace itera_lio::loop

#endif  // ITERA_LIO__LOOP__LOOP_VERIFIER_HPP_
