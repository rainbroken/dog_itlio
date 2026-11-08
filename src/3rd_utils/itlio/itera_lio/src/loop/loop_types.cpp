// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "itera_lio/loop/loop_types.hpp"

#include <cmath>

#include "itera_lio/backend/pose_graph.hpp"
#include "itera_lio/common/so3.hpp"

namespace itera_lio::loop
{

bool KeyframeCloud::isValid() const
{
  if (!keyframe.isValid() || points_I.empty()) {return false;}
  for (const auto & point : points_I) {
    if (!point.position.allFinite() || !std::isfinite(point.intensity)) {return false;}
  }
  return true;
}

bool LoopCandidate::isValid() const
{
  return current_id != matched_id && std::isfinite(descriptor_distance) &&
         descriptor_distance >= 0.0 && std::isfinite(yaw_offset_rad) &&
         std::isfinite(odometry_distance) && odometry_distance >= 0.0 &&
         std::isfinite(odometry_vertical_distance) &&
         odometry_vertical_distance >= 0.0 && std::isfinite(gravity_angle_rad) &&
         gravity_angle_rad >= 0.0;
}

bool LoopVerificationResult::isValid() const
{
  return current_id != matched_id && T_Ia_Ib.isValid() &&
         information.allFinite() &&
         (information - information.transpose()).norm() <= 1e-9 &&
         std::isfinite(fitness) && fitness >= 0.0 && accepted;
}

bool isStrongLoopEvidence(
  const LoopVerificationResult & verification,
  const StrongLoopEvidenceOptions & options)
{
  return options.enabled && verification.isValid() &&
         verification.inliers >= options.min_inliers &&
         verification.descriptor_distance <= options.max_descriptor_distance &&
         verification.fitness <= options.max_fitness &&
         verification.reciprocal_inlier_ratio >= options.min_reciprocal_inlier_ratio &&
         verification.geometry_condition_number > 0.0 &&
         verification.geometry_condition_number <= options.max_geometry_condition_number &&
         verification.gravity_residual_rad <= options.max_gravity_residual_rad &&
         verification.vertical_separation <= options.max_vertical_separation;
}

LoopConfirmationStatus LoopConfirmationGate::observe(
  const LoopVerificationResult & verification,
  const std::size_t required_count,
  const double translation_tolerance,
  const double rotation_tolerance_rad)
{
  if (last_current_id_ && *last_current_id_ == verification.current_id) {
    return LoopConfirmationStatus::kDuplicateCurrentKeyframe;
  }
  if (last_correction_) {
    const auto correction_delta = backend::composePoses(
      backend::inversePose(*last_correction_), verification.global_correction);
    if (correction_delta.p_WI.norm() <= translation_tolerance &&
      common::LogSO3(correction_delta.R_WI).norm() <= rotation_tolerance_rad)
    {
      ++count_;
    } else {
      count_ = 1U;
    }
  } else {
    count_ = 1U;
  }
  last_correction_ = verification.global_correction;
  last_current_id_ = verification.current_id;
  return count_ >= required_count ?
         LoopConfirmationStatus::kConfirmed : LoopConfirmationStatus::kAwaiting;
}

}  // namespace itera_lio::loop
