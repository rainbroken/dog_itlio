// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ITERA_LIO__LOOP__LOOP_TYPES_HPP_
#define ITERA_LIO__LOOP__LOOP_TYPES_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "itera_lio/backend/pose_graph.hpp"
#include "itera_lio/common/ids.hpp"
#include "itera_lio/common/map_types.hpp"

namespace itera_lio::loop
{

// Points are kept in the keyframe IMU frame. They can therefore be transformed
// again after pose-graph optimization without baking the old world pose in.
struct KeyframeCloud
{
  backend::Keyframe keyframe;
  std::vector<common::MapPoint> points_I;
  std::vector<float> descriptor;
  std::vector<float> ring_key;

  bool isValid() const;
};

struct LoopCandidate
{
  common::KeyframeId current_id{0U};
  common::KeyframeId matched_id{0U};
  double descriptor_distance{0.0};
  double yaw_offset_rad{0.0};
  double odometry_distance{0.0};
  double odometry_vertical_distance{0.0};
  double gravity_angle_rad{0.0};

  bool isValid() const;
};

struct LoopVerificationResult
{
  common::KeyframeId current_id{0U};
  common::KeyframeId matched_id{0U};
  common::Pose T_Ia_Ib;
  common::Pose global_correction;
  double fitness{0.0};
  std::size_t inliers{0U};
  double reciprocal_inlier_ratio{0.0};
  double geometry_condition_number{0.0};
  double translation_correction{0.0};
  double rotation_correction{0.0};
  double gravity_residual_rad{0.0};
  double vertical_separation{0.0};
  double descriptor_distance{0.0};
  double odometry_distance{0.0};
  double odometry_vertical_distance{0.0};
  double gravity_angle_rad{0.0};
  // Source points excluded from registration. Target is the same local map,
  // so this is a held-out consistency check, not independent ground truth.
  std::size_t audit_points{0U};
  std::size_t audit_inliers{0U};
  double audit_overlap{0.0};
  double audit_nearest_rmse{0.0};
  std::size_t audit_plane_points{0U};
  double audit_plane_p90{0.0};
  std::size_t audit_wall_points{0U};
  double audit_wall_p90{0.0};
  std::size_t audit_horizontal_points{0U};
  double audit_horizontal_p90{0.0};
  backend::PoseInformationMatrix information{backend::PoseInformationMatrix::Identity()};
  bool accepted{false};
  const char * initialization{"none"};
  const char * rejection_reason{"not_verified"};

  bool isValid() const;
};

struct StrongLoopEvidenceOptions
{
  bool enabled{false};
  std::size_t min_inliers{150U};
  double max_descriptor_distance{0.20};
  double max_fitness{0.025};
  double min_reciprocal_inlier_ratio{0.90};
  double max_geometry_condition_number{30.0};
  double max_gravity_residual_rad{0.05};
  double max_vertical_separation{0.20};
};

bool isStrongLoopEvidence(
  const LoopVerificationResult & verification,
  const StrongLoopEvidenceOptions & options);

enum class LoopConfirmationStatus
{
  kAwaiting,
  kConfirmed,
  kDuplicateCurrentKeyframe
};

// Tracks a correction hypothesis across distinct current keyframes. Multiple
// descriptor candidates from one scan are correlated and must not satisfy a
// multi-frame confirmation requirement by themselves.
class LoopConfirmationGate
{
public:
  LoopConfirmationStatus observe(
    const LoopVerificationResult & verification,
    std::size_t required_count,
    double translation_tolerance,
    double rotation_tolerance_rad);

  std::size_t count() const {return count_;}

private:
  std::optional<common::Pose> last_correction_;
  std::optional<common::KeyframeId> last_current_id_;
  std::size_t count_{0U};
};

struct LoopClosureUpdate
{
  bool keyframe_stored{false};
  std::vector<LoopCandidate> candidates;
  std::vector<LoopVerificationResult> verifications;
};

struct LoopWorkerStats
{
  std::size_t queue_depth{0U};
  std::size_t max_queue_depth{0U};
  std::uint64_t submitted{0U};
  std::uint64_t completed{0U};
  std::uint64_t dropped{0U};
  std::uint64_t timed_out{0U};
  std::uint64_t cancelled{0U};
  double last_duration_ms{0.0};
  double max_duration_ms{0.0};
};

}  // namespace itera_lio::loop

#endif  // ITERA_LIO__LOOP__LOOP_TYPES_HPP_
