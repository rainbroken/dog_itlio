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

#include "itera_lio/system/lio_pipeline.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <fstream>
#include <unordered_set>
#include <vector>
#include <Eigen/Geometry>

#include "itera_lio/common/transforms.hpp"
#include "itera_lio/common/voxel.hpp"
#include "itera_lio/common/voxel_visit_set.hpp"
#include "itera_lio/loop/submap_builder.hpp"

namespace itera_lio::system
{

namespace
{

sensor::LidarFrame voxelSampleScan(
  const sensor::LidarFrame & scan, const float voxel_size)
{
  sensor::LidarFrame result;
  result.scan_begin_time = scan.scan_begin_time;
  result.scan_end_time = scan.scan_end_time;
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0F) {
    return result;
  }
  thread_local common::VoxelVisitSet sampled;
  sampled.reset(scan.points.size());
  result.points.reserve(scan.points.size());
  for (std::size_t index = 0U; index < scan.points.size(); ++index) {
    const auto & point = scan.points[index];
    common::VoxelKey key;
    if (common::makeVoxelKey(point.position, voxel_size, key) && sampled.insert(key)) {
      result.points.push_back(point);
    }
  }
  return result;
}

}  // namespace

LioPipeline::LioPipeline(const LioPipelineOptions & options)
: options_(options),
  map_(
    options.map_voxel_size, options.map_rebuild_policy,
    options.map_async_rebuild_min_subtree_points),
  temporary_map_(
    options.map_voxel_size, options.map_rebuild_policy,
    options.map_async_rebuild_min_subtree_points),
  local_map_(options.local_map), temporary_local_map_(options.local_map),
  backend_(options.backend), loop_closure_(options.loop_closure),
  tracking_state_(options.tracking)
{
  backend_sink_ = &backend_;
  loop_sink_ = &loop_closure_;
  if (!std::isfinite(options_.scan_voxel_size) || options_.scan_voxel_size <= 0.0F) {
    options_.scan_voxel_size = 0.2F;
  }
}

LioPipeline::LioPipeline(
  const LioPipelineOptions & options,
  backend::BackendSink * backend_sink,
  loop::LoopClosureSink * loop_sink)
: LioPipeline(options)
{
  if (backend_sink != nullptr) {
    backend_sink_ = backend_sink;
  }
  if (loop_sink != nullptr) {
    loop_sink_ = loop_sink;
  }
}

PipelineResult LioPipeline::processScan(
  const sensor::LidarFrame & scan,
  estimator::NominalState & state,
  estimator::StateCovariance & covariance,
  const estimator::FixedExtrinsics & extrinsics,
  const sensor::LidarFrame * dense_scan)
{
  PipelineResult result;
  (void)dense_scan;
  const auto total_start = std::chrono::steady_clock::now();
  if (!scan.isValid() || !state.isValid() || !extrinsics.isValid()) {
    return result;
  }
  std::optional<sensor::LidarFrame> sampled_storage;
  if (!options_.input_prevoxelized) {
    sampled_storage.emplace(voxelSampleScan(scan, options_.scan_voxel_size));
  }
  const sensor::LidarFrame & sampled_scan = sampled_storage ? *sampled_storage : scan;
  result.registration_input_points = sampled_scan.points.size();
  std::vector<frontend::MapPoint> map_points;
  if (!map_initialized_) {
    map_points.reserve(sampled_scan.points.size());
    const common::Pose pose{state.R_WI, state.p_WI};
    for (const auto & point : sampled_scan.points) {
      const Eigen::Vector3d point_I = common::lidarToImu(point.position, extrinsics);
      const Eigen::Vector3d point_W = common::imuToWorld(point_I, pose);
      map_points.push_back(frontend::MapPoint{point_W.cast<float>(), point.intensity});
    }
    if (map_points.size() < 3U) {return result;}
    const auto map_insert_start = std::chrono::steady_clock::now();
    map_.build(map_points);
    result.map_updates = map_points;
    result.map_inserted_points = map_.size();
    result.map_insert_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - map_insert_start).count();
    map_initialized_ = map_.size() > 0U;
    result.initialized_map = map_initialized_;
    result.map_insert_reason = "bootstrap";
    updateBackendAndLoop(sampled_scan, state, extrinsics, result);
    result.map_size = map_.size();
    result.ikd_rebuild = map_.rebuildStats();
    result.stored_loop_keyframe_clouds = loop_sink_->storedKeyframeClouds();
    result.map_update_time_ms = result.map_insert_time_ms + result.archive_update_time_ms;
    result.total_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - total_start).count();
    return result;
  }
  double registration_time_ms = 0.0;
  common::RegistrationReport latest_registration;
  frontend::ScanToMapIterationCache registration_cache;
  std::size_t total_knn_queries = 0U;
  std::size_t total_reused_planes = 0U;
  std::size_t total_candidate_attempts = 0U;
  std::size_t total_accepted_candidates = 0U;
  double total_knn_time_ms = 0.0;
  double total_plane_fit_time_ms = 0.0;
  double total_selection_time_ms = 0.0;
  const estimator::ObservationBuilder observation_builder =
    [&](const estimator::NominalState & linearization_state) {
      const auto registration_start = std::chrono::steady_clock::now();
      const common::Pose pose{linearization_state.R_WI, linearization_state.p_WI};
      const frontend::NeighborMap & registration_map =
        tracking_state_.state() == estimator::TrackingState::kNormal ?
        static_cast<const frontend::NeighborMap &>(map_) :
        static_cast<const frontend::NeighborMap &>(temporary_map_);
      latest_registration = frontend::buildScanToMapObservations(
        sampled_scan, pose, extrinsics, registration_map, options_.registration,
        &registration_cache);
      total_knn_queries += latest_registration.knn_queries;
      total_reused_planes += latest_registration.reused_planes;
      total_candidate_attempts += latest_registration.candidate_attempts;
      total_accepted_candidates += latest_registration.accepted_candidates;
      total_knn_time_ms += latest_registration.knn_time_ms;
      total_plane_fit_time_ms += latest_registration.plane_fit_time_ms;
      total_selection_time_ms += latest_registration.selection_time_ms;
      registration_time_ms += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - registration_start).count();
      return latest_registration;
    };
  const auto correction_start = std::chrono::steady_clock::now();
  const Eigen::Vector3d predicted_position = state.p_WI;
  const Eigen::Matrix3d predicted_rotation = state.R_WI;
  {
    const frontend::NeighborMap & query_map =
      tracking_state_.state() == estimator::TrackingState::kNormal ?
      static_cast<const frontend::NeighborMap &>(map_) :
      static_cast<const frontend::NeighborMap &>(temporary_map_);
    // End the read epoch before any map insertion, deletion, or transition.
    auto read_guard = options_.registration.num_threads > 1 ?
      query_map.prepareConcurrentRead() : nullptr;
    result.correction = estimator::correctWithLidar(
      observation_builder, options_.correction, state, covariance, extrinsics);
  }
  result.correction_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - correction_start).count();
  result.frame_translation_correction = (state.p_WI - predicted_position).norm();
  result.frame_rotation_correction = Eigen::AngleAxisd(
    predicted_rotation.transpose() * state.R_WI).angle();
  result.registration = std::move(latest_registration);
  result.registration.knn_queries = total_knn_queries;
  result.registration.reused_planes = total_reused_planes;
  result.registration.total_candidate_attempts = total_candidate_attempts;
  result.registration.total_accepted_candidates = total_accepted_candidates;
  result.registration.knn_time_ms = total_knn_time_ms;
  result.registration.plane_fit_time_ms = total_plane_fit_time_ms;
  result.registration.selection_time_ms = total_selection_time_ms;
  result.registration_time_ms = registration_time_ms;
  result.degeneracy = estimator::detectDegeneracy(
    result.correction.observation_update, options_.degeneracy);
  result.corrected = result.correction.iterations > 0;
  // Include failed/degenerate scans in the segment quality history, even if
  // they cannot create a keyframe or insert trusted map points.
  backend_sink_->observeOdometryQuality(
    common::Pose{state.R_WI, state.p_WI}, result.correction.observation_update.pose_information);
  result.tracking = tracking_state_.update(
    result.corrected, result.correction.effective_points,
    result.correction.mean_abs_residual_after, result.correction.observation_update);
  if (result.tracking.transitioned &&
    result.tracking.previous_state == estimator::TrackingState::kNormal)
  {
    std::vector<frontend::MapPoint> trusted_snapshot;
    if (map_.getAllPoints(trusted_snapshot) && !trusted_snapshot.empty()) {
      temporary_map_.build(trusted_snapshot);
    }
  }
  if (result.corrected && result.correction.effective_points > 0U) {
    const auto & quality = options_.registration;
    // Compare against the observation budget, not the full dense scan. The
    // ESKF intentionally uses at most max_effective_points observations.
    const double effective_ratio = static_cast<double>(result.correction.effective_points) /
      std::max<std::size_t>(1U, std::min(sampled_scan.points.size(), quality.max_effective_points));
    result.map_insert_reason = "accepted";
    if (quality.require_converged_for_map_insert && !result.correction.converged) {
      result.map_insert_reason = "not_converged";
    } else if (!std::isfinite(result.correction.mean_abs_residual_after) ||
      result.correction.mean_abs_residual_after > quality.map_insert_max_mean_residual) {
      result.map_insert_reason = "mean_residual";
    } else if (!std::isfinite(result.frame_rotation_correction) ||
      result.frame_rotation_correction > quality.map_insert_max_rotation_correction) {
      result.map_insert_reason = "rotation_correction";
    } else if (!std::isfinite(result.frame_translation_correction) ||
      result.frame_translation_correction > quality.map_insert_max_translation_correction) {
      result.map_insert_reason = "translation_correction";
    } else if (effective_ratio < quality.map_insert_min_effective_ratio) {
      result.map_insert_reason = "effective_ratio";
    } else if (result.tracking.state != estimator::TrackingState::kNormal) {
      result.map_insert_reason = "tracking_not_normal";
    }
    if (!quality.enable_map_insert_quality_gate) {
      result.map_insert_reason = result.tracking.state == estimator::TrackingState::kNormal ?
        "accepted" : "tracking_not_normal";
    }
    result.map_candidate_points = result.registration.insertion_candidates.size();
    map_points.clear();
    map_points.reserve(result.registration.insertion_candidates.size());
    const common::Pose corrected_pose{state.R_WI, state.p_WI};
    for (const auto & candidate : result.registration.insertion_candidates) {
      const Eigen::Vector3d point_W = common::imuToWorld(candidate.point_I, corrected_pose);
      const double corrected_residual = candidate.normal_W.dot(point_W - candidate.centroid_W);
      if (!std::isfinite(corrected_residual) ||
        std::abs(corrected_residual) > options_.registration.map_insert_residual_threshold) {
        ++result.map_rejected_points;
        continue;
      }
      map_points.push_back(frontend::MapPoint{point_W.cast<float>(), candidate.intensity});
    }
    if (result.map_insert_reason == "accepted" && !map_points.empty()) {
      const auto local_map_start = std::chrono::steady_clock::now();
      local_map_.update(state.p_WI.cast<float>(), map_, nullptr);
      result.local_map_update_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - local_map_start).count();
      result.map_delete_time_ms = local_map_.lastDeleteTimeMs();
      result.deleted_map_points = local_map_.lastDeletedPointCount();
      const auto map_update_start = std::chrono::steady_clock::now();
      auto insert_report = map_.addPoints(map_points, true);
      result.map_inserted_points = insert_report.inserted_count;
      result.map_replaced_points = insert_report.replaced_count;
      result.map_rejected_points += insert_report.rejected_count;
      result.map_compacted_points = insert_report.compacted_count;
      result.map_insert_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - map_update_start).count();
      result.map_updated = true;
      result.map_updates = std::move(insert_report.upserted_points);
    } else if (result.map_insert_reason == "accepted") {
      result.map_insert_reason = "no_consistent_points";
    } else if (result.tracking.state == estimator::TrackingState::kDegraded) {
      const auto local_map_start = std::chrono::steady_clock::now();
      temporary_local_map_.update(state.p_WI.cast<float>(), temporary_map_, nullptr);
      result.local_map_update_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - local_map_start).count();
      result.map_delete_time_ms = temporary_local_map_.lastDeleteTimeMs();
      result.deleted_map_points = temporary_local_map_.lastDeletedPointCount();
      const auto map_update_start = std::chrono::steady_clock::now();
      const auto insert_report = temporary_map_.addPoints(map_points, true);
      result.temporary_map_inserted_points = insert_report.inserted_count +
        insert_report.replaced_count;
      result.map_rejected_points += insert_report.rejected_count;
      result.map_compacted_points = insert_report.compacted_count;
      result.map_insert_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - map_update_start).count();
      result.temporary_map_updated = true;
    }
    updateBackendAndLoop(sampled_scan, state, extrinsics, result);
    result.map_update_time_ms = result.local_map_update_time_ms +
      result.archive_update_time_ms + result.map_insert_time_ms;
  }
  result.pending_delete_boxes = local_map_.pendingDeleteBoxes();
  result.map_size = map_.size();
  result.temporary_map_size = temporary_map_.size();
  result.ikd_rebuild = map_.rebuildStats();
  result.stored_loop_keyframe_clouds = loop_sink_->storedKeyframeClouds();
  result.total_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - total_start).count();
  return result;
}

void LioPipeline::updateBackendAndLoop(
  const sensor::LidarFrame & scan,
  const estimator::NominalState & state,
  const estimator::FixedExtrinsics & extrinsics,
  PipelineResult & result)
{
  auto completed_verifications = loop_sink_->pollCompleted();
  auto completed_backend_updates = backend_sink_->pollCompleted();
  const auto backend_start = std::chrono::steady_clock::now();
  result.backend = backend_sink_->addOdometryKeyframe(
    scan.scan_end_time, common::Pose{state.R_WI, state.p_WI});
  if (!completed_backend_updates.empty()) {
    const auto current_backend = result.backend;
    result.backend = std::move(completed_backend_updates.back());
    result.backend.keyframe_added = current_backend.keyframe_added;
    result.backend.keyframe = current_backend.keyframe;
    result.backend.keyframe_count = current_backend.keyframe_count;
    result.backend.odometry_constraint_count = current_backend.odometry_constraint_count;
    result.backend.loop_constraint_count = current_backend.loop_constraint_count;
  }
  result.backend_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - backend_start).count();
  const auto loop_start = std::chrono::steady_clock::now();
  loop::LoopClosureUpdate loop_update;
  if (
    result.backend.keyframe_added && result.backend.keyframe && options_.loop_closure.enabled)
  {
    loop::KeyframeCloud cloud;
    cloud.keyframe = *result.backend.keyframe;
    cloud.points_I.reserve(scan.points.size());
    for (const auto & point : scan.points) {
      cloud.points_I.push_back(
        frontend::MapPoint{
          (extrinsics.R_IL * point.position + extrinsics.p_IL).cast<float>(),
          point.intensity});
    }
    loop_update = loop_sink_->addKeyframe(std::move(cloud));
  }
  result.loop_candidates = loop_update.candidates.size();
  completed_verifications.insert(
    completed_verifications.end(), loop_update.verifications.begin(),
    loop_update.verifications.end());
  result.loop_verification_results = completed_verifications;
  result.loop_verifications = completed_verifications.size();
  for (const auto & verification : completed_verifications) {
    result.loop_current_id = verification.current_id;
    result.loop_matched_id = verification.matched_id;
    result.loop_inliers = verification.inliers;
    result.loop_fitness = verification.fitness;
    result.loop_translation_correction = verification.translation_correction;
    result.loop_rotation_correction = verification.rotation_correction;
    result.loop_icp_initialization = verification.initialization;
    result.loop_rejection_reason = verification.rejection_reason;
    if (!verification.isValid()) {
      ++result.loop_rejected_verifications;
      continue;
    }
    ++result.loop_accepted_verifications;
    const auto confirmation = loop_confirmation_gate_.observe(
      verification, options_.loop_closure.confirmation_count,
      options_.loop_closure.confirmation_translation_tolerance,
      options_.loop_closure.confirmation_rotation_tolerance_rad);
    if (confirmation == loop::LoopConfirmationStatus::kDuplicateCurrentKeyframe) {
      result.loop_rejection_reason = "duplicate_current_keyframe_confirmation";
      continue;
    }
    const bool strong_single_frame = loop::isStrongLoopEvidence(
      verification, options_.loop_closure.strong_evidence);
    if (confirmation == loop::LoopConfirmationStatus::kAwaiting &&
      !strong_single_frame)
    {
      result.loop_rejection_reason = "awaiting_consistent_confirmation";
      continue;
    }
    const bool submitted = backend_sink_->enqueueLoopConstraint(
      verification.current_id, verification.matched_id,
      verification.T_Ia_Ib, verification.information);
    if (!submitted) {result.loop_rejection_reason = "backend_rejected";}
  }
  result.loop_closure_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - loop_start).count();
  result.loop_worker = loop_sink_->workerStats();
  result.backend_worker = backend_sink_->workerStats();
}

OptimizedLoopRefinementReport LioPipeline::refineOptimizedLoops(const std::string & cloud_archive)
{
  OptimizedLoopRefinementReport report;
  if (!options_.loop_closure.enabled || !backend_.graph().hasLoopConstraint()) {
    return report;
  }
  loop_closure_.cancelPending();
  backend_.cancelPending();
  if (!cloud_archive.empty()) {
    // Versioned, ordered IMU-local points let the exact local geometry be
    // rechecked without another bag playback. Poses are saved by the caller.
    std::ofstream stream(cloud_archive, std::ios::binary);
    stream.write("ILIOREV1", 8);
    const std::uint64_t count = loop_closure_.store().size();
    stream.write(reinterpret_cast<const char *>(&count), sizeof(count));
    for (std::size_t id = 0; id < count && stream; ++id) {
      loop::KeyframeCloud cloud;
      if (!loop_closure_.store().get(id, cloud)) {stream.setstate(std::ios::failbit); break;}
      const std::uint64_t points = cloud.points_I.size();
      stream.write(reinterpret_cast<const char *>(&points), sizeof(points));
      for (const auto & point : cloud.points_I) {
        const float values[]{point.position.x(), point.position.y(), point.position.z(), point.intensity};
        stream.write(reinterpret_cast<const char *>(values), sizeof(values));
      }
    }
    stream.flush();
    if (!stream) {report.error = "Cannot export local keyframe archive; refinement not applied"; return report;}
  }
  const auto verifications = loop_closure_.reverifyOptimized(backend_.optimizedKeyframes());
  report.verifications = verifications.size();
  report.results = verifications;
  loop::LoopConfirmationGate confirmation_gate;
  std::optional<common::Pose> last_committed_pose;
  std::unordered_set<common::KeyframeId> checked_current;
  for (const auto & verification : verifications) {
    if (!verification.isValid()) {continue;}
    ++report.accepted;
    // Candidates from one current cloud are not independent confirmations.
    if (!checked_current.insert(verification.current_id).second) {continue;}
    const auto confirmation = confirmation_gate.observe(
      verification, std::max(std::size_t{2U}, options_.loop_closure.confirmation_count),
      0.25, 0.025);
    if (confirmation != loop::LoopConfirmationStatus::kConfirmed) {continue;}
    const auto pose = backend_.optimizedKeyframes()[verification.current_id].pose_WI;
    if (last_committed_pose &&
      (pose.p_WI - last_committed_pose->p_WI).norm() < 1.5) {continue;}
    const auto update = backend_.addLoopConstraint(
      verification.current_id, verification.matched_id,
      verification.T_Ia_Ib, verification.information);
    if (update && update->loop_constraint_added && update->optimization.optimized) {
      ++report.committed;
      last_committed_pose = pose;
    }
    if (report.committed >= 48U) {break;}
  }
  return report;
}

bool LioPipeline::getOptimizedMapPoints(
  std::vector<common::MapPoint> & points, const float voxel_size) const
{
  points.clear();
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0F ||
    !backend_.graph().hasLoopConstraint())
  {
    return false;
  }
  const auto optimized_keyframes = backend_.optimizedKeyframes();
  points = loop::rebuildOptimizedMap(
    loop_closure_.store(), optimized_keyframes, voxel_size);
  return !points.empty();
}

}  // namespace itera_lio::system
