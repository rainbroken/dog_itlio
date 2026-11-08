// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0 (the "License");

#include "evaluation_format.hpp"
#include "itera_lio/common/evaluation_types.hpp"
#include "itera_lio/common/frame_metrics.hpp"
#include "itera_lio/estimator/state.hpp"
#include "itera_lio/sensor/lidar_frame.hpp"
#include "itera_lio/system/lio_pipeline.hpp"

#include <Eigen/Geometry>
#include <iomanip>

namespace itera_lio::output::detail
{

std::string csvEscape(const std::string & value)
{
  if (value.find_first_of(",\"\r\n") == std::string::npos) {
    return value;
  }
  std::string escaped{"\""};
  for (const char character : value) {
    if (character == '"') {
      escaped += "\"\"";
    } else {
      escaped += character;
    }
  }
  escaped += '"';
  return escaped;
}

void writeRuntimeRow(
  std::ostream & stream, const common::RuntimeSnapshot & snapshot,
  const double lidar_rate, const double imu_rate, const bool detailed)
{
  if (!detailed) {
    stream << std::setprecision(17)
      << snapshot.wall_timestamp << ','
      << snapshot.sensor_timestamp << ','
      << lidar_rate << ','
      << imu_rate << ','
      << snapshot.lidar_queue_size << ','
      << snapshot.imu_queue_size << ','
      << snapshot.lidar_received << ','
      << snapshot.imu_received << ','
      << snapshot.lidar_processed << ','
      << snapshot.lidar_parse_failures << ','
      << snapshot.lidar_deskew_failures << ','
      << snapshot.lidar_timestamp_regressions << ','
      << snapshot.imu_timestamp_regressions << ','
      << snapshot.lidar_capacity_drops << ','
      << snapshot.imu_capacity_drops << ','
      << snapshot.pending_lidar_drops << ','
      << snapshot.imu_gap_resets << ','
      << snapshot.local_map_points << ','
      << snapshot.backend_keyframes << ','
      << snapshot.backend_loop_constraints << ','
      << snapshot.loop_worker_queue_depth << ','
      << snapshot.loop_worker_dropped << ','
      << snapshot.loop_worker_timed_out << ','
      << snapshot.loop_worker_cancelled << ','
      << snapshot.backend_worker_queue_depth << ','
      << snapshot.backend_worker_dropped << ','
      << snapshot.backend_worker_timed_out << ','
      << snapshot.backend_worker_cancelled << ','
      << snapshot.rebuild_total << ','
      << snapshot.rebuild_max_ms << ','
      << snapshot.resident_memory_mib << ','
      << snapshot.peak_resident_memory_mib << '\n';
    return;
  }
  stream << std::setprecision(17) << snapshot.wall_timestamp << ','
         << snapshot.sensor_timestamp << ',' << lidar_rate << ',' << imu_rate << ','
         << snapshot.lidar_queue_size << ',' << snapshot.imu_queue_size << ','
         << snapshot.lidar_received << ',' << snapshot.imu_received << ','
         << snapshot.lidar_processed << ',' << snapshot.imu_processed << ','
         << snapshot.lidar_deskewed << ',' << snapshot.lidar_parse_failures << ','
         << snapshot.lidar_deskew_failures << ','
         << snapshot.lidar_timestamp_regressions << ','
         << snapshot.imu_timestamp_regressions << ','
         << snapshot.lidar_capacity_drops << ',' << snapshot.imu_capacity_drops << ','
         << snapshot.pending_lidar_drops << ',' << snapshot.imu_gap_resets << ','
         << snapshot.range_filter_ms << ',' << snapshot.voxel_ms << ','
         << snapshot.deskew_ms << ',' << snapshot.registration_ms << ','
         << snapshot.correction_ms << ',' << snapshot.map_delete_ms << ','
         << snapshot.archive_update_ms << ',' << snapshot.map_insert_ms << ','
         << snapshot.trajectory_reanchor_ms << ',' << snapshot.repropagation_ms << ','
         << snapshot.core_ms << ',' << snapshot.output_ms << ','
         << snapshot.end_to_end_ms << ',' << snapshot.local_map_points << ','
         << snapshot.output_global_map_points << ',' << snapshot.map_candidates << ','
         << snapshot.map_inserted << ',' << snapshot.map_replaced << ','
         << snapshot.map_rejected << ',' << snapshot.pending_delete_boxes << ','
         << snapshot.rebuild_total << ',' << snapshot.rebuild_synchronous << ','
         << snapshot.rebuild_asynchronous << ','
         << static_cast<unsigned int>(snapshot.rebuild_last_reason) << ','
         << snapshot.rebuild_last_points << ',' << snapshot.rebuild_max_points << ','
         << snapshot.rebuild_last_ms << ',' << snapshot.rebuild_max_ms << ','
         << (snapshot.rebuild_active ? 1 : 0) << ',' << snapshot.backend_keyframes << ','
         << snapshot.backend_odometry_constraints << ','
         << snapshot.backend_loop_constraints << ','
         << snapshot.loop_worker_queue_depth << ','
         << snapshot.loop_worker_max_queue_depth << ','
         << snapshot.loop_worker_submitted << ',' << snapshot.loop_worker_completed << ','
         << snapshot.loop_worker_dropped << ',' << snapshot.loop_worker_timed_out << ','
         << snapshot.loop_worker_cancelled << ','
         << snapshot.backend_worker_queue_depth << ','
         << snapshot.backend_worker_max_queue_depth << ','
         << snapshot.backend_worker_submitted << ',' << snapshot.backend_worker_completed << ','
         << snapshot.backend_worker_dropped << ',' << snapshot.backend_worker_timed_out << ','
         << snapshot.backend_worker_cancelled << ',' << snapshot.resident_memory_mib << ','
         << snapshot.peak_resident_memory_mib << '\n';
}

void writeFrameHeader(std::ostream & stream, const bool detailed)
{
  if (detailed) {
    stream
      << "frame_index,timestamp,scan_points,sampled_points,effective_points,"
      << "mean_residual_before,mean_residual_after,iterations,converged,rotation_correction,"
      << "translation_correction,mahalanobis_rejected,weak_directions,"
      << "condition_number,whitened_condition_number,"
      << "whitened_eig0,whitened_eig1,whitened_eig2,whitened_eig3,"
      << "whitened_eig4,whitened_eig5,"
      << "observability_rx,observability_ry,observability_rz,"
      << "observability_tx,observability_ty,observability_tz,"
      << "tracking_state,tracking_transition,degenerate,map_updated,map_insert_reason,"
      << "frame_translation_correction,frame_rotation_correction,"
      << "temporary_map_updated,temporary_map_inserted,temporary_map_size,"
      << "map_candidates,map_inserted,"
      << "map_replaced,map_rejected,map_compacted,archive_inserted,"
      << "dense_archive_inserted,deleted_map_points,pending_delete_boxes,"
      << "map_size,global_map_size,dense_archive_size,"
      << "range_filter_ms,voxel_ms,imu_to_scan_end_ms,deskew_ms,knn_ms,"
      << "plane_fit_ms,selection_ms,registration_ms,correction_ms,"
      << "local_map_update_ms,map_delete_ms,archive_update_ms,map_insert_ms,"
      << "map_update_ms,total_ms,backend_ms,loop_closure_ms,trajectory_reanchor_ms,"
      << "repropagation_ms,repropagated_imu_samples,post_scan_imu_duration_ms,"
      << "core_end_to_end_ms,output_preparation_ms,serialization_ms,publish_call_ms,"
      << "odometry_log_ms,output_ms,end_to_end_ms,loop_candidates,"
      << "loop_verifications,loop_accepted_verifications,loop_rejected_verifications,"
      << "loop_current_id,loop_matched_id,loop_inliers,loop_fitness,"
      << "loop_icp_initialization,loop_rejection_reason,"
      << "loop_translation_correction,loop_rotation_correction,"
      << "loop_constraint_added,backend_optimized,"
      << "loop_queue_depth,loop_queue_max,loop_submitted,loop_completed,"
      << "loop_dropped,loop_timed_out,loop_cancelled,"
      << "backend_queue_depth,backend_queue_max,backend_submitted,backend_completed,"
      << "backend_dropped,backend_timed_out,backend_cancelled,"
      << "candidate_attempts_total,accepted_candidates_total,candidate_attempts_last,"
      << "accepted_candidates_last,knn_queries_total,reused_planes_total,plane_cache_hit_ratio,"
      << "full_archive_conversion_ms,full_archive_wait_ms,full_archive_write_ms,"
      << "tracking_bad_mask,tracking_observation_good,tracking_observation_lost,"
      << "tracking_consecutive_good,tracking_consecutive_bad,tracking_consecutive_lost,"
      << "input_callback_ms,pre_processing_wait_ms,callback_to_output_ms,callback_to_state_publish_ms,state_publish_to_output_end_ms\n";
  } else {
    stream
      << "frame_index,timestamp,scan_points,sampled_points,effective_points,"
      << "mean_residual_before,mean_residual_after,iterations,converged,tracking_state,"
      << "degenerate,condition_number,map_updated,map_insert_reason,map_size,"
      << "translation_correction,rotation_correction,deskew_ms,registration_ms,correction_ms,"
      << "map_update_ms,total_ms,core_end_to_end_ms,output_ms,end_to_end_ms,"
      << "callback_to_state_publish_ms,loop_candidates,loop_verifications,loop_constraint_added,backend_optimized\n";
  }
}

void writeOdometryHeader(std::ostream & stream)
{
  stream << "timestamp,px,py,pz,qx,qy,qz,qw,vx,vy,vz,gx,gy,gz,bgx,bgy,bgz,bax,bay,baz";
  for (int i = 0; i < estimator::ErrorStateLayout::kDimension; ++i) {
    stream << ",cov_diag_" << i;
  }
  stream << '\n';
}

void writeRuntimeHeader(std::ostream & stream, const bool detailed)
{
  if (detailed) {
    stream
      << "wall_timestamp,sensor_timestamp,lidar_rate_hz,imu_rate_hz,"
      << "lidar_queue,imu_queue,lidar_received,imu_received,lidar_processed,"
      << "imu_processed,lidar_deskewed,lidar_parse_failures,"
      << "lidar_deskew_failures,lidar_timestamp_regressions,"
      << "imu_timestamp_regressions,lidar_capacity_drops,imu_capacity_drops,"
      << "pending_lidar_drops,imu_gap_resets,range_filter_ms,voxel_ms,deskew_ms,"
      << "registration_ms,correction_ms,map_delete_ms,archive_update_ms,"
      << "map_insert_ms,trajectory_reanchor_ms,repropagation_ms,core_ms,"
      << "output_ms,end_to_end_ms,local_map_points,output_global_map_points,"
      << "map_candidates,map_inserted,map_replaced,map_rejected,"
      << "pending_delete_boxes,rebuild_total,rebuild_synchronous,"
      << "rebuild_asynchronous,rebuild_last_reason,rebuild_last_points,"
      << "rebuild_max_points,rebuild_last_ms,rebuild_max_ms,rebuild_active,"
      << "backend_keyframes,backend_odometry_constraints,"
      << "backend_loop_constraints,loop_queue_depth,loop_queue_max,"
      << "loop_submitted,loop_completed,loop_dropped,loop_timed_out,loop_cancelled,"
      << "backend_queue_depth,backend_queue_max,backend_submitted,backend_completed,"
      << "backend_dropped,backend_timed_out,backend_cancelled,"
      << "resident_memory_mib,peak_resident_memory_mib\n";
  } else {
    stream
      << "wall_timestamp,sensor_timestamp,lidar_rate_hz,imu_rate_hz,lidar_queue,"
      << "imu_queue,lidar_received,imu_received,lidar_processed,lidar_parse_failures,"
      << "lidar_deskew_failures,lidar_timestamp_regressions,imu_timestamp_regressions,lidar_capacity_drops,imu_capacity_drops,"
      << "pending_lidar_drops,imu_gap_resets,local_map_points,backend_keyframes,backend_loop_constraints,"
      << "loop_queue_depth,loop_dropped,loop_timed_out,loop_cancelled,backend_queue_depth,"
      << "backend_dropped,backend_timed_out,backend_cancelled,rebuild_total,rebuild_max_ms,"
      << "resident_memory_mib,peak_resident_memory_mib\n";
  }
}

void writeLoopVerificationHeader(std::ostream & stream)
{
  stream
    << "wall_timestamp,sensor_timestamp,frame_index,current_id,matched_id,"
    << "descriptor_distance,odometry_distance,odometry_vertical_distance,"
    << "gravity_angle_rad,initialization,accepted,"
    << "rejection_reason,inliers,fitness,reciprocal_inlier_ratio,"
    << "geometry_condition_number,translation_correction,rotation_correction,"
    << "gravity_residual_rad,vertical_separation,"
    << "correction_px,correction_py,correction_pz,"
    << "correction_qx,correction_qy,correction_qz,correction_qw,"
    << "audit_points,audit_inliers,audit_overlap,audit_nearest_rmse,audit_plane_points,audit_plane_p90,"
    << "audit_wall_points,audit_wall_p90,audit_horizontal_points,audit_horizontal_p90\n";
}

void writeFrameRow(
  std::ostream & stream, const std::uint64_t current_frame_index,
  const std::size_t raw_scan_points, const sensor::LidarFrame & scan,
  const system::PipelineResult & result,
  const common::FrameProcessingTiming & processing_timing,
  const common::FrameOutputTiming & output_timing, const bool detailed)
{
  const auto & observation_update = result.correction.observation_update;
  if (detailed) {
    stream << std::setprecision(17) << current_frame_index << ','
            << scan.scan_end_time << ',' << raw_scan_points << ','
            << result.registration_input_points << ','
            << observation_update.accepted_observations.size() << ','
            << result.correction.mean_abs_residual_before << ','
            << result.correction.mean_abs_residual_after << ','
            << result.correction.iterations << ','
            << (result.correction.converged ? 1 : 0) << ','
            << result.correction.rotation_correction << ','
            << result.correction.translation_correction << ','
            << observation_update.mahalanobis_rejected << ','
            << result.correction.weak_directions << ','
            << result.degeneracy.condition_number << ','
            << observation_update.whitened_condition_number << ',';
    for (Eigen::Index index = 0; index < 6; ++index) {
      stream << observation_update.whitened_eigenvalues[index] << ',';
    }
    for (Eigen::Index index = 0; index < 6; ++index) {
      stream << observation_update.whitened_dof_scores[index] << ',';
    }
    stream << estimator::trackingStateName(result.tracking.state) << ','
            << (result.tracking.transitioned ? 1 : 0) << ','
            << (result.degeneracy.degenerate ? 1 : 0) << ','
            << (result.map_updated ? 1 : 0) << ','
            << result.map_insert_reason << ','
            << result.frame_translation_correction << ','
            << result.frame_rotation_correction << ','
            << (result.temporary_map_updated ? 1 : 0) << ','
            << result.temporary_map_inserted_points << ','
            << result.temporary_map_size << ','
            << result.map_candidate_points << ',' << result.map_inserted_points << ','
            << result.map_replaced_points << ',' << result.map_rejected_points << ','
            << result.map_compacted_points << ',' << result.archive_inserted_points << ','
            << result.dense_archive_inserted_points << ',' << result.deleted_map_points << ','
            << result.pending_delete_boxes << ',' << result.map_size << ','
            << result.global_map_size << ',' << result.dense_archive_size << ','
            << processing_timing.range_filter_time_ms << ','
            << processing_timing.voxel_time_ms << ','
            << processing_timing.imu_to_scan_end_time_ms << ','
            << processing_timing.deskew_time_ms << ','
            << result.registration.knn_time_ms << ','
            << result.registration.plane_fit_time_ms << ','
            << result.registration.selection_time_ms << ','
            << result.registration_time_ms << ',' << result.correction_time_ms << ','
            << result.local_map_update_time_ms << ',' << result.map_delete_time_ms << ','
            << result.archive_update_time_ms << ',' << result.map_insert_time_ms << ','
            << result.map_update_time_ms << ',' << result.total_time_ms << ','
            << result.backend_time_ms << ','
            << result.loop_closure_time_ms << ','
            << processing_timing.trajectory_reanchor_time_ms << ','
            << processing_timing.repropagation_time_ms << ','
            << processing_timing.repropagated_imu_samples << ','
            << processing_timing.post_scan_imu_duration_ms << ','
            << processing_timing.core_end_to_end_time_ms << ','
            << output_timing.preparation_time_ms << ','
            << output_timing.serialization_time_ms << ','
            << output_timing.publish_call_time_ms << ','
            << output_timing.odometry_log_time_ms << ','
            << output_timing.output_total_time_ms << ','
            << output_timing.end_to_end_time_ms << ',' << result.loop_candidates << ','
            << result.loop_verifications << ',' << result.loop_accepted_verifications << ','
            << result.loop_rejected_verifications << ',' << result.loop_current_id << ','
            << result.loop_matched_id << ',' << result.loop_inliers << ','
            << result.loop_fitness << ',' << csvEscape(result.loop_icp_initialization) << ','
            << csvEscape(result.loop_rejection_reason) << ','
            << result.loop_translation_correction << ','
            << result.loop_rotation_correction << ','
            << (result.backend.loop_constraint_added ? 1 : 0) << ','
            << (result.backend.optimization.optimized ? 1 : 0) << ','
            << result.loop_worker.queue_depth << ',' << result.loop_worker.max_queue_depth << ','
            << result.loop_worker.submitted << ',' << result.loop_worker.completed << ','
            << result.loop_worker.dropped << ',' << result.loop_worker.timed_out << ','
            << result.loop_worker.cancelled << ',' << result.backend_worker.queue_depth << ','
            << result.backend_worker.max_queue_depth << ','
            << result.backend_worker.submitted << ',' << result.backend_worker.completed << ','
            << result.backend_worker.dropped << ',' << result.backend_worker.timed_out << ','
            << result.backend_worker.cancelled << ','
            << result.registration.total_candidate_attempts << ','
            << result.registration.total_accepted_candidates << ','
            << result.registration.candidate_attempts << ','
            << result.registration.accepted_candidates << ','
            << result.registration.knn_queries << ',' << result.registration.reused_planes << ','
            << (result.registration.total_candidate_attempts == 0U ? 0.0 :
              static_cast<double>(result.registration.reused_planes) /
              result.registration.total_candidate_attempts) << ','
            << output_timing.full_archive_conversion_time_ms << ','
            << output_timing.full_archive_wait_time_ms << ','
            << output_timing.full_archive_write_time_ms << ','
            << result.tracking.bad_observation_mask << ','
            << result.tracking.observation_good << ',' << result.tracking.observation_lost << ','
            << result.tracking.consecutive_good << ',' << result.tracking.consecutive_bad << ','
            << result.tracking.consecutive_lost << ','
            << processing_timing.input_callback_ms << ','
            << processing_timing.pre_processing_wait_ms << ','
            << output_timing.callback_to_output_ms << ','
            << output_timing.callback_to_state_publish_ms << ','
            << (output_timing.callback_to_state_publish_ms > 0.0 ?
              output_timing.callback_to_output_ms - output_timing.callback_to_state_publish_ms : 0.0) << '\n';
  } else {
    stream << std::setprecision(17)
      << current_frame_index << ','
      << scan.scan_end_time << ','
      << raw_scan_points << ','
      << result.registration_input_points << ','
      << observation_update.accepted_observations.size() << ','
      << result.correction.mean_abs_residual_before << ','
      << result.correction.mean_abs_residual_after << ','
      << result.correction.iterations << ','
      << (result.correction.converged ? 1 : 0) << ','
      << estimator::trackingStateName(result.tracking.state) << ','
      << (result.degeneracy.degenerate ? 1 : 0) << ','
      << result.degeneracy.condition_number << ','
      << (result.map_updated ? 1 : 0) << ','
      << csvEscape(result.map_insert_reason) << ','
      << result.map_size << ','
      << result.correction.translation_correction << ','
      << result.correction.rotation_correction << ','
      << processing_timing.deskew_time_ms << ','
      << result.registration_time_ms << ','
      << result.correction_time_ms << ','
      << result.map_update_time_ms << ','
      << result.total_time_ms << ','
      << processing_timing.core_end_to_end_time_ms << ','
      << output_timing.output_total_time_ms << ','
      << output_timing.end_to_end_time_ms << ','
      << output_timing.callback_to_state_publish_ms << ','
      << result.loop_candidates << ','
      << result.loop_verifications << ','
      << (result.backend.loop_constraint_added ? 1 : 0) << ','
      << (result.backend.optimization.optimized ? 1 : 0) << '\n';

  }
}

void writeLoopVerificationRow(
  std::ostream & stream, const double wall_timestamp, const double sensor_timestamp,
  const std::uint64_t frame_index, const loop::LoopVerificationResult & verification)
{
  Eigen::Quaterniond correction_orientation(verification.global_correction.R_WI);
  correction_orientation.normalize();
  stream << std::setprecision(17) << wall_timestamp << ','
                           << sensor_timestamp << ',' << frame_index << ','
                           << verification.current_id << ',' << verification.matched_id << ','
                           << verification.descriptor_distance << ','
                           << verification.odometry_distance << ','
                           << verification.odometry_vertical_distance << ','
                           << verification.gravity_angle_rad << ','
                           << csvEscape(verification.initialization) << ','
                           << (verification.isValid() ? 1 : 0) << ','
                           << csvEscape(verification.rejection_reason) << ','
                           << verification.inliers << ',' << verification.fitness << ','
                           << verification.reciprocal_inlier_ratio << ','
                           << verification.geometry_condition_number << ','
                           << verification.translation_correction << ','
                           << verification.rotation_correction << ','
                           << verification.gravity_residual_rad << ','
                           << verification.vertical_separation << ','
                           << verification.global_correction.p_WI.x() << ','
                           << verification.global_correction.p_WI.y() << ','
                           << verification.global_correction.p_WI.z() << ','
                           << correction_orientation.x() << ','
                           << correction_orientation.y() << ','
                           << correction_orientation.z() << ','
                           << correction_orientation.w() << ','
                           << verification.audit_points << ',' << verification.audit_inliers << ','
                           << verification.audit_overlap << ',' << verification.audit_nearest_rmse << ','
                           << verification.audit_plane_points << ',' << verification.audit_plane_p90 << ','
                           << verification.audit_wall_points << ',' << verification.audit_wall_p90 << ','
                           << verification.audit_horizontal_points << ','
                           << verification.audit_horizontal_p90 << '\n';
}

void writeOdometryRow(
  std::ostream & stream, const double timestamp,
  const estimator::NominalState & state, const estimator::ErrorStateVector & covariance_diagonal)
{
  const Eigen::Quaterniond quaternion(state.R_WI);
  stream << std::setprecision(17) << timestamp << ',' << state.p_WI.x() << ','
            << state.p_WI.y() << ',' << state.p_WI.z() << ',' << quaternion.x() << ','
            << quaternion.y() << ',' << quaternion.z() << ',' << quaternion.w();
  stream << ',' << state.v_WI.x() << ',' << state.v_WI.y() << ',' << state.v_WI.z() << ','
            << state.g_W.x() << ',' << state.g_W.y() << ',' << state.g_W.z() << ','
            << state.b_g.x() << ',' << state.b_g.y() << ',' << state.b_g.z() << ','
            << state.b_a.x() << ',' << state.b_a.y() << ',' << state.b_a.z();
  for (int i = 0; i < estimator::ErrorStateLayout::kDimension; ++i) {
    stream << ',' << covariance_diagonal(i);
  }
  stream << '\n';
}

}  // namespace itera_lio::output::detail
