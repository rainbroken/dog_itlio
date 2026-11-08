// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/config/module_loaders.hpp"
#include "itera_lio/ros/parameter_reader.hpp"

#include <cmath>
#include <stdexcept>

namespace itera_lio::ros::config {

system::LioProcessorOptions loadFrontendConfig(rclcpp::Node &node) {
  ParameterReader reader(node);
  system::LioProcessorOptions processor_options;
  processor_options.imu_history_max_samples =
      reader.positiveSize("sensor.imu_history_max_samples", 3000);
  processor_options.imu_history_duration_sec =
      node.declare_parameter<double>("sensor.imu_history_duration_sec", 0.5);
  processor_options.max_imu_interval_sec = node.declare_parameter<double>("imu.max_interval", 0.05);
  processor_options.max_pending_lidar = reader.positiveSize("sensor.max_pending_lidar", 8);
  processor_options.imu_initialization.duration_sec =
      node.declare_parameter<double>("imu.init_duration", 2.0);
  processor_options.imu_initialization.gyro_std_threshold =
      node.declare_parameter<double>("imu.gyro_std_threshold", 0.1);
  processor_options.imu_initialization.accel_std_threshold =
      node.declare_parameter<double>("imu.accel_std_threshold", 0.3);
  processor_options.imu_initialization.gyro_mean_threshold =
      node.declare_parameter<double>("imu.gyro_mean_threshold", 0.15);
  processor_options.imu_initialization.accel_norm_threshold =
      node.declare_parameter<double>("imu.accel_norm_threshold", 0.3);
  processor_options.imu_initialization.gravity_magnitude =
      node.declare_parameter<double>("imu.gravity_magnitude", 9.81);
  processor_options.imu_initialization.initialize_accel_bias_from_static_mean =
      node.declare_parameter<bool>("imu.initialize_accel_bias_from_static_mean", true);
  processor_options.imu_initialization.min_samples =
      reader.positiveSize("imu.init_min_samples", 20);
  processor_options.initialize_covariance_from_static_window =
      node.declare_parameter<bool>("imu.initialize_covariance_from_static_window", false);
  processor_options.initial_rotation_stddev =
      node.declare_parameter<double>("imu.initial_rotation_stddev", 0.001);
  processor_options.initial_position_stddev =
      node.declare_parameter<double>("imu.initial_position_stddev", 0.001);
  processor_options.initial_velocity_stddev =
      node.declare_parameter<double>("imu.initial_velocity_stddev", 0.01);
  processor_options.initial_accel_bias_stddev =
      node.declare_parameter<double>("imu.initial_accel_bias_stddev", 0.1);
  processor_options.imu_noise.gyro_noise_density =
      node.declare_parameter<double>("imu.gyro_noise_density", 0.01);
  processor_options.imu_noise.accel_noise_density =
      node.declare_parameter<double>("imu.accel_noise_density", 0.1);
  processor_options.imu_noise.gyro_bias_noise_density =
      node.declare_parameter<double>("imu.gyro_bias_noise_density", 0.001);
  processor_options.imu_noise.accel_bias_noise_density =
      node.declare_parameter<double>("imu.accel_bias_noise_density", 0.01);
  processor_options.lidar_min_range = node.declare_parameter<double>("lidar.min_range", 0.5);
  processor_options.point_filter_num =
      reader.positiveSize("preprocess.point_filter_num", 1);
  // Full scans feed persistent output; registration still uses sampled scans.
  processor_options.full_deskewed_enabled = true;

  auto &pipeline_options = processor_options.pipeline;
  pipeline_options.registration.num_threads = static_cast<int>(
      reader.positiveSize("registration.num_threads", 3));
  pipeline_options.registration.knn = static_cast<int>(reader.positiveSize("registration.knn", 5));
  pipeline_options.registration.max_neighbor_distance =
      static_cast<float>(node.declare_parameter<double>("registration.max_neighbor_distance", 3.0));
  pipeline_options.registration.plane.max_neighbor_distance =
      pipeline_options.registration.max_neighbor_distance;
  pipeline_options.registration.residual_threshold =
      node.declare_parameter<double>("registration.residual_threshold", 0.5);
  pipeline_options.registration.max_effective_points =
      reader.positiveSize("registration.max_effective_points", 200);
  pipeline_options.registration.max_candidate_points =
      reader.positiveSize("registration.max_candidate_points", 2000);
  pipeline_options.registration.max_map_insert_points =
      reader.positiveSize("registration.max_map_insert_points", 1000);
  pipeline_options.registration.adaptive_candidates =
      node.declare_parameter<bool>("registration.adaptive_candidates", false);
  pipeline_options.registration.reuse_max_displacement =
      node.declare_parameter<double>("registration.reuse_max_displacement", 0.05);
  pipeline_options.registration.selection_voxel_size =
      node.declare_parameter<double>("registration.selection_voxel_size", 1.0);
  pipeline_options.registration.normal_azimuth_bins =
      static_cast<int>(reader.positiveSize("registration.normal_azimuth_bins", 8));
  pipeline_options.registration.normal_elevation_bins =
      static_cast<int>(reader.positiveSize("registration.normal_elevation_bins", 4));
  pipeline_options.registration.plane.max_point_to_plane_distance =
      node.declare_parameter<double>("registration.plane.max_distance", 0.2);
  pipeline_options.registration.plane.max_eigenvalue_ratio =
      node.declare_parameter<double>("registration.plane.max_eigenvalue_ratio", 0.1);
  pipeline_options.registration.plane.min_normal_norm =
      node.declare_parameter<double>("registration.plane.min_normal_norm", 0.9);
  pipeline_options.registration.map_insert_residual_threshold =
      node.declare_parameter<double>("registration.map_insert_residual_threshold", 0.5);
  pipeline_options.registration.require_converged_for_map_insert =
      node.declare_parameter<bool>("registration.require_converged_for_map_insert", false);
  pipeline_options.registration.enable_map_insert_quality_gate =
      node.declare_parameter<bool>("registration.enable_map_insert_quality_gate", false);
  pipeline_options.registration.map_insert_max_mean_residual =
      node.declare_parameter<double>("registration.map_insert_max_mean_residual", 0.05);
  pipeline_options.registration.map_insert_max_rotation_correction =
      node.declare_parameter<double>("registration.map_insert_max_rotation_correction", 0.15);
  pipeline_options.registration.map_insert_max_translation_correction =
      node.declare_parameter<double>("registration.map_insert_max_translation_correction", 0.3);
  pipeline_options.registration.map_insert_min_effective_ratio =
      node.declare_parameter<double>("registration.map_insert_min_effective_ratio", 0.2);
  if (!(pipeline_options.registration.map_insert_min_effective_ratio >= 0.0 &&
        pipeline_options.registration.map_insert_min_effective_ratio <= 1.0)) {
    throw std::invalid_argument("registration.map_insert_min_effective_ratio must be in [0,1]");
  }
  for (const double limit : {pipeline_options.registration.map_insert_max_mean_residual,
                             pipeline_options.registration.map_insert_max_rotation_correction,
                             pipeline_options.registration.map_insert_max_translation_correction}) {
    if (!std::isfinite(limit) || limit <= 0.0) {
      throw std::invalid_argument("map insertion quality limits must be finite and positive");
    }
  }
  pipeline_options.registration.plane.enable_quality_checks =
      node.declare_parameter<bool>("registration.plane.enable_quality_checks", true);
  pipeline_options.registration.plane.min_tangent_ratio =
      node.declare_parameter<double>("registration.plane.min_tangent_ratio", 0.02);
  pipeline_options.registration.plane.max_normal_ratio =
      node.declare_parameter<double>("registration.plane.max_normal_ratio", 0.1);
  pipeline_options.map_voxel_size =
      static_cast<float>(node.declare_parameter<double>("voxel.map_size", 0.3));
  const std::string rebuild_policy =
      node.declare_parameter<std::string>("ikd_tree.rebuild_policy", "hybrid");
  if (rebuild_policy == "synchronous") {
    pipeline_options.map_rebuild_policy = frontend::IkdRebuildPolicy::kSynchronous;
  } else if (rebuild_policy == "asynchronous") {
    pipeline_options.map_rebuild_policy = frontend::IkdRebuildPolicy::kAsynchronous;
  } else if (rebuild_policy == "hybrid") {
    pipeline_options.map_rebuild_policy = frontend::IkdRebuildPolicy::kHybrid;
  } else {
    throw std::invalid_argument("ikd_tree.rebuild_policy must be "
                                "synchronous, asynchronous, or hybrid");
  }
  pipeline_options.map_async_rebuild_min_subtree_points =
      reader.positiveSize("ikd_tree.asynchronous_min_subtree_points", 20000);
  pipeline_options.scan_voxel_size =
      static_cast<float>(node.declare_parameter<double>("voxel.scan_size", 0.2));
  processor_options.deskew_voxel_size =
      static_cast<float>(node.declare_parameter<double>("voxel.deskew_size", 0.2));
  // Deskew input is sampled before the per-point pose interpolation. When
  // both voxel sizes are equal, let the pipeline reuse that sampled frame
  // instead of hashing the same points a second time.
  pipeline_options.input_prevoxelized =
      std::isfinite(pipeline_options.scan_voxel_size) &&
      std::abs(pipeline_options.scan_voxel_size - processor_options.deskew_voxel_size) <= 1e-6F;
  pipeline_options.local_map.cube_size =
      static_cast<float>(node.declare_parameter<double>("local_map.cube_size", 100.0));
  pipeline_options.local_map.move_threshold =
      static_cast<float>(node.declare_parameter<double>("local_map.move_threshold", 20.0));
  pipeline_options.local_map.delete_chunk_size =
      static_cast<float>(node.declare_parameter<double>("local_map.delete_chunk_size", 10.0));
  pipeline_options.local_map.max_delete_boxes_per_update =
      reader.positiveSize("local_map.max_delete_boxes_per_update", 1);
  pipeline_options.local_map.max_delete_points_per_update =
      reader.positiveSize("local_map.max_delete_points_per_update", 5000);
  pipeline_options.local_map.max_delete_split_depth =
      reader.positiveSize("local_map.max_delete_split_depth", 12);
  pipeline_options.correction.max_iterations =
      static_cast<int>(reader.positiveSize("registration.max_iterations", 4));
  pipeline_options.correction.measurement_stddev =
      node.declare_parameter<double>("registration.measurement_stddev", 0.05);
  pipeline_options.correction.enable_mahalanobis_gate =
      node.declare_parameter<bool>("registration.enable_mahalanobis_gate", true);
  pipeline_options.correction.chi_square_threshold =
      node.declare_parameter<double>("registration.chi_square_threshold", 9.21);
  pipeline_options.correction.enable_huber =
      node.declare_parameter<bool>("registration.enable_huber", true);
  pipeline_options.correction.huber_delta =
      node.declare_parameter<double>("registration.huber_delta", 2.5);
  pipeline_options.correction.update_gravity =
      node.declare_parameter<bool>("imu.update_gravity_after_initialization", false);
  pipeline_options.correction.gravity_magnitude =
      processor_options.imu_initialization.gravity_magnitude;
  pipeline_options.correction.gravity_magnitude_stddev =
      node.declare_parameter<double>("imu.gravity_magnitude_stddev", 0.0);
  pipeline_options.correction.freeze_weak_directions =
      node.declare_parameter<bool>("registration.freeze_weak_directions", true);
  pipeline_options.correction.weak_direction_relative_threshold =
      node.declare_parameter<double>("registration.weak_direction_relative_threshold", 1e-3);
  pipeline_options.correction.weak_direction_weight =
      node.declare_parameter<double>("registration.weak_direction_weight", 0.1);
  pipeline_options.correction.rotation_convergence =
      node.declare_parameter<double>("registration.rotation_convergence", 1e-3);
  pipeline_options.correction.translation_convergence =
      node.declare_parameter<double>("registration.translation_convergence", 1e-3);
  pipeline_options.degeneracy.minimum_eigenvalue =
      node.declare_parameter<double>("registration.degeneracy.minimum_eigenvalue", 1e-6);
  pipeline_options.degeneracy.maximum_condition_number =
      node.declare_parameter<double>("registration.degeneracy.maximum_condition_number", 1e8);
  pipeline_options.tracking.minimum_effective_points =
      reader.positiveSize("registration.tracking.minimum_effective_points", 20);
  pipeline_options.tracking.lost_effective_points =
      reader.nonNegativeSize("registration.tracking.lost_effective_points", 10);
  pipeline_options.tracking.maximum_mean_residual =
      node.declare_parameter<double>("registration.tracking.maximum_mean_residual", 0.25);
  pipeline_options.tracking.minimum_whitened_eigenvalue =
      node.declare_parameter<double>("registration.tracking.minimum_whitened_eigenvalue", 1e-2);
  pipeline_options.tracking.maximum_whitened_condition_number = node.declare_parameter<double>(
      "registration.tracking.maximum_whitened_condition_number", 500.0);
  pipeline_options.tracking.degraded_frames =
      reader.positiveSize("registration.tracking.degraded_frames", 5);
  pipeline_options.tracking.lost_frames =
      reader.positiveSize("registration.tracking.lost_frames", 5);
  pipeline_options.tracking.recovery_frames =
      reader.positiveSize("registration.tracking.recovery_frames", 10);
  pipeline_options.tracking.allow_automatic_recovery =
      node.declare_parameter<bool>("registration.tracking.allow_automatic_recovery", false);
  processor_options.lidar_extrinsics = reader.lidarExtrinsics();
  return processor_options;
}

} // namespace itera_lio::ros::config
