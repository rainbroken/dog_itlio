// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/config/module_loaders.hpp"
#include "itera_lio/ros/parameter_reader.hpp"
#include <stdexcept>

namespace itera_lio::ros::config {

loop::LoopClosureOptions loadLoopConfig(rclcpp::Node &node) {
  ParameterReader reader(node);
  loop::LoopClosureOptions options;
  options.enabled = node.declare_parameter<bool>("loop_closure.enabled", false);
  options.detector.min_keyframe_separation =
      reader.positiveSize("loop_closure.detector.min_keyframe_separation", 30);
  options.detector.max_candidates = reader.positiveSize("loop_closure.detector.max_candidates", 1);
  options.detector.max_alignment_candidates =
      reader.positiveSize("loop_closure.detector.max_alignment_candidates", 20);
  options.detector.min_time_separation_sec =
      node.declare_parameter<double>("loop_closure.detector.min_time_separation_sec", 30.0);
  options.detector.max_descriptor_distance =
      node.declare_parameter<double>("loop_closure.detector.max_descriptor_distance", 0.35);
  options.detector.min_descriptor_overlap_ratio =
      node.declare_parameter<double>("loop_closure.detector.min_descriptor_overlap_ratio", 0.10);
  options.detector.max_odometry_vertical_separation =
      node.declare_parameter<double>("loop_closure.detector.max_odometry_vertical_separation", 1.5);
  options.detector.max_gravity_angle_rad =
      node.declare_parameter<double>("loop_closure.detector.max_gravity_angle_rad", 0.2);
  options.verifier.registration_engine = node.declare_parameter<std::string>(
      "loop_closure.verifier.registration_engine", "manual_gicp");
  if (options.verifier.registration_engine != "manual_gicp") {
    throw std::invalid_argument("loop_closure.verifier.registration_engine must be manual_gicp");
  }
  options.verifier.min_points = reader.positiveSize("loop_closure.verifier.min_points", 100);
  options.verifier.min_inliers = reader.positiveSize("loop_closure.verifier.min_inliers", 100);
  options.verifier.max_fitness =
      node.declare_parameter<double>("loop_closure.verifier.max_fitness", 0.05);
  options.verifier.max_translation_jump =
      node.declare_parameter<double>("loop_closure.verifier.max_translation_jump", 20.0);
  options.verifier.max_rotation_rad =
      node.declare_parameter<double>("loop_closure.verifier.max_rotation_rad", 1.0);
  options.verifier.max_correspondence_distance =
      node.declare_parameter<double>("loop_closure.verifier.max_correspondence_distance", 1.5);
  options.verifier.min_inlier_ratio =
      node.declare_parameter<double>("loop_closure.verifier.min_inlier_ratio", 0.4);
  options.verifier.min_reciprocal_inlier_ratio =
      node.declare_parameter<double>("loop_closure.verifier.min_reciprocal_inlier_ratio", 0.25);
  options.verifier.min_geometry_eigenvalue_ratio =
      node.declare_parameter<double>("loop_closure.verifier.min_geometry_eigenvalue_ratio", 1e-4);
  options.verifier.max_normal_eigenvalue_ratio =
      node.declare_parameter<double>("loop_closure.verifier.max_normal_eigenvalue_ratio", 0.2);
  options.verifier.max_gravity_tilt_correction_rad =
      node.declare_parameter<double>("loop_closure.verifier.max_gravity_tilt_correction_rad", 0.10);
  options.verifier.max_vertical_correction =
      node.declare_parameter<double>("loop_closure.verifier.max_vertical_correction", 1.0);
  options.verifier.normal_neighbors =
      static_cast<int>(reader.positiveSize("loop_closure.verifier.normal_neighbors", 10));
  options.verifier.inlier_distance =
      node.declare_parameter<double>("loop_closure.verifier.inlier_distance", 0.45);
  options.verifier.gicp_voxel_size =
      node.declare_parameter<double>("loop_closure.verifier.gicp_voxel_size", 0.30);
  options.verifier.robust_kernel_scale =
      node.declare_parameter<double>("loop_closure.verifier.robust_kernel_scale", 1.0);
  options.verifier.num_threads =
      node.declare_parameter<int>("loop_closure.verifier.num_threads", 2);
  options.verifier.max_iterations =
      node.declare_parameter<int>("loop_closure.verifier.max_iterations", 40);
  options.verifier.max_source_points =
      reader.positiveSize("loop_closure.verifier.max_source_points", 2000);
  options.verifier.max_target_points =
      reader.positiveSize("loop_closure.verifier.max_target_points", 10000);
  options.submap_half_window = reader.positiveSize("loop_closure.submap_half_window", 10);
  options.min_attempt_interval_sec =
      node.declare_parameter<double>("loop_closure.min_attempt_interval_sec", 5.0);
  options.confirmation_count = reader.positiveSize("loop_closure.confirmation.count", 2);
  options.confirmation_translation_tolerance =
      node.declare_parameter<double>("loop_closure.confirmation.translation_tolerance", 2.0);
  options.confirmation_rotation_tolerance_rad =
      node.declare_parameter<double>("loop_closure.confirmation.rotation_tolerance_rad", 0.10);
  options.strong_evidence.enabled =
      node.declare_parameter<bool>("loop_closure.confirmation.strong_single_frame.enabled", false);
  options.strong_evidence.min_inliers =
      reader.positiveSize("loop_closure.confirmation.strong_single_frame.min_inliers", 150);
  options.strong_evidence.max_descriptor_distance = node.declare_parameter<double>(
      "loop_closure.confirmation.strong_single_frame.max_descriptor_distance", 0.20);
  options.strong_evidence.max_fitness = node.declare_parameter<double>(
      "loop_closure.confirmation.strong_single_frame.max_fitness", 0.025);
  options.strong_evidence.min_reciprocal_inlier_ratio = node.declare_parameter<double>(
      "loop_closure.confirmation.strong_single_frame.min_reciprocal_inlier_ratio", 0.90);
  options.strong_evidence.max_geometry_condition_number = node.declare_parameter<double>(
      "loop_closure.confirmation.strong_single_frame.max_geometry_condition_number", 30.0);
  options.strong_evidence.max_gravity_residual_rad = node.declare_parameter<double>(
      "loop_closure.confirmation.strong_single_frame.max_gravity_residual_rad", 0.05);
  options.strong_evidence.max_vertical_separation = node.declare_parameter<double>(
      "loop_closure.confirmation.strong_single_frame.max_vertical_separation", 0.20);
  options.keyframe_voxel_size =
      static_cast<float>(node.declare_parameter<double>("loop_closure.keyframe_voxel_size", 0.20));
  options.max_keyframe_points = reader.positiveSize("loop_closure.max_keyframe_points", 4000);
  options.worker_queue_capacity = reader.positiveSize("loop_closure.worker.queue_capacity", 2);
  options.task_timeout_ms =
      node.declare_parameter<double>("loop_closure.worker.task_timeout_ms", 1000.0);
  options.max_resident_keyframe_clouds =
      reader.positiveSize("loop_closure.storage.max_resident_keyframe_clouds", 200);
  options.archive_directory = node.declare_parameter<std::string>(
      "loop_closure.storage.archive_directory", "data/loop_keyframes");
  options.diagnostic_pair_directory=node.declare_parameter<std::string>(
      "loop_closure.diagnostics.pair_directory", "");
  options.diagnostic_pair_limit=reader.positiveSize("loop_closure.diagnostics.max_pairs",128);
  return options;
}

} // namespace itera_lio::ros::config
