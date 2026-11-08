// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/config/module_loaders.hpp"
#include "itera_lio/ros/parameter_reader.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace itera_lio::ros::config {

OutputPublisherOptions loadOutputConfig(rclcpp::Node &node) {
  ParameterReader reader(node);
  OutputPublisherOptions output_options;
  output_options.publish_map_to_odom = node.declare_parameter<bool>("tf.publish_map_to_odom", true);
  output_options.publish_odom_to_imu = node.declare_parameter<bool>("tf.publish_odom_to_imu", true);
  output_options.publish_imu_to_lidar = node.declare_parameter<bool>("tf.publish_imu_to_lidar", true);
  output_options.publish_deskewed = node.declare_parameter<bool>("debug.publish_deskewed", false);
  output_options.publish_registered =
      node.declare_parameter<bool>("debug.publish_registered", false);
  output_options.publish_effective = node.declare_parameter<bool>("debug.publish_effective", false);
  output_options.publish_local_map = node.declare_parameter<bool>("debug.publish_local_map", false);
  output_options.publish_global_map =
      node.declare_parameter<bool>("debug.publish_global_map", false);
  // RViz2 is launched by itera_lio.launch.py, but this parameter is part of
  // the same profile and must be declared because the complete YAML file is
  // also passed to this node.
  (void)node.declare_parameter<bool>("debug.launch_rviz", true);
  output_options.registered_voxel_size =
      static_cast<float>(node.declare_parameter<double>("preview.registered_voxel_size", 0.05));
  output_options.global_voxel_size =
      static_cast<float>(node.declare_parameter<double>("preview.global_map_voxel_size", 0.10));
  output_options.max_preview_points = reader.positiveSize("preview.registered_max_points", 50000);
  output_options.global_map_max_points =
      reader.positiveSize("preview.global_map_max_points", 200000);
  if (!std::isfinite(output_options.registered_voxel_size) ||
      output_options.registered_voxel_size <= 0.0F ||
      !std::isfinite(output_options.global_voxel_size) ||
      output_options.global_voxel_size <= 0.0F) {
    throw std::invalid_argument("preview voxel sizes must be finite and positive");
  }
  output_options.save_pcd_on_shutdown =
      node.declare_parameter<bool>("save_pcd.save_pcd_on_shutdown", false);
  output_options.pcd_voxel_size =
      static_cast<float>(node.declare_parameter<double>("save_pcd.voxel_size", 0.05));
  auto &filter = output_options.static_filter;
  filter.keep_debug_artifacts = node.declare_parameter<bool>("save_pcd.keep_debug_artifacts", false);
  filter.enabled = node.declare_parameter<bool>("save_pcd.dynamic_filter.enabled", false);
  filter.evidence_voxel_size = static_cast<float>(node.declare_parameter<double>(
      "save_pcd.dynamic_filter.evidence_voxel_size", 0.20));
  filter.max_range = node.declare_parameter<double>("save_pcd.dynamic_filter.max_range", 25.0);
  filter.endpoint_margin = node.declare_parameter<double>("save_pcd.dynamic_filter.endpoint_margin", 0.15);
  filter.margin_per_meter = node.declare_parameter<double>("save_pcd.dynamic_filter.margin_per_meter", 0.005);
  filter.max_endpoint_margin = node.declare_parameter<double>("save_pcd.dynamic_filter.max_endpoint_margin", 0.35);
  filter.observer_voxel_size = node.declare_parameter<double>("save_pcd.dynamic_filter.observer_voxel_size", 0.50);
  const auto min_free = node.declare_parameter<int64_t>("save_pcd.dynamic_filter.min_free_observations", 3);
  if (min_free < 2 || min_free > 100000) {throw std::invalid_argument("invalid minimum free observations");}
  filter.min_free_observations = static_cast<unsigned>(min_free);
  filter.min_scan_interval = node.declare_parameter<double>("save_pcd.dynamic_filter.min_scan_interval", 0.20);
  filter.validate();
  if (!std::isfinite(output_options.pcd_voxel_size) || output_options.pcd_voxel_size < 0.0F) {
    throw std::invalid_argument("save_pcd.voxel_size must be finite and non-negative");
  }
  output_options.refinement_archive =
      node.declare_parameter<bool>("save_pcd.refinement_archive", false);
  output_options.refine_after_save =
      node.declare_parameter<bool>("save_pcd.refine_after_save", false);
  output_options.refinement_archive |= output_options.refine_after_save;
  // LIO auto saves complete deskewed scans; explicit sources select archives.
  output_options.pcd_source = node.declare_parameter<std::string>("save_pcd.source", "auto");
  if (output_options.pcd_source != "auto" && output_options.pcd_source != "full_deskewed") {
    throw std::invalid_argument("save_pcd.source must be auto or full_deskewed");
  }
  output_options.pcd_directory =
      node.declare_parameter<std::string>("save_pcd.pcd_directory", "data");
  if (output_options.pcd_directory.empty()) {
    throw std::invalid_argument("save_pcd.pcd_directory must not be empty");
  }
  output_options.world_frame = node.declare_parameter<std::string>("frames.world", "map");
  output_options.odom_frame = node.declare_parameter<std::string>("frames.odom", "odom");
  output_options.imu_frame = node.declare_parameter<std::string>("frames.imu", "imu");
  output_options.lidar_frame = node.declare_parameter<std::string>("frames.lidar", "lidar");
  const std::vector<std::string> frames{output_options.world_frame, output_options.odom_frame,
    output_options.imu_frame, output_options.lidar_frame};
  for (std::size_t i = 0; i < frames.size(); ++i) {
    if (frames[i].empty()) {throw std::invalid_argument("frame names must not be empty");}
    for (std::size_t j = 0; j < i; ++j) {
      if (frames[i] == frames[j]) {throw std::invalid_argument("TF frame names must be distinct");}
    }
  }
  output_options.odometry_topic =
      node.declare_parameter<std::string>("topics.odometry", "/lio/odom");
  output_options.path_topic = node.declare_parameter<std::string>("topics.path", "/lio/path");
  output_options.optimized_path_topic =
      node.declare_parameter<std::string>("topics.optimized_path", "/lio/optimized_path");
  output_options.optimized_map_topic =
      node.declare_parameter<std::string>("topics.optimized_map", "/lio/optimized_map");
  output_options.registered_topic =
      node.declare_parameter<std::string>("topics.cloud_registered", "/lio/cloud_registered");
  output_options.deskewed_topic =
      node.declare_parameter<std::string>("topics.cloud_deskewed", "/lio/cloud_deskewed");
  output_options.local_map_topic =
      node.declare_parameter<std::string>("topics.local_map", "/lio/local_map");
  output_options.global_map_topic =
      node.declare_parameter<std::string>("topics.global_map", "/lio/global_map");
  output_options.map_updates_topic =
      node.declare_parameter<std::string>("topics.map_updates", "/lio/map_updates");
  output_options.effective_points_topic =
      node.declare_parameter<std::string>("topics.effective_points", "/lio/effective_points");
  output_options.colored_topic =
      node.declare_parameter<std::string>("topics.cloud_colored", "/lio/cloud_colored");
  output_options.publish_map_updates =
      node.declare_parameter<bool>("debug.publish_map_updates", true);
  output_options.publish_optimized_map =
      node.declare_parameter<bool>("debug.publish_optimized_map", true);
  const double global_map_snapshot_frequency_hz =
      node.declare_parameter<double>("preview.global_map_snapshot_frequency_hz", 0.0);
  if (!std::isfinite(global_map_snapshot_frequency_hz) || global_map_snapshot_frequency_hz < 0.0) {
    throw std::invalid_argument("preview.global_map_snapshot_frequency_hz must be finite and "
                                "non-negative");
  }
  output_options.global_map_snapshot_period_sec =
      global_map_snapshot_frequency_hz > 0.0 ? 1.0 / global_map_snapshot_frequency_hz : 0.0;
  output_options.optimized_map_voxel_size =
      static_cast<float>(node.declare_parameter<double>("preview.optimized_map_voxel_size", 0.20));
  const auto path_max_poses = node.declare_parameter<int64_t>("path.max_poses", 0);
  if (path_max_poses < 0) {
    throw std::invalid_argument("path.max_poses must be non-negative; zero means unlimited");
  }
  output_options.path_max_poses = static_cast<std::size_t>(path_max_poses);
  output_options.path_publish_frequency_hz =
      node.declare_parameter<double>("path.publish_frequency_hz", 5.0);
  output_options.full_deskewed_enabled = true;
  return output_options;
}

} // namespace itera_lio::ros::config
