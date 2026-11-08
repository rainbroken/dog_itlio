// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__PUBLISHERS__OUTPUT_OPTIONS_HPP_
#define ITERA_LIO__ROS__PUBLISHERS__OUTPUT_OPTIONS_HPP_

#include "itera_lio/mapping/color/colorizer.hpp"
#include "itera_lio/output/image_recorder.hpp"
#include "itera_lio/output/static_map_filter.hpp"
#include <cstddef>
#include <string>

namespace itera_lio::ros {

struct OutputPublisherOptions {
  // Allow robot integration to own body and sensor TF without duplicate parents.
  bool publish_map_to_odom{true};
  bool publish_odom_to_imu{true};
  bool publish_imu_to_lidar{true};
  // Opt-in full local scans and processed IMU archive for offline refinement.
  bool refinement_archive{false};
  bool refine_after_save{false};
  std::string world_frame{"map"};
  std::string odom_frame{"odom"};
  std::string imu_frame{"imu"};
  std::string lidar_frame{"lidar"};
  std::string odometry_topic{"/lio/odom"};
  std::string path_topic{"/lio/path"};
  std::string optimized_path_topic{"/lio/optimized_path"};
  std::string optimized_map_topic{"/lio/optimized_map"};
  std::string registered_topic{"/lio/cloud_registered"};
  std::string deskewed_topic{"/lio/cloud_deskewed"};
  std::string local_map_topic{"/lio/local_map"};
  std::string global_map_topic{"/lio/global_map"};
  std::string map_updates_topic{"/lio/map_updates"};
  std::string effective_points_topic{"/lio/effective_points"};
  std::string colored_topic{"/lio/cloud_colored"};
  bool publish_deskewed{true};
  bool publish_registered{false};
  bool publish_effective{false};
  bool publish_local_map{true};
  bool publish_global_map{true};
  bool publish_map_updates{true};
  bool publish_optimized_map{true};
  // Zero publishes the complete snapshot on map initialization only.
  double global_map_snapshot_period_sec{0.0};
  float registered_voxel_size{0.05F};
  float global_voxel_size{0.10F};
  float optimized_map_voxel_size{0.20F};
  std::size_t max_preview_points{50000U};
  // Hard point limit for the LIO/RViz and LIVO/Web preview caches.
  std::size_t global_map_max_points{200000U};
  std::size_t path_max_poses{2000U};
  double path_publish_frequency_hz{5.0};
  bool full_deskewed_enabled{false};
  // Controls automatic PCD save at shutdown; manual snapshots are independent.
  bool save_pcd_on_shutdown{false};
  float pcd_voxel_size{0.05F};
  output::StaticMapFilterOptions static_filter;
  std::string pcd_directory{"data/pcd"};
  std::string pcd_source{"auto"};
  bool livo_enabled{false};
  std::string image_topic{"/camera1/image_raw"};
  std::string image_qos_reliability{"best_effort"};
  std::size_t image_qos_depth{16U};
  double image_sync_tolerance_sec{0.05};
  double image_sync_wait_sec{0.2};
  std::size_t image_queue_capacity{16U};
  mapping::color::CameraColorizerOptions camera;
  output::ImageRecorderOptions image_recorder;
};

} // namespace itera_lio::ros

#endif // ITERA_LIO__ROS__PUBLISHERS__OUTPUT_OPTIONS_HPP_
