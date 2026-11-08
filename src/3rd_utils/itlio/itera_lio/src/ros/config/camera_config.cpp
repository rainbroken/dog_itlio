// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/config/module_loaders.hpp"
#include "itera_lio/ros/parameter_reader.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace itera_lio::ros::config {

void loadCameraConfig(rclcpp::Node &node, OutputPublisherOptions &output_options) {
  ParameterReader reader(node);
  const auto mode = node.declare_parameter<std::string>("mode.type", "lio");
  if (mode != "lio" && mode != "livo") {
    throw std::invalid_argument("mode.type must be lio or livo");
  }
  output_options.livo_enabled = mode == "livo";
  const std::string camera_image_topic =
      node.declare_parameter<std::string>("camera.image_topic", "/camera1/image_raw");
  const double camera_sync_tolerance =
      node.declare_parameter<double>("camera.sync_tolerance_sec", 0.05);
  if (!std::isfinite(camera_sync_tolerance) || camera_sync_tolerance <= 0.0) {
    throw std::invalid_argument("camera.sync_tolerance_sec must be positive");
  }
  output_options.image_topic = camera_image_topic;
  output_options.camera.image_scale = node.declare_parameter<double>("camera.image_scale", 1.0);
  if (!std::isfinite(output_options.camera.image_scale) ||
      output_options.camera.image_scale <= 0.0) {
    throw std::invalid_argument("camera.image_scale must be finite and positive");
  }
  output_options.camera.width = node.declare_parameter<int>("camera.width", 1280);
  output_options.camera.height = node.declare_parameter<int>("camera.height", 720);
  if (output_options.camera.width <= 0 || output_options.camera.height <= 0) {
    throw std::invalid_argument("camera.width and camera.height must be positive");
  }
  output_options.image_qos_reliability =
      node.declare_parameter<std::string>("camera.qos.reliability", "best_effort");
  output_options.image_qos_depth = reader.positiveSize("camera.qos.depth", 16);
  output_options.image_sync_tolerance_sec = camera_sync_tolerance;
  output_options.image_sync_wait_sec = node.declare_parameter<double>("camera.sync_wait_sec", 0.20);
  output_options.image_queue_capacity = reader.positiveSize("camera.image_queue_capacity", 16);
  output_options.image_recorder.enabled =
      node.declare_parameter<bool>("image_record.enabled", false);
  output_options.image_recorder.directory =
      node.declare_parameter<std::string>("image_record.directory", "images");
  output_options.image_recorder.max_rate_hz =
      node.declare_parameter<double>("image_record.max_rate_hz", 5.0);
  output_options.image_recorder.max_interval_sec =
      node.declare_parameter<double>("image_record.max_interval_sec", 0.5);
  (void)node.declare_parameter<double>("image_record.min_translation_m", 0.08);
  (void)node.declare_parameter<double>("image_record.min_rotation_deg", 4.0);
  const auto camera_r = node.declare_parameter<std::vector<double>>(
      "camera.extrinsic.lidar_to_camera_rotation",
      {-0.007823, -0.999947, -0.006722, 0.353953, 0.003518, -0.935256, 0.935230, -0.009696,
       0.353907});
  const auto camera_t = node.declare_parameter<std::vector<double>>(
      "camera.extrinsic.lidar_to_camera_translation", {0.128979, 0.068637, -0.016452});
  output_options.camera.fx = node.declare_parameter<double>("camera.intrinsics.fx", 907.292);
  output_options.camera.fy = node.declare_parameter<double>("camera.intrinsics.fy", 903.016);
  output_options.camera.cx = node.declare_parameter<double>("camera.intrinsics.cx", 632.5);
  output_options.camera.cy = node.declare_parameter<double>("camera.intrinsics.cy", 365.557);
  output_options.camera.k1 = node.declare_parameter<double>("camera.intrinsics.k1", 0.074039);
  output_options.camera.k2 = node.declare_parameter<double>("camera.intrinsics.k2", -0.114819);
  output_options.camera.p1 = node.declare_parameter<double>("camera.intrinsics.p1", 0.002575);
  output_options.camera.p2 = node.declare_parameter<double>("camera.intrinsics.p2", -0.004635);
  if (!std::isfinite(output_options.camera.fx) || !std::isfinite(output_options.camera.fy) ||
      !std::isfinite(output_options.camera.cx) || !std::isfinite(output_options.camera.cy) ||
      !std::isfinite(output_options.camera.k1) || !std::isfinite(output_options.camera.k2) ||
      !std::isfinite(output_options.camera.p1) || !std::isfinite(output_options.camera.p2) ||
      output_options.camera.fx <= 0.0 || output_options.camera.fy <= 0.0) {
    throw std::invalid_argument(
        "camera intrinsics must be finite and focal lengths must be positive");
  }
  if (camera_r.size() != 9 || camera_t.size() != 3) {
    throw std::invalid_argument("camera extrinsic dimensions invalid");
  }
  for (int i = 0; i < 9; ++i) {
    output_options.camera.R_CL(i / 3, i % 3) = camera_r[static_cast<std::size_t>(i)];
  }
  for (int i = 0; i < 3; ++i) {
    output_options.camera.p_CL(i) = camera_t[static_cast<std::size_t>(i)];
  }
}

} // namespace itera_lio::ros::config
