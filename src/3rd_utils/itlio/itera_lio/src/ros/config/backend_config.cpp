// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/config/module_loaders.hpp"
#include "itera_lio/ros/parameter_reader.hpp"

namespace itera_lio::ros::config {

backend::BackendOptions loadBackendConfig(rclcpp::Node &node) {
  ParameterReader reader(node);
  backend::BackendOptions options;
  options.enabled = node.declare_parameter<bool>("backend.enabled", false);
  options.keyframes.min_translation =
      node.declare_parameter<double>("backend.keyframe.min_translation", 1.0);
  options.keyframes.min_rotation_rad =
      node.declare_parameter<double>("backend.keyframe.min_rotation_rad", 0.1745329252);
  options.keyframes.max_interval_sec =
      node.declare_parameter<double>("backend.keyframe.max_interval_sec", 10.0);
  options.odometry_rotation_stddev =
      node.declare_parameter<double>("backend.odometry.rotation_stddev", 0.05);
  options.odometry_translation_stddev =
      node.declare_parameter<double>("backend.odometry.translation_stddev", 0.10);
  options.asynchronous_optimization = node.declare_parameter<bool>("backend.worker.enabled", true);
  options.worker_queue_capacity = reader.positiveSize("backend.worker.queue_capacity", 2);
  options.task_timeout_ms =
      node.declare_parameter<double>("backend.worker.task_timeout_ms", 2000.0);
  options.optimizer.max_iterations =
      node.declare_parameter<int>("backend.optimizer.max_iterations", 10);
  options.optimizer.convergence_threshold =
      node.declare_parameter<double>("backend.optimizer.convergence_threshold", 1e-5);
  options.optimizer.damping = node.declare_parameter<double>("backend.optimizer.damping", 1e-6);
  options.optimizer.huber_delta =
      node.declare_parameter<double>("backend.optimizer.huber_delta", 3.0);
  options.optimizer.active_window_size =
      reader.positiveSize("backend.optimizer.active_window_size", 500);
  return options;
}

} // namespace itera_lio::ros::config
