// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/config/module_loaders.hpp"

#include <cmath>
#include <stdexcept>

namespace itera_lio::ros::config {

EvaluationConfig loadEvaluationConfig(rclcpp::Node &node) {
  EvaluationConfig options;
  options.enabled = node.declare_parameter<bool>("evaluation.enabled", true);
  options.output_directory = node.declare_parameter<std::string>("evaluation.output_dir", "data");
  auto &evaluation_options = options.outputs;
  evaluation_options.detailed = node.declare_parameter<bool>("evaluation.detailed", false);
  evaluation_options.slow_rebuild_ms =
      node.declare_parameter<double>("evaluation.slow_rebuild_ms", 100.0);
  evaluation_options.frames = node.declare_parameter<bool>("evaluation.outputs.frames", true);
  evaluation_options.runtime = node.declare_parameter<bool>("evaluation.outputs.runtime", true);
  evaluation_options.events = node.declare_parameter<bool>("evaluation.outputs.events", true);
  evaluation_options.loop_verifications =
      node.declare_parameter<bool>("evaluation.outputs.loop_verifications", true);
  evaluation_options.summary = node.declare_parameter<bool>("evaluation.outputs.summary", true);
  evaluation_options.odometry = node.declare_parameter<bool>("evaluation.outputs.odometry", true);
  evaluation_options.parameters =
      node.declare_parameter<bool>("evaluation.outputs.parameters", true);
  evaluation_options.manifest = node.declare_parameter<bool>("evaluation.outputs.manifest", true);
  evaluation_options.runtime_frequency_hz =
      node.declare_parameter<double>("evaluation.runtime_frequency_hz", 0.2);
  const double comparison_warmup_sec =
      node.declare_parameter<double>("evaluation.comparison.warmup_sec", 2.0);
  const double comparison_percentile =
      node.declare_parameter<double>("evaluation.comparison.percentile", 0.95);
  evaluation_options.warmup_sec = comparison_warmup_sec;
  evaluation_options.comparison_percentile = comparison_percentile;
  if (!std::isfinite(evaluation_options.slow_rebuild_ms) ||
      evaluation_options.slow_rebuild_ms <= 0.0 ||
      !std::isfinite(evaluation_options.runtime_frequency_hz) ||
      evaluation_options.runtime_frequency_hz <= 0.0 || !std::isfinite(comparison_warmup_sec) ||
      comparison_warmup_sec < 0.0 || !std::isfinite(comparison_percentile) ||
      comparison_percentile <= 0.0 || comparison_percentile >= 1.0) {
    throw std::invalid_argument("evaluation frequency and comparison parameters must be finite and "
                                "in valid ranges");
  }
  return options;
}

} // namespace itera_lio::ros::config
