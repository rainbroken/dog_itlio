// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/config/module_loaders.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace itera_lio::ros::config {

NodeConfig loadNodeConfig(rclcpp::Node &node) {
  NodeConfig options;
  options.sensor = loadSensorConfig(node);
  options.processor = loadFrontendConfig(node);
  options.processor.pipeline.backend = loadBackendConfig(node);
  options.processor.pipeline.loop_closure = loadLoopConfig(node);
  options.output = loadOutputConfig(node);
  loadCameraConfig(node, options.output);
  options.evaluation = loadEvaluationConfig(node);
  const double frequency_hz = node.declare_parameter<double>("diagnostics.log_frequency_hz", 0.2);
  if (!std::isfinite(frequency_hz) || frequency_hz <= 0.0) {
    throw std::invalid_argument("diagnostics.log_frequency_hz must be finite and positive");
  }
  options.diagnostic_period_sec = 1.0 / frequency_hz;
  if (std::filesystem::absolute(options.output.pcd_directory) !=
      std::filesystem::absolute(options.evaluation.output_directory)) {
    throw std::invalid_argument("save_pcd.pcd_directory and evaluation.output_dir must match");
  }
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::system_clock::to_time_t(now);
  const auto milliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
  std::tm local_time{};
  localtime_r(&seconds, &local_time);
  std::ostringstream run_name;
  run_name << std::put_time(&local_time, "%Y%m%d_%H%M%S") << '_' << std::setfill('0')
           << std::setw(3) << milliseconds;
  const auto run_directory =
      std::filesystem::absolute(options.output.pcd_directory) / run_name.str();
  options.output.pcd_directory = run_directory.string();
  options.evaluation.output_directory = (run_directory / "evaluation").string();
  options.output.image_recorder.directory = (std::filesystem::path(options.output.pcd_directory) /
                                             options.output.image_recorder.directory)
                                                .string();
  return options;
}

} // namespace itera_lio::ros::config
