// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "itera_lio/ros/evaluation_reporter.hpp"

#include <utility>
#include <vector>

namespace itera_lio::ros {

EvaluationReporter::EvaluationReporter(
    rclcpp::Node &node, const bool enabled, const std::string &output_directory,
    const output::EvaluationMetadata &metadata,
    const output::EvaluationOutputOptions &options)
    : node_(node), enabled_(enabled), options_(options) {
  if (!enabled_) {
    return;
  }
  if (!options_.anyEnabled()) {
    enabled_ = false;
    RCLCPP_INFO(node_.get_logger(),
                "Evaluation logging disabled: all outputs are disabled");
    return;
  }
  const auto parameter_names = node_.list_parameters({}, 10U).names;
  const auto parameter_values = node_.get_parameters(parameter_names);
  std::vector<std::pair<std::string, std::string>> parameters;
  parameters.reserve(parameter_values.size());
  for (const auto &parameter : parameter_values) {
    parameters.emplace_back(parameter.get_name(), parameter.value_to_string());
  }
  std::string error;
  if (!logger_.initialize(output_directory, metadata, options_, parameters,
                          error)) {
    enabled_ = false;
    RCLCPP_ERROR(node_.get_logger(), "Evaluation logging disabled: %s",
                 error.c_str());
    return;
  }
  RCLCPP_INFO(node_.get_logger(), "Evaluation logging enabled: %s",
              logger_.runDirectory().c_str());
}

void EvaluationReporter::writeFrame(
    const std::size_t raw_scan_points, const sensor::LidarFrame &scan,
    const system::PipelineResult &result,
    const common::FrameProcessingTiming &processing_timing,
    const common::FrameOutputTiming &output_timing) {
  if (!enabled()) {
    return;
  }
  logger_.writeFrame(raw_scan_points, scan, result, processing_timing,
                     output_timing);
}

void EvaluationReporter::writeOdometry(const double timestamp,
                                       const estimator::NominalState &state,
                                       const estimator::ErrorStateVector &covariance_diagonal) {
  if (!enabled()) {
    return;
  }
  logger_.writeOdometry(timestamp, state, covariance_diagonal);
}

void EvaluationReporter::writeRuntime(
    const common::RuntimeSnapshot &snapshot) {
  if (!enabled()) {
    return;
  }
  logger_.writeRuntime(snapshot);
}

void EvaluationReporter::recordRevisitRefinement(
    const std::size_t verifications, const std::size_t accepted, const std::size_t committed) {
  if (enabled()) {logger_.recordRevisitRefinement(verifications, accepted, committed);}
}

void EvaluationReporter::finalize(
    const common::RuntimeSnapshot &snapshot) {
  if (!enabled()) {
    return;
  }
  logger_.finalize(snapshot);
}

} // namespace itera_lio::ros
