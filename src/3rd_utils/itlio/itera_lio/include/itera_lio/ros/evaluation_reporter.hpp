// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ITERA_LIO__ROS__EVALUATION_REPORTER_HPP_
#define ITERA_LIO__ROS__EVALUATION_REPORTER_HPP_

#include <string>

#include "itera_lio/output/evaluation_logger.hpp"
#include "rclcpp/rclcpp.hpp"

namespace itera_lio::ros {

class EvaluationReporter {
public:
  EvaluationReporter(rclcpp::Node &node, bool enabled,
                     const std::string &output_directory,
                     const output::EvaluationMetadata &metadata,
                     const output::EvaluationOutputOptions &options);

  bool enabled() const { return enabled_ && logger_.isOpen(); }
  bool runtimeTickEnabled() const {
    return enabled() && (options_.runtime || options_.events);
  }
  double runtimeFrequencyHz() const { return options_.runtime_frequency_hz; }
  void writeFrame(std::size_t raw_scan_points, const sensor::LidarFrame &scan,
                  const system::PipelineResult &result,
                  const common::FrameProcessingTiming &processing_timing,
                  const common::FrameOutputTiming &output_timing);
  void writeOdometry(double timestamp, const estimator::NominalState &state,
                     const estimator::ErrorStateVector &covariance_diagonal);
  void writeRuntime(const common::RuntimeSnapshot &snapshot);
  void recordRevisitRefinement(std::size_t verifications, std::size_t accepted, std::size_t committed);
  void finalize(const common::RuntimeSnapshot &snapshot);

private:
  rclcpp::Node &node_;
  bool enabled_{false};
  output::EvaluationOutputOptions options_;
  output::EvaluationLogger logger_;
};

} // namespace itera_lio::ros

#endif // ITERA_LIO__ROS__EVALUATION_REPORTER_HPP_
