// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__OUTPUT__EVALUATION_FORMAT_HPP_
#define ITERA_LIO__OUTPUT__EVALUATION_FORMAT_HPP_

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <string>

#include "itera_lio/estimator/state.hpp"

namespace itera_lio::common
{
struct RuntimeSnapshot;
struct FrameOutputTiming;
struct FrameProcessingTiming;
}
namespace itera_lio::sensor {struct LidarFrame;}
namespace itera_lio::estimator {struct NominalState;}
namespace itera_lio::system {struct PipelineResult;}
namespace itera_lio::loop {struct LoopVerificationResult;}

// Package-private serialization. The caller owns streams, locking and event policy.
namespace itera_lio::output::detail
{
std::string jsonEscape(const std::string & value);
std::string csvEscape(const std::string & value);
void writeFrameHeader(std::ostream & stream, bool detailed);
void writeOdometryHeader(std::ostream & stream);
void writeRuntimeHeader(std::ostream & stream, bool detailed);
void writeLoopVerificationHeader(std::ostream & stream);
void writeFrameRow(
  std::ostream & stream, std::uint64_t frame_index, std::size_t raw_scan_points,
  const sensor::LidarFrame & scan, const system::PipelineResult & result,
  const common::FrameProcessingTiming & processing_timing,
  const common::FrameOutputTiming & output_timing, bool detailed);
void writeRuntimeRow(
  std::ostream & stream, const common::RuntimeSnapshot & snapshot,
  double lidar_rate, double imu_rate, bool detailed);
void writeOdometryRow(
  std::ostream & stream, double timestamp, const estimator::NominalState & state,
  const estimator::ErrorStateVector & covariance_diagonal);
void writeLoopVerificationRow(
  std::ostream & stream, double wall_timestamp, double sensor_timestamp,
  std::uint64_t frame_index, const loop::LoopVerificationResult & verification);
}  // namespace itera_lio::output::detail

#endif  // ITERA_LIO__OUTPUT__EVALUATION_FORMAT_HPP_
