// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/output/scan_archive.hpp"
#include <chrono>

namespace itera_lio::output {

ScanArchive::ScanArchive(const ScanArchiveOptions &options) : options_(options) {
  options_.static_filter.validate();
  if (options.colored) {
    colored_ = std::make_unique<DenseColoredPcdWriter>(options.directory);
    initialization_error_ = colored_->error();
    if (!initialization_error_.empty()) {
      colored_.reset();
    }
  }
  if (options.full_deskewed) {
    full_ = std::make_unique<FullCloudPcdWriter>(options.directory, options.pcd_voxel_size);
    const auto error = full_->error();
    if (!error.empty()) {
      initialization_error_ += error;
      full_.reset();
    }
  }
}

PcdWriteResult ScanArchive::appendFull(const sensor::LidarFrame &scan, const common::Pose &pose,
                                       const common::FixedExtrinsics &extrinsics,
                                       common::FrameOutputTiming *timing) {
  if (!full_) {
    return {false, {}, "full scan writer unavailable"};
  }
  const auto wait_start = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(append_mutex_);
  const auto conversion_start = std::chrono::steady_clock::now();
  auto &points = full_points_buffer_;
  points.clear();
  points.reserve(scan.points.size());
  for (const auto &point : scan.points) {
    points.push_back({point.position.cast<float>(), point.intensity});
  }
  const auto write_start = std::chrono::steady_clock::now();
  const bool success = full_->append(points, scan.scan_end_time, pose, extrinsics);
  if (timing != nullptr) {
    const auto write_end = std::chrono::steady_clock::now();
    timing->full_archive_conversion_time_ms =
      std::chrono::duration<double, std::milli>(write_start - conversion_start).count();
    timing->full_archive_wait_time_ms =
      std::chrono::duration<double, std::milli>(conversion_start - wait_start).count();
    timing->full_archive_write_time_ms =
      std::chrono::duration<double, std::milli>(write_end - write_start).count();
  }
  return {success, {}, success ? std::string{} : full_->error()};
}

PcdWriteResult
ScanArchive::appendColored(const std::vector<mapping::color::ColoredPoint> &world_points,
                           const double timestamp, const common::Pose &pose,
                           const common::FixedExtrinsics &extrinsics) {
  if (!colored_) {
    return {false, {}, "colored scan writer unavailable"};
  }
  auto local_points = world_points;
  for (auto &point : local_points) {
    const Eigen::Vector3d point_I =
        pose.R_WI.transpose() * (point.position.cast<double>() - pose.p_WI);
    point.position = (extrinsics.R_IL.transpose() * (point_I - extrinsics.p_IL)).cast<float>();
  }
  std::lock_guard<std::mutex> lock(append_mutex_);
  const bool success = colored_->append(local_points, timestamp, pose, extrinsics);
  return {success, {}, success ? std::string{} : colored_->error()};
}

PcdWriteResult ScanArchive::snapshot(const std::vector<backend::Keyframe> &raw,
                                     const std::vector<backend::Keyframe> &optimized,
                                     const bool corrected) {
  std::lock_guard<std::mutex> export_lock(export_mutex_);
  std::lock_guard<std::mutex> append_lock(append_mutex_);
  if (colored_) {
    return corrected ? colored_->optimizedSnapshot(raw, optimized) : colored_->snapshot();
  }
  if (full_) {
    auto result = corrected ? full_->optimizedSnapshot(raw, optimized) : full_->snapshot();
    if (result.success && !corrected && options_.static_filter.enabled) {
      result = exportStaticMap(options_.directory, options_.pcd_voxel_size, options_.static_filter);
    }
    return result;
  }
  return {false, {}, "scan writer unavailable"};
}

PcdWriteResult ScanArchive::optimizedSnapshot(const std::vector<backend::Keyframe> &raw,
                                              const std::vector<backend::Keyframe> &optimized) {
  return snapshot(raw, optimized, true);
}

PcdWriteResult ScanArchive::optimizedPreview(const std::vector<backend::Keyframe> &raw,
                                             const std::vector<backend::Keyframe> &optimized) {
  if (!colored_) {
    return {false, {}, "colored scan writer unavailable"};
  }
  std::lock_guard<std::mutex> export_lock(export_mutex_);
  // The writer captures an immutable prefix under append_mutex_, then releases
  // it before generating the manifest. Append can continue during file output.
  return colored_->optimizedPreview(raw, optimized, append_mutex_);
}

PcdWriteResult ScanArchive::finalize() {
  std::lock_guard<std::mutex> export_lock(export_mutex_);
  std::lock_guard<std::mutex> append_lock(append_mutex_);
  if (colored_) {
    return colored_->finalize();
  }
  if (full_) {
    auto result = full_->finalize();
    if (result.success && options_.static_filter.enabled) {
      result = exportStaticMap(options_.directory, options_.pcd_voxel_size, options_.static_filter);
    }
    return result;
  }
  return {false, {}, "scan writer unavailable"};
}

} // namespace itera_lio::output
