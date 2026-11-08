// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__OUTPUT__SCAN_ARCHIVE_HPP_
#define ITERA_LIO__OUTPUT__SCAN_ARCHIVE_HPP_

#include "itera_lio/output/scan_writers.hpp"
#include "itera_lio/output/static_map_filter.hpp"
#include "itera_lio/common/frame_metrics.hpp"
#include "itera_lio/sensor/lidar_frame.hpp"
#include <memory>
#include <mutex>

namespace itera_lio::output {

struct ScanArchiveOptions {
  bool colored{false};
  bool full_deskewed{false};
  std::string directory;
  float pcd_voxel_size{0.0F};
  StaticMapFilterOptions static_filter{};
};

// Owns append-only scan writers and their synchronization. Preview caches and
// ROS publishers do not own or expose the writers. Full scans are LiDAR-local;
// colored input is converted from world to LiDAR coordinates before storage.
class ScanArchive {
public:
  explicit ScanArchive(const ScanArchiveOptions &options);
  bool hasFullScans() const { return bool(full_); }
  bool hasColoredScans() const { return bool(colored_); }
  const std::string &initializationError() const { return initialization_error_; }
  PcdWriteResult appendFull(const sensor::LidarFrame &scan, const common::Pose &pose,
                            const common::FixedExtrinsics &extrinsics,
                            common::FrameOutputTiming *timing = nullptr);
  PcdWriteResult appendColored(const std::vector<mapping::color::ColoredPoint> &world_points,
                               double timestamp, const common::Pose &pose,
                               const common::FixedExtrinsics &extrinsics);
  PcdWriteResult snapshot(const std::vector<backend::Keyframe> &raw,
                          const std::vector<backend::Keyframe> &optimized, bool corrected);
  PcdWriteResult optimizedSnapshot(const std::vector<backend::Keyframe> &raw,
                                   const std::vector<backend::Keyframe> &optimized);
  PcdWriteResult optimizedPreview(const std::vector<backend::Keyframe> &raw,
                                  const std::vector<backend::Keyframe> &optimized);
  PcdWriteResult finalize();

private:
  std::mutex export_mutex_;
  std::mutex append_mutex_;
  // Protected by append_mutex_; capacity follows the largest observed scan.
  std::vector<common::MapPoint> full_points_buffer_;
  std::unique_ptr<FullCloudPcdWriter> full_;
  std::unique_ptr<DenseColoredPcdWriter> colored_;
  std::string initialization_error_;
  ScanArchiveOptions options_;
};

} // namespace itera_lio::output

#endif // ITERA_LIO__OUTPUT__SCAN_ARCHIVE_HPP_
