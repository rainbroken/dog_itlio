// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__OUTPUT__SCAN_WRITERS_HPP_
#define ITERA_LIO__OUTPUT__SCAN_WRITERS_HPP_

#include "itera_lio/backend/pose_graph.hpp"
#include "itera_lio/common/map_types.hpp"
#include "itera_lio/mapping/color/colorizer.hpp"
#include "itera_lio/output/pcd_result.hpp"
#include <fstream>
#include <array>
#include <limits>
#include <mutex>
#include <vector>

namespace itera_lio::output {

class FullCloudPcdWriter {
public:
  explicit FullCloudPcdWriter(const std::string &output_directory, float voxel_size = 0.0F);
  ~FullCloudPcdWriter();

  FullCloudPcdWriter(const FullCloudPcdWriter &) = delete;
  FullCloudPcdWriter &operator=(const FullCloudPcdWriter &) = delete;

  bool append(const std::vector<common::MapPoint> &points,
              double timestamp = std::numeric_limits<double>::quiet_NaN());
  bool append(const std::vector<common::MapPoint> &points, double timestamp,
              const common::Pose &pose, const common::FixedExtrinsics &extrinsics);
  PcdWriteResult finalize();
  PcdWriteResult snapshot();
  PcdWriteResult optimizedSnapshot(const std::vector<backend::Keyframe> &raw_keyframes,
                                   const std::vector<backend::Keyframe> &optimized_keyframes);
  std::size_t pointsWritten() const { return points_written_; }
  const std::string &error() const { return error_; }

private:
  PcdWriteResult exportPcd();
  float voxel_size_{0.0F};
  // Fixed 64 KiB batch: no per-point stream calls or unbounded staging queue.
  std::array<std::array<float, 4>, 4096> records_;
  std::string output_directory_;
  std::string temporary_path_;
  std::ofstream stream_;
  std::ofstream scan_index_;
  std::size_t points_written_{0U};
  std::string error_;
  bool finalized_{false};
};

// Append-only writer for dense LIVO output. Unlike ColoredGlobalMap this
// writer intentionally keeps repeated observations in the final scan product.
class DenseColoredPcdWriter {
public:
  explicit DenseColoredPcdWriter(const std::string &output_directory);
  ~DenseColoredPcdWriter();

  DenseColoredPcdWriter(const DenseColoredPcdWriter &) = delete;
  DenseColoredPcdWriter &operator=(const DenseColoredPcdWriter &) = delete;

  bool append(const std::vector<mapping::color::ColoredPoint> &points);
  bool append(const std::vector<mapping::color::ColoredPoint> &points, double scan_timestamp);
  bool append(const std::vector<mapping::color::ColoredPoint> &points, double scan_timestamp,
              const common::Pose &pose, const common::FixedExtrinsics &extrinsics);
  PcdWriteResult finalize();
  PcdWriteResult snapshot();
  // Deform each archived scan with the correction of its surrounding graph
  // keyframes. Color and intensity remain attached to the original point.
  PcdWriteResult optimizedSnapshot(const std::vector<backend::Keyframe> &raw_keyframes,
                                   const std::vector<backend::Keyframe> &optimized_keyframes,
                                   std::mutex *archive_mutex = nullptr);
  // Publish only block bounds/transforms and an immutable archive prefix.
  PcdWriteResult optimizedPreview(const std::vector<backend::Keyframe> &raw_keyframes,
                                  const std::vector<backend::Keyframe> &optimized_keyframes,
                                  std::mutex &archive_mutex);
  std::size_t pointsWritten() const { return points_written_; }
  const std::string &error() const { return error_; }

private:
  std::mutex optimized_export_mutex_;
  struct Batch {
    double timestamp{0.0};
    std::size_t point_count{0U};
    common::Pose pose;
    common::FixedExtrinsics extrinsics;
    Eigen::Vector3d minimum;
    Eigen::Vector3d maximum;
  };
  std::vector<Batch> preview_batches_;
  PcdWriteResult exportPcd();
  std::vector<Batch> loadBatches(std::string &error);
  std::string output_directory_;
  std::string temporary_path_;
  std::ofstream stream_;
  std::ofstream scan_index_;
  std::size_t points_written_{0U};
  std::string error_;
  bool finalized_{false};
};

} // namespace itera_lio::output

#endif // ITERA_LIO__OUTPUT__SCAN_WRITERS_HPP_
