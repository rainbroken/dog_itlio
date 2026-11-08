// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "archive_pose_correction.hpp"
#include "itera_lio/output/scan_writers.hpp"
#include "itera_lio/common/voxel.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>
#include <unordered_set>
#include <stdexcept>

#include <Eigen/Geometry>

namespace itera_lio::output {
namespace {
// Filter only after the scan (and optional graph correction) is in map coordinates.
// Keep original samples and intensity; do not average across obstacle boundaries.
class VoxelPcdOutput {
public:
  VoxelPcdOutput(std::ofstream &stream, float voxel_size)
      : stream_(stream), voxel_size_(voxel_size) {
    stream_ << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
            << "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n"
            << "COUNT 1 1 1 1\nWIDTH ";
    width_position_ = stream_.tellp();
    stream_ << std::setw(20) << 0 << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS ";
    points_position_ = stream_.tellp();
    stream_ << std::setw(20) << 0 << "\nDATA binary\n";
  }

  void write(float *records, std::size_t count) {
    std::size_t kept = 0;
    for (std::size_t i = 0; i < count; ++i) {
      const float *record = records + 4 * i;
      if (voxel_size_ > 0.0F) {
        common::VoxelKey key;
        const Eigen::Vector3f point(record[0], record[1], record[2]);
        if (!common::makeVoxelKey(point, voxel_size_, key)) {
          throw std::runtime_error("invalid map point during PCD voxel sampling");
        }
        if (!occupied_.insert(key).second) {continue;}
      }
      if (kept != i) {std::memmove(records + 4 * kept, record, 4 * sizeof(float));}
      ++kept;
    }
    stream_.write(reinterpret_cast<const char *>(records),
                  static_cast<std::streamsize>(kept * 4 * sizeof(float)));
    count_ += kept;
  }

  std::size_t finish() {
    const auto end = stream_.tellp();
    stream_.seekp(width_position_);
    stream_ << std::setw(20) << count_;
    stream_.seekp(points_position_);
    stream_ << std::setw(20) << count_;
    stream_.seekp(end);
    return count_;
  }

private:
  std::ofstream &stream_;
  float voxel_size_;
  std::streampos width_position_, points_position_;
  std::size_t count_{0};
  std::unordered_set<common::VoxelKey, common::VoxelKeyHash> occupied_;
};
} // namespace

PcdWriteResult FullCloudPcdWriter::snapshot() {
  if (!error_.empty() || finalized_) {
    return {false, {}, error_.empty() ? "writer finalized" : error_};
  }
  try {
    return exportPcd();
  } catch (const std::exception &exception) {
    PcdWriteResult result;
    result.error = exception.what();
    return result;
  }
}

PcdWriteResult
FullCloudPcdWriter::optimizedSnapshot(const std::vector<backend::Keyframe> &raw_keyframes,
                                      const std::vector<backend::Keyframe> &optimized_keyframes) {
  PcdWriteResult result;
  if (finalized_ || !error_.empty() || raw_keyframes.empty() ||
      raw_keyframes.size() != optimized_keyframes.size()) {
    result.error = "optimized full scan archive is unavailable";
    return result;
  }
  for (std::size_t i = 0; i < raw_keyframes.size(); ++i) {
    if (!raw_keyframes[i].isValid() || !optimized_keyframes[i].isValid() ||
        raw_keyframes[i].id != optimized_keyframes[i].id ||
        raw_keyframes[i].timestamp != optimized_keyframes[i].timestamp ||
        (i > 0 && raw_keyframes[i].timestamp <= raw_keyframes[i - 1].timestamp)) {
      result.error = "invalid optimized keyframe correspondence";
      return result;
    }
  }
  stream_.flush();
  scan_index_.flush();
  if (!stream_ || !scan_index_) {
    result.error = "failed to flush full scan archive";
    return result;
  }
  try {
    const auto directory = std::filesystem::absolute(std::filesystem::path(output_directory_));
    const auto path = directory / "cloud_optimized.pcd";
    const auto temporary_output = path.string() + ".tmp";
    std::ifstream input(temporary_path_, std::ios::binary);
    std::ifstream index(directory / "full_deskewed_scans.csv");
    std::ofstream output(temporary_output, std::ios::binary | std::ios::trunc);
    if (!input || !index || !output) {
      result.error = "failed to open optimized full scan files";
      return result;
    }
    VoxelPcdOutput pcd(output, voxel_size_);
    std::string line;
    std::getline(index, line);
    std::size_t written = 0U;
    while (std::getline(index, line)) {
      std::istringstream row(line);
      std::vector<std::string> fields;
      std::string field;
      while (std::getline(row, field, ',')) {
        fields.push_back(field);
      }
      if (fields.size() != 17U) {
        result.error = "invalid full scan pose index";
        break;
      }
      const double timestamp = std::stod(fields[0]);
      const auto first = static_cast<std::size_t>(std::stoull(fields[1]));
      const auto count = static_cast<std::size_t>(std::stoull(fields[2]));
      if (!std::isfinite(timestamp) || first != written || count > points_written_ - written) {
        result.error = "full scan pose index does not cover the archive";
        break;
      }
      const common::Pose correction =
          detail::archiveCorrection(timestamp, raw_keyframes, optimized_keyframes);
      const Eigen::Vector3d p_WI(std::stod(fields[3]), std::stod(fields[4]), std::stod(fields[5]));
      const Eigen::Quaterniond q_WI(std::stod(fields[9]), std::stod(fields[6]),
                                    std::stod(fields[7]), std::stod(fields[8]));
      const Eigen::Vector3d p_IL(std::stod(fields[10]), std::stod(fields[11]),
                                 std::stod(fields[12]));
      const Eigen::Quaterniond q_IL(std::stod(fields[16]), std::stod(fields[13]),
                                    std::stod(fields[14]), std::stod(fields[15]));
      // Read/write bounded blocks; preserve each point's transform and order.
      std::array<float, 4096U * 4U> records;
      for (std::size_t offset = 0U; offset < count;) {
        const std::size_t block = std::min<std::size_t>(4096U, count - offset);
        const auto bytes = static_cast<std::streamsize>(block * 4U * sizeof(float));
        input.read(reinterpret_cast<char *>(records.data()), bytes);
        if (!input) {
          result.error = "full scan archive is truncated";
          break;
        }
        for (std::size_t i = 0U; i < block; ++i) {
          float *record = records.data() + 4U * i;
          const Eigen::Vector3d raw_point_W =
              q_WI * (q_IL * Eigen::Vector3d(record[0], record[1], record[2]) + p_IL) + p_WI;
          const Eigen::Vector3d point_W = correction.R_WI * raw_point_W + correction.p_WI;
          record[0] = static_cast<float>(point_W.x());
          record[1] = static_cast<float>(point_W.y());
          record[2] = static_cast<float>(point_W.z());
        }
        pcd.write(records.data(), block);
        written += block;
        offset += block;
      }
      if (!result.error.empty()) {
        break;
      }
    }
    const auto exported_points = pcd.finish();
    output.flush();
    output.close();
    if (written != points_written_ && result.error.empty()) {
      result.error = "full scan pose index does not cover the archive";
    }
    if (!result.error.empty() || !output) {
      if (result.error.empty()) {
        result.error = "failed to write optimized full scan PCD";
      }
      std::filesystem::remove(temporary_output);
      return result;
    }
    std::filesystem::rename(temporary_output, path);
    result.success = true;
    result.output_path = path.string();
    result.points_written = exported_points;
  } catch (const std::exception &exception) {
    result.error = exception.what();
  }
  return result;
}

FullCloudPcdWriter::FullCloudPcdWriter(const std::string &output_directory, float voxel_size)
    : voxel_size_(voxel_size), output_directory_(output_directory) {
  if (!std::isfinite(voxel_size_) || voxel_size_ < 0.0F) {
    throw std::invalid_argument("PCD voxel size must be finite and non-negative");
  }
  try {
    namespace fs = std::filesystem;
    const fs::path directory = fs::absolute(fs::path(output_directory_));
    fs::create_directories(directory);
    temporary_path_ = (directory / "full_deskewed_points.bin").string();
    scan_index_.open(directory / "full_deskewed_scans.csv");
    scan_index_ << "timestamp,first_point,point_count,px,py,pz,qx,qy,qz,qw,"
                   "extrinsic_px,extrinsic_py,extrinsic_pz,extrinsic_qx,extrinsic_qy,"
                   "extrinsic_qz,extrinsic_qw\n"
                << std::setprecision(17);
    stream_.open(temporary_path_, std::ios::binary | std::ios::trunc);
    if (!stream_) {
      error_ = "failed to open full deskewed temporary file";
    }
  } catch (const std::exception &exception) {
    error_ = exception.what();
  }
}

FullCloudPcdWriter::~FullCloudPcdWriter() {
  if (stream_.is_open()) {
    stream_.close();
  }
}

bool FullCloudPcdWriter::append(const std::vector<common::MapPoint> &points, double timestamp) {
  return append(points, timestamp, common::Pose{}, common::FixedExtrinsics{});
}

bool FullCloudPcdWriter::append(const std::vector<common::MapPoint> &points, const double timestamp,
                                const common::Pose &pose,
                                const common::FixedExtrinsics &extrinsics) {
  if (!pose.isValid() || !extrinsics.isValid()) {
    error_ = "invalid scan pose or LiDAR extrinsics";
    return false;
  }
  const auto before = points_written_;
  if (finalized_ || !error_.empty() || !stream_.is_open()) {
    return false;
  }
  static_assert(sizeof(records_[0]) == 4U * sizeof(float));
  std::size_t buffered = 0U;
  const auto flush_batch = [&]() {
    if (buffered == 0U) {return true;}
    stream_.write(reinterpret_cast<const char *>(records_.data()),
                  static_cast<std::streamsize>(buffered * sizeof(records_[0])));
    if (!stream_) {
      error_ = "failed to append full deskewed point cloud";
      return false;
    }
    points_written_ += buffered;
    buffered = 0U;
    return true;
  };
  for (const auto &point : points) {
    if (!point.position.allFinite() || !std::isfinite(point.intensity)) {
      continue;
    }
    records_[buffered++] = {point.position.x(), point.position.y(), point.position.z(),
                           point.intensity};
    if (buffered == records_.size() && !flush_batch()) {return false;}
  }
  if (!flush_batch()) {return false;}
  const Eigen::Quaterniond q_WI(pose.R_WI);
  const Eigen::Quaterniond q_IL(extrinsics.R_IL);
  scan_index_ << timestamp << ',' << before << ',' << (points_written_ - before) << ','
              << pose.p_WI.x() << ',' << pose.p_WI.y() << ',' << pose.p_WI.z() << ',' << q_WI.x()
              << ',' << q_WI.y() << ',' << q_WI.z() << ',' << q_WI.w() << ',' << extrinsics.p_IL.x()
              << ',' << extrinsics.p_IL.y() << ',' << extrinsics.p_IL.z() << ',' << q_IL.x() << ','
              << q_IL.y() << ',' << q_IL.z() << ',' << q_IL.w() << '\n';
  if (!scan_index_) {
    error_ = "failed to write scan pose index";
    return false;
  }
  return true;
}

PcdWriteResult FullCloudPcdWriter::exportPcd() {
  PcdWriteResult result;
  if (!error_.empty() || points_written_ == 0U) {
    result.error = error_.empty() ? "full deskewed point cloud is empty" : error_;
    return result;
  }
  stream_.flush();
  scan_index_.flush();
  if (!stream_ || !scan_index_) {
    result.error = "failed to flush full deskewed scan archive";
    return result;
  }
  const auto path =
      std::filesystem::absolute(std::filesystem::path(output_directory_)) / "cloud.pcd";
  const auto temporary_output = path.string() + ".tmp";
  std::ifstream input(temporary_path_, std::ios::binary);
  std::ifstream index(std::filesystem::absolute(std::filesystem::path(output_directory_)) /
                      "full_deskewed_scans.csv");
  std::ofstream output(temporary_output, std::ios::binary | std::ios::trunc);
  if (!input || !index || !output) {
    result.error = "failed to open full deskewed PCD conversion files";
    return result;
  }
  VoxelPcdOutput pcd(output, voxel_size_);
  std::size_t written = 0U;
  std::string line;
  std::getline(index, line); // header
  while (std::getline(index, line)) {
    std::istringstream row(line);
    std::vector<std::string> fields;
    std::string field;
    while (std::getline(row, field, ',')) {
      fields.push_back(field);
    }
    if (fields.size() != 17U) {
      result.error = "invalid full deskewed scan pose index";
      break;
    }
    const auto first = static_cast<std::size_t>(std::stoull(fields[1]));
    const auto count = static_cast<std::size_t>(std::stoull(fields[2]));
    if (first != written || count > points_written_ - written) {
      result.error = "full deskewed scan index does not cover the archive";
      break;
    }
    const Eigen::Vector3d p_WI(std::stod(fields[3]), std::stod(fields[4]), std::stod(fields[5]));
    const Eigen::Quaterniond q_WI(std::stod(fields[9]), std::stod(fields[6]), std::stod(fields[7]),
                                  std::stod(fields[8]));
    const Eigen::Vector3d p_IL(std::stod(fields[10]), std::stod(fields[11]), std::stod(fields[12]));
    const Eigen::Quaterniond q_IL(std::stod(fields[16]), std::stod(fields[13]),
                                  std::stod(fields[14]), std::stod(fields[15]));
    if (!p_WI.allFinite() || !p_IL.allFinite() || !q_WI.coeffs().allFinite() ||
        !q_IL.coeffs().allFinite()) {
      result.error = "nonfinite full deskewed scan pose";
      break;
    }
    // Read/write bounded contiguous blocks, keeping each point's exact
    // original transform expression and its record order.
    std::array<float, 4096U * 4U> records;
    for (std::size_t offset = 0U; offset < count;) {
      const std::size_t block = std::min<std::size_t>(4096U, count - offset);
      const auto bytes = static_cast<std::streamsize>(block * 4U * sizeof(float));
      input.read(reinterpret_cast<char *>(records.data()), bytes);
      if (!input) {
        result.error = "full deskewed archive ended before its scan index";
        break;
      }
      for (std::size_t i = 0U; i < block; ++i) {
        float *record = records.data() + 4U * i;
        const Eigen::Vector3d point_L(record[0], record[1], record[2]);
        const Eigen::Vector3d point_W = q_WI * (q_IL * point_L + p_IL) + p_WI;
        record[0] = static_cast<float>(point_W.x());
        record[1] = static_cast<float>(point_W.y());
        record[2] = static_cast<float>(point_W.z());
      }
      pcd.write(records.data(), block);
      written += block;
      offset += block;
    }
    if (!result.error.empty()) {
      break;
    }
  }
  if (index.bad()) {
    result.error = "failed to read full deskewed scan index";
  }
  const auto exported_points = pcd.finish();
  output.flush();
  output.close();
  if (!result.error.empty() || !output || written != points_written_) {
    if (result.error.empty()) {
      result.error = "failed to write full deskewed PCD";
    }
    std::filesystem::remove(temporary_output);
    return result;
  }
  std::filesystem::rename(temporary_output, path);
  result.success = true;
  result.output_path = path.string();
  result.points_written = exported_points;
  return result;
}

PcdWriteResult FullCloudPcdWriter::finalize() {
  PcdWriteResult result;
  const auto total_start = std::chrono::steady_clock::now();
  if (finalized_) {
    result.error = "full deskewed PCD writer already finalized";
    return result;
  }
  try {
    result = exportPcd();
    if (result.success) {
      stream_.close();
      scan_index_.close();
      finalized_ = true;
    }
  } catch (const std::exception &exception) {
    result.error = exception.what();
  }
  result.total_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - total_start)
          .count();
  return result;
}

} // namespace itera_lio::output
