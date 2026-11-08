// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/output/scan_writers.hpp"
#include <algorithm>
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

#include <Eigen/Geometry>

namespace itera_lio::output {
using mapping::color::ColoredPoint;
PcdWriteResult DenseColoredPcdWriter::snapshot() {
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

DenseColoredPcdWriter::DenseColoredPcdWriter(const std::string &output_directory)
    : output_directory_(output_directory) {
  try {
    namespace fs = std::filesystem;
    const fs::path directory = fs::absolute(fs::path(output_directory_));
    fs::create_directories(directory);
    temporary_path_ = (directory / "colored_points.bin").string();
    scan_index_.open(directory / "colored_scans.csv");
    scan_index_ << "timestamp,first_point,point_count,px,py,pz,qx,qy,qz,qw,"
                   "extrinsic_px,extrinsic_py,extrinsic_pz,extrinsic_qx,extrinsic_qy,"
                   "extrinsic_qz,extrinsic_qw\n"
                << std::setprecision(17);
    stream_.open(temporary_path_, std::ios::binary | std::ios::trunc);
    if (!stream_) {
      error_ = "failed to open dense colored temporary file";
    }
  } catch (const std::exception &exception) {
    error_ = exception.what();
  }
}

DenseColoredPcdWriter::~DenseColoredPcdWriter() {
  if (stream_.is_open()) {
    stream_.close();
  }
}

bool DenseColoredPcdWriter::append(const std::vector<mapping::color::ColoredPoint> &points) {
  return append(points, std::numeric_limits<double>::quiet_NaN());
}

bool DenseColoredPcdWriter::append(const std::vector<mapping::color::ColoredPoint> &points,
                                   const double scan_timestamp) {
  return append(points, scan_timestamp, common::Pose{}, common::FixedExtrinsics{});
}

bool DenseColoredPcdWriter::append(const std::vector<mapping::color::ColoredPoint> &points,
                                   const double scan_timestamp, const common::Pose &pose,
                                   const common::FixedExtrinsics &extrinsics) {
  if (!pose.isValid() || !extrinsics.isValid()) {
    error_ = "invalid colored scan pose or LiDAR extrinsics";
    return false;
  }
  if (finalized_ || !error_.empty() || !stream_.is_open()) {
    return false;
  }
  const std::size_t before = points_written_;
  Eigen::Vector3d minimum = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d maximum = -minimum;
  for (const auto &point : points) {
    if (!point.position.allFinite() || !std::isfinite(point.intensity)) {
      continue;
    }
    const float position[4] = {point.position.x(), point.position.y(), point.position.z(),
                               point.intensity};
    minimum = minimum.cwiseMin(point.position.cast<double>());
    maximum = maximum.cwiseMax(point.position.cast<double>());
    const std::uint32_t packed_rgb = (static_cast<std::uint32_t>(point.r) << 16U) |
                                     (static_cast<std::uint32_t>(point.g) << 8U) |
                                     static_cast<std::uint32_t>(point.b);
    float rgb = 0.0F;
    std::memcpy(&rgb, &packed_rgb, sizeof(rgb));
    stream_.write(reinterpret_cast<const char *>(position), sizeof(position));
    stream_.write(reinterpret_cast<const char *>(&rgb), sizeof(rgb));
    if (!stream_) {
      error_ = "failed to append dense colored point cloud";
      return false;
    }
    ++points_written_;
  }
  if (points_written_ > before) {
    preview_batches_.push_back(
        Batch{scan_timestamp, points_written_ - before, pose, extrinsics, minimum, maximum});
    const Eigen::Quaterniond q_WI(pose.R_WI);
    const Eigen::Quaterniond q_IL(extrinsics.R_IL);
    scan_index_ << scan_timestamp << ',' << before << ',' << (points_written_ - before) << ','
                << pose.p_WI.x() << ',' << pose.p_WI.y() << ',' << pose.p_WI.z() << ',' << q_WI.x()
                << ',' << q_WI.y() << ',' << q_WI.z() << ',' << q_WI.w() << ','
                << extrinsics.p_IL.x() << ',' << extrinsics.p_IL.y() << ',' << extrinsics.p_IL.z()
                << ',' << q_IL.x() << ',' << q_IL.y() << ',' << q_IL.z() << ',' << q_IL.w() << '\n';
    if (!scan_index_) {
      error_ = "failed to write colored scan index";
      return false;
    }
  }
  return true;
}

std::vector<DenseColoredPcdWriter::Batch> DenseColoredPcdWriter::loadBatches(std::string &error) {
  scan_index_.flush();
  if (!scan_index_) {
    error = "failed to flush colored scan index";
    return {};
  }
  std::ifstream index(std::filesystem::absolute(std::filesystem::path(output_directory_)) /
                      "colored_scans.csv");
  if (!index) {
    error = "failed to open colored scan index";
    return {};
  }
  std::vector<Batch> batches;
  std::string line;
  std::getline(index, line);
  std::size_t covered = 0U;
  while (std::getline(index, line)) {
    std::istringstream row(line);
    std::vector<std::string> fields;
    std::string field;
    while (std::getline(row, field, ',')) {
      fields.push_back(field);
    }
    if (fields.size() != 17U) {
      error = "invalid colored scan index";
      return {};
    }
    Batch batch;
    batch.timestamp = std::stod(fields[0]);
    const auto first = static_cast<std::size_t>(std::stoull(fields[1]));
    batch.point_count = static_cast<std::size_t>(std::stoull(fields[2]));
    if (first != covered || batch.point_count > points_written_ - covered) {
      error = "colored scan index does not cover the archive";
      return {};
    }
    batch.pose.p_WI =
        Eigen::Vector3d(std::stod(fields[3]), std::stod(fields[4]), std::stod(fields[5]));
    const Eigen::Quaterniond q_WI(std::stod(fields[9]), std::stod(fields[6]), std::stod(fields[7]),
                                  std::stod(fields[8]));
    batch.pose.R_WI = q_WI.toRotationMatrix();
    batch.extrinsics.p_IL =
        Eigen::Vector3d(std::stod(fields[10]), std::stod(fields[11]), std::stod(fields[12]));
    const Eigen::Quaterniond q_IL(std::stod(fields[16]), std::stod(fields[13]),
                                  std::stod(fields[14]), std::stod(fields[15]));
    batch.extrinsics.R_IL = q_IL.toRotationMatrix();
    if (!batch.pose.isValid(1e-6) || !batch.extrinsics.isValid(1e-6)) {
      error = "invalid colored scan pose";
      return {};
    }
    covered += batch.point_count;
    batches.push_back(batch);
  }
  if (index.bad() || covered != points_written_) {
    error = "colored scan index does not cover the archive";
    return {};
  }
  return batches;
}

PcdWriteResult DenseColoredPcdWriter::exportPcd() {
  PcdWriteResult result;
  if (!error_.empty() || points_written_ == 0U) {
    result.error = error_.empty() ? "dense colored point cloud is empty" : error_;
    return result;
  }
  stream_.flush();
  scan_index_.flush();
  if (!stream_ || !scan_index_) {
    result.error = "failed to flush colored scan archive";
    return result;
  }
  std::string index_error;
  const auto batches = loadBatches(index_error);
  if (!index_error.empty()) {
    result.error = index_error;
    return result;
  }
  const auto path =
      std::filesystem::absolute(std::filesystem::path(output_directory_)) / "cloud_color.pcd";
  const auto temporary_output = path.string() + ".tmp";
  std::ifstream input(temporary_path_, std::ios::binary);
  std::ofstream output(temporary_output, std::ios::binary | std::ios::trunc);
  if (!input || !output) {
    result.error = "failed to open dense colored PCD conversion files";
    return result;
  }
  output << "# .PCD v0.7 - Point Cloud Data file\nVERSION 0.7\n"
         << "FIELDS x y z intensity rgb\nSIZE 4 4 4 4 4\n"
         << "TYPE F F F F F\nCOUNT 1 1 1 1 1\nWIDTH " << points_written_
         << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << points_written_ << "\nDATA binary\n";
  std::size_t written = 0U;
  for (const auto &batch : batches) {
    for (std::size_t i = 0; i < batch.point_count; ++i) {
      float record[5];
      input.read(reinterpret_cast<char *>(record), sizeof(record));
      if (!input) {
        result.error = "colored archive ended before its batch index";
        break;
      }
      const Eigen::Vector3d point_L(record[0], record[1], record[2]);
      const Eigen::Vector3d point_W =
          batch.pose.R_WI * (batch.extrinsics.R_IL * point_L + batch.extrinsics.p_IL) +
          batch.pose.p_WI;
      record[0] = static_cast<float>(point_W.x());
      record[1] = static_cast<float>(point_W.y());
      record[2] = static_cast<float>(point_W.z());
      output.write(reinterpret_cast<const char *>(record), sizeof(record));
      ++written;
    }
    if (!result.error.empty()) {
      break;
    }
  }
  output.flush();
  output.close();
  if (written != points_written_ && result.error.empty()) {
    result.error = "colored batch index does not cover the archive";
  }
  if (!result.error.empty() || !output) {
    if (result.error.empty()) {
      result.error = "failed to write dense colored PCD";
    }
    std::filesystem::remove(temporary_output);
    return result;
  }
  std::filesystem::rename(temporary_output, path);
  result.success = true;
  result.output_path = path.string();
  result.points_written = written;
  return result;
}

PcdWriteResult DenseColoredPcdWriter::finalize() {
  PcdWriteResult result;
  const auto total_start = std::chrono::steady_clock::now();
  if (finalized_) {
    result.error = "dense colored PCD writer already finalized";
    return result;
  }
  if (!error_.empty()) {
    result.error = error_;
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
