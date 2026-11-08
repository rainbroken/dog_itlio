// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0 (the "License");

#include "archive_pose_correction.hpp"
#include "itera_lio/output/scan_writers.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace itera_lio::output
{

PcdWriteResult
DenseColoredPcdWriter::optimizedPreview(const std::vector<backend::Keyframe> &raw_keyframes,
                                        const std::vector<backend::Keyframe> &optimized_keyframes,
                                        std::mutex &archive_mutex) {
  PcdWriteResult result;
  if (raw_keyframes.empty() || raw_keyframes.size() != optimized_keyframes.size()) {
    result.error = "optimized preview keyframes unavailable";
    return result;
  }
  for (std::size_t i = 0; i < raw_keyframes.size(); ++i) {
    if (!raw_keyframes[i].isValid() || !optimized_keyframes[i].isValid() ||
        raw_keyframes[i].id != optimized_keyframes[i].id ||
        raw_keyframes[i].timestamp != optimized_keyframes[i].timestamp ||
        (i && raw_keyframes[i].timestamp <= raw_keyframes[i - 1].timestamp)) {
      result.error = "invalid optimized preview correspondence";
      return result;
    }
  }
  std::vector<Batch> batches;
  {
    std::lock_guard<std::mutex> lock(archive_mutex);
    stream_.flush();
    if (finalized_ || !error_.empty() || !stream_ || preview_batches_.empty()) {
      result.error = "colored archive prefix unavailable";
      return result;
    }
    batches = preview_batches_;
    result.points_written = points_written_;
  }
  // The append-only prefix is now durable to readers. No point payload copy.
  const auto path = std::filesystem::absolute(std::filesystem::path(output_directory_)) /
                    "optimized_preview.json";
  const auto temporary = path.string() + ".tmp";
  try {
    std::ofstream file(temporary, std::ios::trunc);
    const auto quote = [](const std::string &value) {
      std::string escaped;
      for (const char c : value) {
        if (c == '\\' || c == '"')
          escaped += '\\';
        if (c == '\n') {
          escaped += "\\n";
        } else {
          escaped += c;
        }
      }
      return escaped;
    };
    file << std::setprecision(17) << "{\"format\":\"scanforge-blocks-v1\",\"archive\":\""
         << quote(temporary_path_) << "\",\"points\":" << result.points_written << ",\"blocks\":[";
    std::size_t first = 0;
    for (const auto &batch : batches) {
      if (!std::isfinite(batch.timestamp)) {
        throw std::runtime_error("invalid scan timestamp");
      }
      const auto correction =
          detail::archiveCorrection(batch.timestamp, raw_keyframes, optimized_keyframes);
      const Eigen::Matrix3d rotation = correction.R_WI * batch.pose.R_WI * batch.extrinsics.R_IL;
      const Eigen::Vector3d translation =
          correction.R_WI * (batch.pose.R_WI * batch.extrinsics.p_IL + batch.pose.p_WI) +
          correction.p_WI;
      if (first)
        file << ',';
      file << "{\"first\":" << first << ",\"count\":" << batch.point_count << ",\"transform\":[";
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
          if (row || col)
            file << ',';
          file << rotation(row, col);
        }
        file << ',' << translation[row];
      }
      file << "],\"bounds\":[";
      for (int i = 0; i < 3; ++i) {
        if (i)
          file << ',';
        file << batch.minimum[i];
      }
      for (int i = 0; i < 3; ++i)
        file << ',' << batch.maximum[i];
      file << "]}";
      first += batch.point_count;
    }
    file << "]}";
    file.flush();
    if (!file || first != result.points_written) {
      throw std::runtime_error("incomplete preview manifest");
    }
    file.close();
    std::filesystem::rename(temporary, path);
    result.output_path = path.string();
    result.success = true;
  } catch (const std::exception &exception) {
    result.error = exception.what();
    std::filesystem::remove(temporary);
  }
  return result;
}

PcdWriteResult
DenseColoredPcdWriter::optimizedSnapshot(const std::vector<backend::Keyframe> &raw_keyframes,
                                         const std::vector<backend::Keyframe> &optimized_keyframes,
                                         std::mutex *archive_mutex) {
  std::unique_lock<std::mutex> snapshot_lock;
  if (archive_mutex) {
    snapshot_lock = std::unique_lock<std::mutex>(*archive_mutex);
  }
  PcdWriteResult result;
  std::string index_error;
  const auto batches = loadBatches(index_error);
  if (finalized_ || !error_.empty() || !index_error.empty() || raw_keyframes.empty() ||
      raw_keyframes.size() != optimized_keyframes.size() || batches.empty()) {
    result.error = "optimized colored archive is unavailable";
    return result;
  }
  for (std::size_t index = 0; index < raw_keyframes.size(); ++index) {
    if (!raw_keyframes[index].isValid() || !optimized_keyframes[index].isValid() ||
        raw_keyframes[index].id != optimized_keyframes[index].id ||
        raw_keyframes[index].timestamp != optimized_keyframes[index].timestamp ||
        (index > 0 && raw_keyframes[index].timestamp <= raw_keyframes[index - 1].timestamp)) {
      result.error = "invalid optimized keyframe correspondence";
      return result;
    }
  }
  stream_.flush();
  if (!stream_) {
    result.error = "failed to flush colored archive";
    return result;
  }
  const std::size_t snapshot_count = points_written_;
  std::lock_guard<std::mutex> export_lock(optimized_export_mutex_);
  if (snapshot_lock.owns_lock()) {
    snapshot_lock.unlock();
  }
  try {
    namespace fs = std::filesystem;
    const fs::path output_path =
        fs::absolute(fs::path(output_directory_)) / "cloud_color_optimized.pcd";
    const fs::path temporary_output = output_path.string() + ".tmp";
    std::ifstream input(temporary_path_, std::ios::binary);
    std::ofstream output(temporary_output, std::ios::binary | std::ios::trunc);
    if (!input || !output) {
      result.error = "failed to open optimized colored PCD files";
      return result;
    }
    output << "# .PCD v0.7 - Point Cloud Data file\nVERSION 0.7\n"
           << "FIELDS x y z intensity rgb\nSIZE 4 4 4 4 4\n"
           << "TYPE F F F F F\nCOUNT 1 1 1 1 1\n"
           << "WIDTH " << snapshot_count << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
           << "POINTS " << snapshot_count << "\nDATA binary\n";
    std::size_t written = 0U;
    for (const auto &batch : batches) {
      if (!std::isfinite(batch.timestamp)) {
        result.error = "colored archive lacks scan timestamps";
        break;
      }
      const auto correction =
          detail::archiveCorrection(batch.timestamp, raw_keyframes, optimized_keyframes);
      for (std::size_t index = 0; index < batch.point_count; ++index) {
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
        const Eigen::Vector3d point = correction.R_WI * point_W + correction.p_WI;
        record[0] = static_cast<float>(point.x());
        record[1] = static_cast<float>(point.y());
        record[2] = static_cast<float>(point.z());
        output.write(reinterpret_cast<const char *>(record), sizeof(record));
        if (!output) {
          result.error = "failed to write optimized colored point";
          break;
        }
        ++written;
      }
      if (!result.error.empty()) {
        break;
      }
    }
    output.flush();
    output.close();
    if (result.error.empty() && written != snapshot_count) {
      result.error = "colored batch index does not cover the archive";
    }
    if (!result.error.empty() || !output) {
      fs::remove(temporary_output);
      return result;
    }
    fs::rename(temporary_output, output_path);
    result.success = true;
    result.output_path = output_path.string();
    result.points_written = written;
  } catch (const std::exception &exception) {
    result.error = exception.what();
  }
  return result;
}

}  // namespace itera_lio::output
