// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/output/map_pcd.hpp"
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

#include "itera_lio/mapping/color/colored_global_map.hpp"

namespace itera_lio::output {
using mapping::color::ColoredGlobalMap;
using mapping::color::ColoredPoint;
PcdWriteResult writeMapPcd(const frontend::GlobalMap &map, const std::string &output_directory) {
  PcdWriteResult result;
  const auto total_start = std::chrono::steady_clock::now();
  if (map.size() == 0U) {
    result.error = "global map is empty";
    return result;
  }
  try {
    namespace fs = std::filesystem;
    const fs::path directory = fs::absolute(fs::path(output_directory));
    fs::create_directories(directory);
    const fs::path output_path = directory / "cloud.pcd";

    const fs::path temporary_path = output_path.string() + ".tmp";
    std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
    if (!stream) {
      result.error = "failed to open temporary PCD file";
      return result;
    }
    const auto conversion_start = std::chrono::steady_clock::now();
    stream << "# .PCD v0.7 - Point Cloud Data file format\n"
           << "VERSION 0.7\n"
           << "FIELDS x y z intensity\n"
           << "SIZE 4 4 4 4\n"
           << "TYPE F F F F\n"
           << "COUNT 1 1 1 1\n"
           << "WIDTH " << map.size() << "\n"
           << "HEIGHT 1\n"
           << "VIEWPOINT 0 0 0 1 0 0 0\n"
           << "POINTS " << map.size() << "\n"
           << "DATA binary\n";
    if (!stream) {
      result.error = "failed to write PCD header";
      stream.close();
      fs::remove(temporary_path);
      return result;
    }
    const auto write_start = std::chrono::steady_clock::now();
    std::size_t written = 0U;
    const bool visited = map.forEachPoint([&](const common::MapPoint &point) {
      if (!point.position.allFinite() || !std::isfinite(point.intensity)) {
        return;
      }
      const float record[4] = {point.position.x(), point.position.y(), point.position.z(),
                               point.intensity};
      stream.write(reinterpret_cast<const char *>(record), sizeof(record));
      if (stream) {
        ++written;
      }
    });
    stream.flush();
    stream.close();
    result.conversion_time_ms =
        std::chrono::duration<double, std::milli>(write_start - conversion_start).count();
    result.file_write_time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - write_start)
            .count();
    if (!visited || written != map.size()) {
      result.error = "global map changed or contained invalid points while writing";
      fs::remove(temporary_path);
      return result;
    }
    fs::rename(temporary_path, output_path);
    result.success = true;
    result.output_path = output_path.string();
    result.points_written = written;
  } catch (const std::exception &exception) {
    result.error = exception.what();
  }
  result.total_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - total_start)
          .count();
  return result;
}

PcdWriteResult writeMapPcd(const std::vector<common::MapPoint> &points,
                           const std::string &output_directory,
                           const std::string &filename_suffix) {
  PcdWriteResult result;
  const auto total_start = std::chrono::steady_clock::now();
  std::vector<const common::MapPoint *> valid_points;
  valid_points.reserve(points.size());
  for (const auto &point : points) {
    if (point.position.allFinite() && std::isfinite(point.intensity)) {
      valid_points.push_back(&point);
    }
  }
  if (valid_points.empty() || filename_suffix.empty()) {
    result.error = "map point vector or filename suffix is empty";
    return result;
  }
  try {
    namespace fs = std::filesystem;
    const fs::path directory = fs::absolute(fs::path(output_directory));
    fs::create_directories(directory);
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::system_clock::to_time_t(now);
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() %
        1000;
    std::tm local_time{};
    localtime_r(&seconds, &local_time);
    std::ostringstream filename;
    filename << std::put_time(&local_time, "%Y%m%d_%H%M%S") << '_' << std::setfill('0')
             << std::setw(3) << milliseconds << '_' << filename_suffix << ".pcd";
    const fs::path output_path = directory / filename.str();
    const fs::path temporary_path = output_path.string() + ".tmp";
    std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
    if (!stream) {
      result.error = "failed to open temporary map PCD file";
      return result;
    }
    stream << "# .PCD v0.7 - Point Cloud Data file format\n"
           << "VERSION 0.7\nFIELDS x y z intensity\n"
           << "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
           << "WIDTH " << valid_points.size() << "\nHEIGHT 1\n"
           << "VIEWPOINT 0 0 0 1 0 0 0\n"
           << "POINTS " << valid_points.size() << "\nDATA binary\n";
    const auto write_start = std::chrono::steady_clock::now();
    for (const auto *point : valid_points) {
      const float record[4] = {point->position.x(), point->position.y(), point->position.z(),
                               point->intensity};
      stream.write(reinterpret_cast<const char *>(record), sizeof(record));
    }
    stream.flush();
    stream.close();
    if (!stream) {
      result.error = "failed to write map PCD";
      fs::remove(temporary_path);
      return result;
    }
    fs::rename(temporary_path, output_path);
    result.success = true;
    result.output_path = output_path.string();
    result.points_written = valid_points.size();
    result.file_write_time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - write_start)
            .count();
  } catch (const std::exception &exception) {
    result.error = exception.what();
  }
  result.total_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - total_start)
          .count();
  return result;
}

PcdWriteResult writeColoredMapPcd(const std::vector<ColoredPoint> &points,
                                  const std::string &output_directory) {
  PcdWriteResult result;
  const auto total_start = std::chrono::steady_clock::now();
  std::vector<const ColoredPoint *> valid_points;
  valid_points.reserve(points.size());
  for (const auto &point : points) {
    if (point.position.allFinite() && std::isfinite(point.intensity)) {
      valid_points.push_back(&point);
    }
  }
  if (valid_points.empty()) {
    result.error = "colored global map is empty";
    return result;
  }
  try {
    namespace fs = std::filesystem;
    const fs::path directory = fs::absolute(fs::path(output_directory));
    fs::create_directories(directory);
    const fs::path output_path = directory / "cloud_color.pcd";
    const fs::path temporary_path = output_path.string() + ".tmp";
    std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
    if (!stream) {
      result.error = "failed to open temporary colored PCD file";
      return result;
    }
    stream << "# .PCD v0.7 - Point Cloud Data file\n"
           << "VERSION 0.7\n"
           << "FIELDS x y z intensity rgb\n"
           << "SIZE 4 4 4 4 4\n"
           << "TYPE F F F F F\n"
           << "COUNT 1 1 1 1 1\n"
           << "WIDTH " << valid_points.size() << "\n"
           << "HEIGHT 1\n"
           << "VIEWPOINT 0 0 0 1 0 0 0\n"
           << "POINTS " << valid_points.size() << "\n"
           << "DATA binary\n";
    if (!stream) {
      result.error = "failed to write colored PCD header";
      stream.close();
      fs::remove(temporary_path);
      return result;
    }
    const auto write_start = std::chrono::steady_clock::now();
    for (const auto *point : valid_points) {
      const float position[4] = {point->position.x(), point->position.y(), point->position.z(),
                                 point->intensity};
      const std::uint32_t packed_rgb = (static_cast<std::uint32_t>(point->r) << 16U) |
                                       (static_cast<std::uint32_t>(point->g) << 8U) |
                                       static_cast<std::uint32_t>(point->b);
      float rgb;
      std::memcpy(&rgb, &packed_rgb, sizeof(rgb));
      stream.write(reinterpret_cast<const char *>(position), sizeof(position));
      stream.write(reinterpret_cast<const char *>(&rgb), sizeof(rgb));
    }
    stream.flush();
    stream.close();
    result.file_write_time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - write_start)
            .count();
    if (!stream) {
      result.error = "failed to write colored PCD";
      fs::remove(temporary_path);
      return result;
    }
    fs::rename(temporary_path, output_path);
    result.success = true;
    result.output_path = output_path.string();
    result.points_written = valid_points.size();
  } catch (const std::exception &exception) {
    result.error = exception.what();
  }
  result.total_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - total_start)
          .count();
  return result;
}

PcdWriteResult writeColoredMapPcd(const ColoredGlobalMap &map,
                                  const std::string &output_directory) {
  PcdWriteResult result;
  const auto total_start = std::chrono::steady_clock::now();
  const std::size_t expected_points = map.size();
  if (expected_points == 0U) {
    result.error = "colored global map is empty";
    return result;
  }
  try {
    namespace fs = std::filesystem;
    const fs::path directory = fs::absolute(fs::path(output_directory));
    fs::create_directories(directory);
    const fs::path output_path = directory / "cloud_color.pcd";
    const fs::path temporary_path = output_path.string() + ".tmp";
    std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
    if (!stream) {
      result.error = "failed to open temporary colored PCD file";
      return result;
    }
    stream << "# .PCD v0.7 - Point Cloud Data file\n"
           << "VERSION 0.7\n"
           << "FIELDS x y z intensity rgb\n"
           << "SIZE 4 4 4 4 4\n"
           << "TYPE F F F F F\n"
           << "COUNT 1 1 1 1 1\n"
           << "WIDTH " << expected_points << "\n"
           << "HEIGHT 1\n"
           << "VIEWPOINT 0 0 0 1 0 0 0\n"
           << "POINTS " << expected_points << "\n"
           << "DATA binary\n";
    if (!stream) {
      result.error = "failed to write colored PCD header";
      stream.close();
      fs::remove(temporary_path);
      return result;
    }
    const auto write_start = std::chrono::steady_clock::now();
    std::size_t written = 0U;
    const bool visited = map.forEachPoint([&](const ColoredPoint &point) {
      if (!point.position.allFinite() || !std::isfinite(point.intensity) || !stream) {
        return;
      }
      const float position[4] = {point.position.x(), point.position.y(), point.position.z(),
                                 point.intensity};
      const std::uint32_t packed_rgb = (static_cast<std::uint32_t>(point.r) << 16U) |
                                       (static_cast<std::uint32_t>(point.g) << 8U) |
                                       static_cast<std::uint32_t>(point.b);
      float rgb;
      std::memcpy(&rgb, &packed_rgb, sizeof(rgb));
      stream.write(reinterpret_cast<const char *>(position), sizeof(position));
      stream.write(reinterpret_cast<const char *>(&rgb), sizeof(rgb));
      if (stream) {
        ++written;
      }
    });
    stream.flush();
    stream.close();
    result.file_write_time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - write_start)
            .count();
    if (!visited || !stream || written != expected_points) {
      result.error = "failed to stream the complete colored global map";
      fs::remove(temporary_path);
      return result;
    }
    fs::rename(temporary_path, output_path);
    result.success = true;
    result.output_path = output_path.string();
    result.points_written = written;
  } catch (const std::exception &exception) {
    result.error = exception.what();
  }
  result.total_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - total_start)
          .count();
  return result;
}

} // namespace itera_lio::output
