// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/output/graph_export.hpp"
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
PcdWriteResult writePoseGraphConstraintsCsv(const backend::PoseGraph &graph,
                                            const std::string &output_directory) {
  PcdWriteResult result;
  try {
    std::filesystem::create_directories(output_directory);
    const auto path = std::filesystem::path(output_directory) / "pose_graph_constraints.csv";
    const auto temporary = path.string() + ".tmp";
    std::ofstream stream(temporary);
    stream << "from_id,to_id,type,px,py,pz,qx,qy,qz,qw";
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        stream << ",i" << i << j;
      }
    }
    stream << '\n' << std::setprecision(17);
    for (const auto &edge : graph.constraints()) {
      const Eigen::Quaterniond q(edge.T_Ia_Ib.R_WI);
      stream << edge.from_id << ',' << edge.to_id << ',' << static_cast<int>(edge.type) << ','
             << edge.T_Ia_Ib.p_WI.x() << ',' << edge.T_Ia_Ib.p_WI.y() << ','
             << edge.T_Ia_Ib.p_WI.z() << ',' << q.x() << ',' << q.y() << ',' << q.z() << ','
             << q.w();
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
          stream << ',' << edge.information(i, j);
        }
      }
      stream << '\n';
    }
    stream.close();
    if (!stream) {
      result.error = "failed to write pose graph audit";
      return result;
    }
    std::filesystem::rename(temporary, path);
    result.success = true;
    result.output_path = path.string();
    result.points_written = graph.constraints().size();
  } catch (const std::exception &error) {
    result.error = error.what();
  }
  return result;
}


PcdWriteResult writeTrajectoryCsv(const std::vector<backend::Keyframe> &keyframes,
                                  const std::string &output_directory,
                                  const std::string &filename_suffix) {
  PcdWriteResult result;
  const auto total_start = std::chrono::steady_clock::now();
  if (keyframes.empty() || filename_suffix.empty()) {
    result.error = "trajectory or filename suffix is empty";
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
             << std::setw(3) << milliseconds << '_' << filename_suffix << ".csv";
    const fs::path output_path = directory / (
        filename_suffix == "localization_keyframes" || filename_suffix == "localization_optimized_keyframes" ?
        filename_suffix + ".csv" : filename.str());
    const fs::path temporary_path = output_path.string() + ".tmp";
    std::ofstream stream(temporary_path, std::ios::out | std::ios::trunc);
    if (!stream) {
      result.error = "failed to open temporary trajectory CSV";
      return result;
    }
    stream << "id,timestamp,px,py,pz,qx,qy,qz,qw\n" << std::setprecision(17);
    for (const auto &keyframe : keyframes) {
      const Eigen::Quaterniond orientation(keyframe.pose_WI.R_WI);
      stream << keyframe.id << ',' << keyframe.timestamp << ',' << keyframe.pose_WI.p_WI.x() << ','
             << keyframe.pose_WI.p_WI.y() << ',' << keyframe.pose_WI.p_WI.z() << ','
             << orientation.x() << ',' << orientation.y() << ',' << orientation.z() << ','
             << orientation.w() << '\n';
    }
    stream.flush();
    stream.close();
    if (!stream) {
      result.error = "failed to write trajectory CSV";
      fs::remove(temporary_path);
      return result;
    }
    fs::rename(temporary_path, output_path);
    result.success = true;
    result.output_path = output_path.string();
    result.points_written = keyframes.size();
  } catch (const std::exception &exception) {
    result.error = exception.what();
  }
  result.total_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - total_start)
          .count();
  return result;
}

} // namespace itera_lio::output
