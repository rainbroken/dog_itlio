// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0
#include "itera_lio/output/static_map_filter.hpp"
#include <Eigen/Geometry>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace itera_lio::output {
namespace {
namespace fs = std::filesystem;
struct Scan {
  double timestamp;
  std::size_t first, count;
  Eigen::Quaterniond rotation, extrinsic;
  Eigen::Vector3d origin, translation, offset;
};
std::vector<std::string> split(const std::string &line) {
  std::istringstream stream(line);
  std::vector<std::string> result;
  std::string field;
  while (std::getline(stream, field, ',')) {result.push_back(field);}
  return result;
}
class Pcd {
public:
  explicit Pcd(const fs::path &path) : path_(path), stream_(path.string()+".tmp", std::ios::binary) {
    if (!stream_) {throw std::runtime_error("cannot open static PCD temporary file");}
    stream_ << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
               "TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH ";
    width_ = stream_.tellp();
    stream_ << std::setw(20) << 0 << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS ";
    points_ = stream_.tellp();
    stream_ << std::setw(20) << 0 << "\nDATA binary\n";
  }
  ~Pcd() {if (!committed_) {std::error_code error; fs::remove(path_.string()+".tmp", error);}}
  void write(const Eigen::Vector3d &point, float intensity) {
    const float data[4] = {float(point.x()), float(point.y()), float(point.z()), intensity};
    stream_.write(reinterpret_cast<const char *>(data), sizeof(data));
    ++count;
  }
  void commit() {
    stream_.seekp(width_); stream_ << std::setw(20) << count;
    stream_.seekp(points_); stream_ << std::setw(20) << count;
    stream_.flush(); stream_.close();
    if (!stream_) {throw std::runtime_error("static PCD write failed");}
    fs::rename(path_.string()+".tmp", path_); committed_ = true;
  }
  std::size_t count{0};
private:
  fs::path path_;
  std::ofstream stream_;
  std::streampos width_, points_;
  bool committed_{false};
};
}

PcdWriteResult exportStaticMap(const std::string &directory, float pcd_voxel_size,
                              const StaticMapFilterOptions &options) {
  const auto started = std::chrono::steady_clock::now();
  try {
    options.validate();
    if (!std::isfinite(pcd_voxel_size) || pcd_voxel_size < 0) {
      throw std::invalid_argument("invalid saved PCD voxel size");
    }
    const fs::path root(directory);
    std::ifstream index(root/"full_deskewed_scans.csv");
    std::ifstream binary(root/"full_deskewed_points.bin", std::ios::binary);
    if (!index || !binary) {throw std::runtime_error("full deskewed archive unavailable");}
    std::vector<Scan> scans;
    std::string line;
    std::getline(index, line);
    std::size_t total = 0;
    while (std::getline(index, line)) {
      const auto fields = split(line);
      if (fields.size() != 17) {throw std::runtime_error("invalid scan index columns");}
      Scan scan;
      scan.timestamp = std::stod(fields[0]);
      scan.first = std::stoull(fields[1]); scan.count = std::stoull(fields[2]);
      const Eigen::Vector3d translation(std::stod(fields[3]), std::stod(fields[4]), std::stod(fields[5]));
      const Eigen::Quaterniond q(std::stod(fields[9]), std::stod(fields[6]), std::stod(fields[7]), std::stod(fields[8]));
      const Eigen::Vector3d offset(std::stod(fields[10]), std::stod(fields[11]), std::stod(fields[12]));
      const Eigen::Quaterniond extrinsic(std::stod(fields[16]), std::stod(fields[13]), std::stod(fields[14]), std::stod(fields[15]));
      if (!std::isfinite(scan.timestamp) || scan.first != total ||
          (!scans.empty() && scan.timestamp <= scans.back().timestamp) ||
          !translation.allFinite() || !offset.allFinite() ||
          !q.coeffs().allFinite() || !extrinsic.coeffs().allFinite() ||
          std::abs(q.norm()-1) > 1e-6 || std::abs(extrinsic.norm()-1) > 1e-6 ||
          scan.count > fs::file_size(root/"full_deskewed_points.bin") / 16 - total) {
        throw std::runtime_error("invalid archive pose or point range");
      }
      scan.rotation = q; scan.extrinsic = extrinsic;
      scan.translation = translation; scan.offset = offset;
      scan.origin = translation + q * offset;
      total += scan.count; scans.push_back(scan);
    }
    if (scans.empty() || fs::file_size(root/"full_deskewed_points.bin") != total * 16) {
      throw std::runtime_error("archive point count mismatch or empty archive");
    }
    // If available, failed tracking/insertion scans cannot cast clearing rays.
    // Their points remain candidates and are preserved without contradictory evidence.
    std::map<double, bool> quality;
    std::ifstream frames(root/"evaluation/frames.csv");
    if (frames) {
      std::getline(frames, line);
      while (std::getline(frames, line)) {
        const auto f = split(line);
        if (f.size() < 14) {throw std::runtime_error("invalid frame quality CSV");}
        quality[std::stod(f[1])] = f[9] == "normal" && (f[13] == "accepted" || f[13] == "bootstrap");
      }
    }
    auto read = [&](const Scan &scan, auto callback) {
      binary.clear(); binary.seekg(static_cast<std::streamoff>(scan.first * 16));
      std::array<float, 4096 * 4> block;
      for (std::size_t offset = 0; offset < scan.count;) {
        const auto count = std::min<std::size_t>(4096, scan.count-offset);
        binary.read(reinterpret_cast<char *>(block.data()), count * 16);
        if (!binary) {throw std::runtime_error("truncated full scan archive");}
        for (std::size_t i = 0; i < count; ++i) {
          const auto p = block.data()+4*i;
          const Eigen::Vector3d world = scan.rotation *
              (scan.extrinsic * Eigen::Vector3d(p[0],p[1],p[2]) + scan.offset) + scan.translation;
          if (!world.allFinite() || !std::isfinite(p[3])) {throw std::runtime_error("non-finite archive point");}
          callback(world, p[3]);
        }
        offset += count;
      }
    };
    StaticMapFilter filter(options);
    for (const auto &scan : scans) {
      read(scan, [&](const auto &p, float) {filter.addCandidate(p);});
    }
    std::cerr << "Static filter: " << scans.size() << " scans, " << filter.candidateCount() << " candidate voxels\n";
    std::size_t observations = 0, bad_quality = 0;
    double previous_time = -std::numeric_limits<double>::infinity();
    for (const auto &scan : scans) {
      if (!quality.empty()) {
        const auto found = quality.find(scan.timestamp);
        if (found == quality.end() || !found->second) {++bad_quality; continue;}
      }
      if (scan.timestamp - previous_time < options.min_scan_interval - 1e-6) {continue;}
      std::vector<Eigen::Vector3d> points;
      points.reserve(scan.count);
      read(scan, [&](const auto &p, float) {points.push_back(p);});
      filter.observe(scan.origin, points);
      previous_time = scan.timestamp; ++observations;
      if (observations % 200 == 0) {std::cerr << "Static filter observations: " << observations << '\n';}
    }
    Pcd kept(root/"cloud_static.pcd"), localization(root/"localization.pcd");
    std::unique_ptr<Pcd> removed;
    if (options.keep_debug_artifacts) {removed = std::make_unique<Pcd>(root/"cloud_removed.pcd");}
    std::size_t removed_count = 0;
    // Partition the exact saved samples, rather than reselecting representatives
    // from the archive. This preserves the original PCD resolution, intensity,
    // and float coordinates even for old runs produced by a different build.
    std::ifstream source(root/"cloud.pcd", std::ios::binary);
    std::size_t source_count = 0;
    bool fields_ok = false, sizes_ok = false, types_ok = false, counts_ok = false, binary_ok = false;
    while (std::getline(source, line)) {
      if (line == "FIELDS x y z intensity") {fields_ok = true;}
      if (line == "SIZE 4 4 4 4") {sizes_ok = true;}
      if (line == "TYPE F F F F") {types_ok = true;}
      if (line == "COUNT 1 1 1 1") {counts_ok = true;}
      if (line.rfind("POINTS ", 0) == 0) {source_count = std::stoull(line.substr(7));}
      if (line == "DATA binary") {binary_ok = true; break;}
    }
    if (!source || !fields_ok || !sizes_ok || !types_ok || !counts_ok || !binary_ok || source_count == 0) {
      throw std::runtime_error("cloud.pcd is not a supported XYZI binary saved map");
    }
    for (std::size_t i = 0; i < source_count; ++i) {
      float p[4]; source.read(reinterpret_cast<char *>(p), sizeof(p));
      if (!source) {throw std::runtime_error("truncated source PCD");}
      const Eigen::Vector3d point(p[0],p[1],p[2]);
      if (!point.allFinite() || !std::isfinite(p[3])) {throw std::runtime_error("non-finite source PCD point");}
      if (filter.keep(point)) {
        kept.write(point, p[3]);
        if (filter.stable(point)) {localization.write(point, p[3]);}
      } else {
        ++removed_count;
        if (removed) {removed->write(point, p[3]);}
      }
    }
    if (source.peek() != std::ifstream::traits_type::eof()) {throw std::runtime_error("unexpected trailing source PCD data");}
    // Commit report last. It describes only a completed pair of PCDs.
    if (localization.count == 0) {throw std::runtime_error("no stable localization points; retaining source files");}
    fs::remove(root/"dynamic_filter_report.json");
    kept.commit(); localization.commit();
    if (removed) {removed->commit();}
    else {fs::remove(root/"cloud_removed.pcd");}
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
    const auto report_path = root/"dynamic_filter_report.json";
    std::ofstream report(report_path.string()+".tmp");
    report << std::setprecision(12)
           << "{\n  \"algorithm\":\"multiview_ray_evidence_v1\",\n"
           << "  \"pose_frame\":\"raw_lio\",\n  \"scans\":" << scans.size()
           << ",\n  \"observations\":" << observations << ",\n  \"quality_rejected_scans\":" << bad_quality
           << ",\n  \"quality_gate_available\":" << (quality.empty()?"false":"true")
           << ",\n  \"pcd_voxel_size\":" << pcd_voxel_size
           << ",\n  \"evidence_voxel_size\":" << options.evidence_voxel_size
           << ",\n  \"max_range\":" << options.max_range
           << ",\n  \"endpoint_margin\":" << options.endpoint_margin
           << ",\n  \"margin_per_meter\":" << options.margin_per_meter
           << ",\n  \"max_endpoint_margin\":" << options.max_endpoint_margin
           << ",\n  \"observer_voxel_size\":" << options.observer_voxel_size
           << ",\n  \"min_free_observations\":" << options.min_free_observations
           << ",\n  \"min_scan_interval\":" << options.min_scan_interval
           << ",\n  \"candidate_voxels\":" << filter.candidateCount()
           << ",\n  \"removed_evidence_voxels\":" << filter.rejectedCount()
           << ",\n  \"input_points\":" << total
           << ",\n  \"keep_debug_artifacts\":" << (options.keep_debug_artifacts?"true":"false")
           << ",\n  \"localization_points\":" << localization.count
           << ",\n  \"localization_min_evidence_hits\":2"
           << ",\n  \"baseline_pcd_points\":" << kept.count+removed_count
           << ",\n  \"kept_points\":" << kept.count << ",\n  \"removed_points\":" << removed_count
           << ",\n  \"seconds\":" << seconds << "\n}\n";
    report.flush(); report.close();
    if (!report) {throw std::runtime_error("failed to write filter report");}
    fs::rename(report_path.string()+".tmp", report_path);
    PcdWriteResult result;
    result.success = true; result.output_path = (root/"cloud_static.pcd").string();
    result.points_written = kept.count;
    return result;
  } catch (const std::exception &error) {return {false, {}, error.what()};}
}
void compactStaticMapFiles(const std::string &directory, bool finalized) {
  const fs::path root(directory);
  for (const auto *name : {"cloud_static.pcd", "localization.pcd", "localization_keyframes.csv",
                           "dynamic_filter_report.json"}) {
    const auto path = root/name;
    if (!fs::is_regular_file(path) || fs::is_symlink(path) || fs::file_size(path) == 0) {
      throw std::runtime_error("cannot compact incomplete saved map: " + path.string());
    }
  }
  fs::remove(root/"cloud.pcd");
  fs::remove(root/"cloud_removed.pcd");
  if (finalized) {
    fs::remove(root/"full_deskewed_points.bin");
    fs::remove(root/"full_deskewed_scans.csv");
  }
}
}  // namespace itera_lio::output
