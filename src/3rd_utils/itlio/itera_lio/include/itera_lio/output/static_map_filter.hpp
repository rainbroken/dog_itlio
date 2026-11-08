// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0
#pragma once

#include "itera_lio/common/voxel.hpp"
#include "itera_lio/output/pcd_result.hpp"
#include <unordered_map>
#include <vector>

namespace itera_lio::output {

// Evidence resolution is independent of the saved PCD resolution. Unknown or
// weakly observed geometry is kept; absence of observations is never free space.
struct StaticMapFilterOptions {
  bool enabled{false};
  bool keep_debug_artifacts{false};
  float evidence_voxel_size{0.20F};
  double max_range{25.0};
  double endpoint_margin{0.15};
  double margin_per_meter{0.005};
  double max_endpoint_margin{0.35};
  double observer_voxel_size{0.50};
  unsigned min_free_observations{3};
  double min_scan_interval{0.20};
  void validate() const;
};

class StaticMapFilter {
public:
  explicit StaticMapFilter(StaticMapFilterOptions options);
  void addCandidate(const Eigen::Vector3d &point);
  void observe(const Eigen::Vector3d &origin, const std::vector<Eigen::Vector3d> &points);
  bool keep(const Eigen::Vector3d &point) const;
  bool stable(const Eigen::Vector3d &point) const;
  std::size_t candidateCount() const { return evidence_.size(); }
  std::size_t rejectedCount() const;

private:
  struct Evidence {
    std::size_t hits{0}, misses{0};
    common::VoxelKey first_free_observer;
    bool multiple_free_observers{false};
  };
  bool rejected(const Evidence &value) const;
  StaticMapFilterOptions options_;
  std::unordered_map<common::VoxelKey, Evidence, common::VoxelKeyHash> evidence_;
};

// Reads raw LiDAR-local scans with their exact poses/extrinsics. Produces
// cloud_static.pcd, localization.pcd and a report. Debug mode also saves removed
// points. Never changes the source cloud.pcd or scan archive.
PcdWriteResult exportStaticMap(const std::string &directory, float pcd_voxel_size,
                              const StaticMapFilterOptions &options);
// Call only after maps AND matching keyframe poses were saved successfully.
// Active snapshots retain the archive needed by subsequent scans/saves.
void compactStaticMapFiles(const std::string &directory, bool finalized);
}  // namespace itera_lio::output
