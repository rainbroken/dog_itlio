// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0
#include "itera_lio/output/static_map_filter.hpp"
#include "itera_lio/output/scan_archive.hpp"
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <chrono>
#include <filesystem>
#include <fstream>

using itera_lio::output::StaticMapFilter;
using Eigen::Vector3d;

TEST(StaticMapFilter, ClearsContradictedReturnsInEveryDirection) {
  for (const auto &axis : {Vector3d::UnitY().eval(), Vector3d::UnitZ().eval()}) {
  for (double angle : {0.0, 1.5707963267948966, 3.141592653589793, -1.5707963267948966}) {
    const Eigen::AngleAxisd rotation(angle, axis);
    const auto at = [&](double x) -> Vector3d {return rotation * Vector3d(x, .07, .07);};
    StaticMapFilter filter({});
    filter.addCandidate(at(2.07)); filter.addCandidate(at(6.07));
    filter.observe(at(.07), {at(2.07)});
    for (int i = 0; i < 6; ++i) {filter.observe(at(.07 + i*.21), {at(6.07)});}
    EXPECT_FALSE(filter.keep(at(2.07))) << angle;
    EXPECT_TRUE(filter.keep(at(6.07))) << angle;
  }
  }
}

TEST(StaticMapFilter, KeepsUnseenOccludedAndSingleViewGeometry) {
  StaticMapFilter filter({});
  const Vector3d human(2.07,.07,.07), hidden(8.07,.07,.07), rare(3,2,1);
  filter.addCandidate(human); filter.addCandidate(hidden); filter.addCandidate(rare);
  filter.observe(Vector3d(.07,.07,.07), {hidden, rare});
  for (int i = 0; i < 10; ++i) {
    filter.observe(Vector3d(.07+i*.1,.07,.07), {human});
  }
  EXPECT_TRUE(filter.keep(hidden)); // no ray ever reaches behind the occluder
  EXPECT_TRUE(filter.keep(rare));   // not observed again is not empty space
  EXPECT_TRUE(filter.keep(human)); // repeatedly occupied geometry is retained
}

TEST(StaticMapFilter, RequiresDistinctObserverPositionsAndProtectsEndpoints) {
  StaticMapFilter filter({});
  const Vector3d ghost(2.07,.07,.07), wall(5.07,.07,.07), near_wall(4.97,.07,.07);
  filter.addCandidate(ghost); filter.addCandidate(wall); filter.addCandidate(near_wall);
  filter.observe(Vector3d(.07,.07,.07), {ghost, near_wall});
  for (int i = 0; i < 30; ++i) {filter.observe(Vector3d(.07,.07,.07), {wall});}
  EXPECT_TRUE(filter.keep(ghost)); // one observer voxel cannot clear a return
  EXPECT_TRUE(filter.keep(near_wall)); // endpoint margin protects nearby surfaces
}

TEST(StaticMapFilter, DoesNotClearBeyondRangeAndRejectsInvalidOptions) {
  StaticMapFilter filter({});
  const Vector3d far(30.07,.07,.07);
  filter.addCandidate(far); filter.observe(Vector3d(.07,.07,.07), {far});
  for (int i = 0; i < 10; ++i) {
    filter.observe(Vector3d(.07+i*.1,.07,.07), {Vector3d(40,.07,.07)});
  }
  EXPECT_TRUE(filter.keep(far));
  itera_lio::output::StaticMapFilterOptions options;
  options.evidence_voxel_size = 0;
  EXPECT_THROW(StaticMapFilter invalid(options), std::invalid_argument);
}

TEST(StaticMapFilter, ArchiveExportPreservesRawAndPcdResolutionOnRepeatedSave) {
  namespace fs = std::filesystem;
  const auto root = fs::temp_directory_path()/(
      "static-filter-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  itera_lio::output::ScanArchiveOptions options;
  options.directory = root.string(); options.full_deskewed = true;
  options.pcd_voxel_size = .05F; options.static_filter.enabled = true;
  {
    itera_lio::output::ScanArchive archive(options);
    for (int i = 0; i < 8; ++i) {
      itera_lio::sensor::LidarFrame scan;
      scan.scan_begin_time = scan.scan_end_time = 1+i*.3;
      itera_lio::sensor::LidarPoint p;
      p.position = Vector3d((i == 0 ? 2.07 : 6.07)-i*.1,.07,.07);
      p.intensity = 1; scan.points.push_back(p);
      // Adjacent 5cm samples must not be merged by the 20cm evidence grid.
      p.position.y() += .06; scan.points.push_back(p);
      itera_lio::common::Pose pose; pose.p_WI.x() = i*.1;
      ASSERT_TRUE(archive.appendFull(scan, pose, {}).success);
    }
    ASSERT_TRUE(archive.snapshot({}, {}, false).success);
    const auto bytes = fs::file_size(root/"cloud.pcd");
    std::ofstream(root/"localization_keyframes.csv") << "id,timestamp,px,py,pz,qx,qy,qz,qw\n0,1,0,0,0,0,0,0,1\n";
    itera_lio::output::compactStaticMapFiles(root.string(), false);
    EXPECT_FALSE(fs::exists(root/"cloud.pcd"));
    EXPECT_TRUE(fs::exists(root/"full_deskewed_points.bin"));
    itera_lio::sensor::LidarFrame extra;
    extra.scan_begin_time = extra.scan_end_time = 4;
    itera_lio::sensor::LidarPoint endpoint;
    endpoint.position = Vector3d(6.07,.07,.07); endpoint.intensity = 1;
    extra.points.push_back(endpoint);
    ASSERT_TRUE(archive.appendFull(extra, {}, {}).success);
    ASSERT_TRUE(archive.snapshot({}, {}, false).success);
    EXPECT_EQ(fs::file_size(root/"cloud.pcd"), bytes);
    ASSERT_TRUE(archive.finalize().success);
    std::ifstream report(root/"dynamic_filter_report.json");
    const std::string text((std::istreambuf_iterator<char>(report)), {});
    EXPECT_NE(text.find("\"kept_points\":2"), std::string::npos);
    EXPECT_NE(text.find("\"removed_points\":2"), std::string::npos);
    EXPECT_TRUE(fs::exists(root/"full_deskewed_points.bin"));
    EXPECT_TRUE(fs::exists(root/"localization.pcd"));
    EXPECT_FALSE(fs::exists(root/"cloud_removed.pcd"));
    fs::remove(root/"localization_keyframes.csv");
    EXPECT_THROW(itera_lio::output::compactStaticMapFiles(root.string(), true), std::exception);
    EXPECT_TRUE(fs::exists(root/"full_deskewed_points.bin"));
    std::ofstream(root/"localization_keyframes.csv") << "id,timestamp,px,py,pz,qx,qy,qz,qw\n0,1,0,0,0,0,0,0,1\n";
    itera_lio::output::compactStaticMapFiles(root.string(), false);
    EXPECT_FALSE(fs::exists(root/"cloud.pcd"));
    EXPECT_TRUE(fs::exists(root/"full_deskewed_points.bin"));
    itera_lio::output::compactStaticMapFiles(root.string(), true);
    EXPECT_FALSE(fs::exists(root/"full_deskewed_points.bin"));
    EXPECT_FALSE(fs::exists(root/"full_deskewed_scans.csv"));
  }
  fs::remove_all(root);
}
