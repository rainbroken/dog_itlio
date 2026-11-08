// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>
#include <cstring>
#include <vector>

#include "itera_lio/mapping/color/colored_global_map.hpp"
#include "itera_lio/output/pcd_writer.hpp"

TEST(FullCloudPcdWriter, ExportsLocalScanUsingRecordedPose) {
  const auto directory = std::filesystem::temp_directory_path() /
                         "itera_lio_full_scan_pose_test";
  std::filesystem::remove_all(directory);
  itera_lio::output::FullCloudPcdWriter writer(directory.string());
  ASSERT_TRUE(writer.error().empty()) << writer.error();
  itera_lio::common::Pose pose;
  pose.p_WI = Eigen::Vector3d(10.0, 0.0, 0.0);
  itera_lio::common::FixedExtrinsics extrinsics;
  extrinsics.p_IL = Eigen::Vector3d(0.0, 2.0, 0.0);
  itera_lio::common::MapPoint point{Eigen::Vector3f(1.0F, 0.0F, 0.0F), 7.0F};
  ASSERT_TRUE(writer.append({point}, 1.5, pose, extrinsics)) << writer.error();
  const auto result = writer.finalize();
  ASSERT_TRUE(result.success) << result.error;
  std::ifstream stream(result.output_path, std::ios::binary);
  std::string line;
  while (std::getline(stream, line) && line != "DATA binary") {}
  float record[4]{};
  stream.read(reinterpret_cast<char *>(record), sizeof(record));
  ASSERT_TRUE(stream);
  EXPECT_FLOAT_EQ(record[0], 11.0F);
  EXPECT_FLOAT_EQ(record[1], 2.0F);
  EXPECT_FLOAT_EQ(record[3], 7.0F);
  std::filesystem::remove_all(directory);
}

TEST(FullCloudPcdWriter, AppliesGraphCorrectionToRecordedScanPose) {
  const auto directory = std::filesystem::temp_directory_path() /
                         "itera_lio_full_scan_optimized_test";
  std::filesystem::remove_all(directory);
  itera_lio::output::FullCloudPcdWriter writer(directory.string());
  itera_lio::common::Pose scan_pose;
  scan_pose.p_WI.x() = 0.5;
  itera_lio::common::MapPoint point{Eigen::Vector3f::Zero(), 1.0F};
  ASSERT_TRUE(writer.append({point}, 0.5, scan_pose,
                            itera_lio::common::FixedExtrinsics{}));
  std::vector<itera_lio::backend::Keyframe> raw(2U), optimized(2U);
  for (std::size_t i = 0; i < 2U; ++i) {
    raw[i].id = i;
    raw[i].timestamp = static_cast<double>(i);
    optimized[i] = raw[i];
  }
  optimized[1].pose_WI.p_WI.x() = 2.0;
  const auto result = writer.optimizedSnapshot(raw, optimized);
  ASSERT_TRUE(result.success) << result.error;
  std::ifstream stream(result.output_path, std::ios::binary);
  std::string line;
  while (std::getline(stream, line) && line != "DATA binary") {}
  float record[4]{};
  stream.read(reinterpret_cast<char *>(record), sizeof(record));
  ASSERT_TRUE(stream);
  EXPECT_FLOAT_EQ(record[0], 1.5F);
  std::filesystem::remove_all(directory);
}

TEST(ColoredGlobalMap, KeepsOneRepresentativePerVoxel) {
  itera_lio::mapping::color::ColoredGlobalMap map(1.0F);
  itera_lio::mapping::color::ColoredPoint near_center;
  near_center.position = Eigen::Vector3f(0.49F, 0.49F, 0.49F);
  near_center.r = 10U;
  itera_lio::mapping::color::ColoredPoint far_from_center;
  far_from_center.position = Eigen::Vector3f(0.01F, 0.01F, 0.01F);
  far_from_center.r = 20U;
  EXPECT_EQ(map.addPoints({far_from_center, near_center}), 1U);
  EXPECT_EQ(map.addPoints({{{1.1F, 0.1F, 0.1F}, 0.0F, 30U, 0U, 0U}}), 1U);

  std::vector<itera_lio::mapping::color::ColoredPoint> points;
  ASSERT_TRUE(map.getAllPoints(points));
  ASSERT_EQ(points.size(), 2U);
  const auto first =
      std::find_if(points.begin(), points.end(),
                   [](const auto &point) { return point.position.x() < 1.0F; });
  ASSERT_NE(first, points.end());
  EXPECT_EQ(first->r, 10U);
}

TEST(ColoredGlobalMap, ReportsFinalRepresentativeDelta) {
  itera_lio::mapping::color::ColoredGlobalMap map(1.0F);
  itera_lio::mapping::color::ColoredGlobalMapUpdate update;
  map.addPoints({{{0.1F, 0.1F, 0.1F}, 0.0F, 1U, 0U, 0U},
                 {{0.49F, 0.49F, 0.49F}, 0.0F, 2U, 0U, 0U}},
                &update);
  EXPECT_EQ(update.inserted_count, 1U);
  EXPECT_EQ(update.replaced_count, 1U);
  ASSERT_EQ(update.upserted_points.size(), 1U);
  EXPECT_EQ(update.upserted_points.front().r, 2U);
}

TEST(ColoredGlobalMap, IgnoresNonFinitePoints) {
  itera_lio::mapping::color::ColoredGlobalMap map(1.0F);
  itera_lio::mapping::color::ColoredPoint point;
  point.position.x() = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(map.addPoints({point}), 0U);
  EXPECT_EQ(map.size(), 0U);
}

TEST(ColoredGlobalMap, HonorsConfiguredPointBudget) {
  itera_lio::mapping::color::ColoredGlobalMap map(1.0F, 2U);
  EXPECT_GT(map.addPoints({{{0.1F, 0.1F, 0.1F}, 0.0F, 1U, 0U, 0U},
                           {{1.1F, 0.1F, 0.1F}, 0.0F, 2U, 0U, 0U},
                           {{2.1F, 0.1F, 0.1F}, 0.0F, 3U, 0U, 0U}}),
            0U);
  EXPECT_LE(map.size(), 2U);
  std::vector<itera_lio::mapping::color::ColoredPoint> points;
  ASSERT_TRUE(map.getAllPoints(points));
  EXPECT_LE(points.size(), 2U);
}

TEST(ColoredGlobalMap, SpillsColdChunksAndReloadsThem) {
  const auto directory =
      std::filesystem::temp_directory_path() / "itera_lio_colored_chunk_test";
  std::filesystem::remove_all(directory);
  {
    itera_lio::mapping::color::ColoredGlobalMap map(1.0F, 0U, 1U,
                                                           directory.string());
    EXPECT_EQ(map.addPoints({{{0.1F, 0.1F, 0.1F}, 0.0F, 1U, 0U, 0U},
                             {{6.1F, 0.1F, 0.1F}, 0.0F, 2U, 0U, 0U},
                             {{-6.1F, 0.1F, 0.1F}, 0.0F, 3U, 0U, 0U}}),
              3U);
    EXPECT_EQ(map.size(), 3U);
    EXPECT_LE(map.residentChunkCount(), 1U);
    EXPECT_GE(map.spilledChunkCount(), 2U);

    EXPECT_EQ(map.addPoints({{{0.49F, 0.49F, 0.49F}, 0.0F, 9U, 0U, 0U}}), 0U);
    std::vector<itera_lio::mapping::color::ColoredPoint> points;
    ASSERT_TRUE(map.getAllPoints(points));
    ASSERT_EQ(points.size(), 3U);
    const auto replaced =
        std::find_if(points.begin(), points.end(), [](const auto &point) {
          return point.position.x() > 0.0F && point.position.x() < 1.0F;
        });
    ASSERT_NE(replaced, points.end());
    EXPECT_EQ(replaced->r, 9U);

    const auto result =
        itera_lio::output::writeColoredMapPcd(map, directory.string());
    ASSERT_TRUE(result.success) << result.error;
    EXPECT_EQ(result.points_written, 3U);
    EXPECT_EQ(std::filesystem::path(result.output_path).filename(),
              "cloud_color.pcd");
  }
  std::filesystem::remove_all(directory);
}

TEST(ColoredGlobalMap, WritesRgbPcd) {
  const auto directory =
      std::filesystem::temp_directory_path() / "itera_lio_colored_pcd_test";
  std::filesystem::remove_all(directory);
  itera_lio::mapping::color::ColoredPoint first;
  first.position = Eigen::Vector3f(1.0F, 2.0F, 3.0F);
  first.intensity = 0.5F;
  first.r = 10U;
  first.g = 20U;
  first.b = 30U;
  itera_lio::mapping::color::ColoredPoint second;
  second.position = Eigen::Vector3f(4.0F, 5.0F, 6.0F);
  second.intensity = 0.6F;
  second.r = 40U;
  second.g = 50U;
  second.b = 60U;
  const std::vector<itera_lio::mapping::color::ColoredPoint> points = {
      first, second};

  const auto result =
      itera_lio::output::writeColoredMapPcd(points, directory.string());
  ASSERT_TRUE(result.success) << result.error;
  ASSERT_EQ(result.points_written, 2U);
  EXPECT_EQ(std::filesystem::path(result.output_path).filename(),
            "cloud_color.pcd");

  std::ifstream stream(result.output_path, std::ios::binary);
  ASSERT_TRUE(stream);
  std::string header;
  std::getline(stream, header);
  EXPECT_EQ(header, "# .PCD v0.7 - Point Cloud Data file");
  std::getline(stream, header);
  EXPECT_EQ(header, "VERSION 0.7");
  std::getline(stream, header);
  EXPECT_EQ(header, "FIELDS x y z intensity rgb");
  std::filesystem::remove_all(directory);
}

TEST(DenseColoredPcdWriter, PreservesRepeatedObservations) {
  const auto directory =
      std::filesystem::temp_directory_path() / "itera_lio_dense_colored_pcd_test";
  std::filesystem::remove_all(directory);
  itera_lio::mapping::color::ColoredPoint first;
  first.position = Eigen::Vector3f(1.0F, 2.0F, 3.0F);
  first.r = 10U;
  itera_lio::mapping::color::ColoredPoint second = first;
  second.position.x() += 0.01F;
  second.r = 20U;
  itera_lio::output::DenseColoredPcdWriter writer(directory.string());
  ASSERT_TRUE(writer.error().empty()) << writer.error();
  ASSERT_TRUE(writer.append({first, second})) << writer.error();
  const auto result = writer.finalize();
  ASSERT_TRUE(result.success) << result.error;
  EXPECT_EQ(result.points_written, 2U);
  EXPECT_EQ(std::filesystem::path(result.output_path).filename(),
            "cloud_color.pcd");

  std::ifstream stream(result.output_path, std::ios::binary);
  ASSERT_TRUE(stream);
  std::string line;
  bool found_width = false;
  while (std::getline(stream, line)) {
    if (line == "WIDTH 2") {
      found_width = true;
      break;
    }
  }
  EXPECT_TRUE(found_width);
  std::filesystem::remove_all(directory);
}

TEST(DenseColoredPcdWriter, ExportsLocalColorUsingRecordedPose) {
  const auto directory = std::filesystem::temp_directory_path() /
                         "itera_lio_local_color_pose_test";
  std::filesystem::remove_all(directory);
  itera_lio::output::DenseColoredPcdWriter writer(directory.string());
  itera_lio::mapping::color::ColoredPoint point;
  point.position = Eigen::Vector3f(1.0F, 0.0F, 0.0F);
  point.intensity = 4.0F;
  point.r = 25U;
  itera_lio::common::Pose pose;
  pose.p_WI.x() = 10.0;
  itera_lio::common::FixedExtrinsics extrinsics;
  extrinsics.p_IL.y() = 2.0;
  ASSERT_TRUE(writer.append({point}, 1.5, pose, extrinsics));
  const auto result = writer.finalize();
  ASSERT_TRUE(result.success) << result.error;
  std::ifstream stream(result.output_path, std::ios::binary);
  std::string line;
  while (std::getline(stream, line) && line != "DATA binary") {}
  float record[5]{};
  stream.read(reinterpret_cast<char *>(record), sizeof(record));
  ASSERT_TRUE(stream);
  EXPECT_FLOAT_EQ(record[0], 11.0F);
  EXPECT_FLOAT_EQ(record[1], 2.0F);
  EXPECT_FLOAT_EQ(record[3], 4.0F);
  std::filesystem::remove_all(directory);
}

TEST(DenseColoredPcdWriter, InterpolatesPoseGraphCorrectionPerScan) {
  const auto directory = std::filesystem::temp_directory_path() /
                         "itera_lio_optimized_colored_pcd_test";
  std::filesystem::remove_all(directory);
  itera_lio::output::DenseColoredPcdWriter writer(directory.string());
  itera_lio::mapping::color::ColoredPoint point;
  point.position = Eigen::Vector3f::Zero();
  point.r = 42U;
  ASSERT_TRUE(writer.append({point}, 0.0));
  ASSERT_TRUE(writer.append({point}, 0.5));
  ASSERT_TRUE(writer.append({point}, 1.0));
  std::vector<itera_lio::backend::Keyframe> raw(2U), optimized(2U);
  for (std::size_t index = 0; index < 2U; ++index) {
    raw[index].id = index;
    raw[index].timestamp = static_cast<double>(index);
    optimized[index] = raw[index];
  }
  optimized[1].pose_WI.p_WI.x() = 2.0;
  std::mutex archive_mutex;
  const auto preview = writer.optimizedPreview(raw, optimized, archive_mutex);
  ASSERT_TRUE(preview.success) << preview.error;
  EXPECT_EQ(preview.points_written, 3U);
  EXPECT_FALSE(std::filesystem::exists(directory / "cloud_color_optimized.pcd"));
  std::ifstream manifest(preview.output_path);
  const std::string metadata((std::istreambuf_iterator<char>(manifest)), {});
  EXPECT_NE(metadata.find("scanforge-blocks-v1"), std::string::npos);
  EXPECT_NE(metadata.find("\"transform\":[1,0,0,1,0,1,0,0,0,0,1,0]"), std::string::npos);
  const auto result = writer.optimizedSnapshot(raw, optimized);
  ASSERT_TRUE(result.success) << result.error;
  ASSERT_EQ(result.points_written, 3U);
  std::ifstream stream(result.output_path, std::ios::binary);
  ASSERT_TRUE(stream);
  std::string line;
  while (std::getline(stream, line) && line != "DATA binary") {}
  ASSERT_EQ(line, "DATA binary");
  for (const float expected_x : {0.0F, 1.0F, 2.0F}) {
    float record[5];
    stream.read(reinterpret_cast<char *>(record), sizeof(record));
    ASSERT_TRUE(stream);
    EXPECT_FLOAT_EQ(record[0], expected_x);
    EXPECT_FLOAT_EQ(record[1], 0.0F);
  }
  std::filesystem::remove_all(directory);
}

TEST(DenseColoredPcdWriter, InterpolatesPosesBeforeApplyingRotationCorrection) {
  const auto directory = std::filesystem::temp_directory_path() /
                         "itera_lio_rotating_colored_pcd_test";
  std::filesystem::remove_all(directory);
  itera_lio::output::DenseColoredPcdWriter writer(directory.string());
  itera_lio::mapping::color::ColoredPoint point;
  point.position = Eigen::Vector3f(10.0F, 0.0F, 0.0F);
  ASSERT_TRUE(writer.append({point}, 0.5));
  std::vector<itera_lio::backend::Keyframe> raw(2U), optimized(2U);
  for (std::size_t index = 0; index < 2U; ++index) {
    raw[index].id = index;
    raw[index].timestamp = static_cast<double>(index);
    raw[index].pose_WI.p_WI = Eigen::Vector3d(10.0, 0.0, 0.0);
    optimized[index] = raw[index];
  }
  optimized[1].pose_WI.R_WI =
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto result = writer.optimizedSnapshot(raw, optimized);
  ASSERT_TRUE(result.success) << result.error;
  std::ifstream stream(result.output_path, std::ios::binary);
  ASSERT_TRUE(stream);
  std::string line;
  while (std::getline(stream, line) && line != "DATA binary") {}
  ASSERT_EQ(line, "DATA binary");
  float record[5];
  stream.read(reinterpret_cast<char *>(record), sizeof(record));
  ASSERT_TRUE(stream);
  EXPECT_NEAR(record[0], 10.0F, 1e-4F);
  EXPECT_NEAR(record[1], 0.0F, 1e-4F);
  std::filesystem::remove_all(directory);
}

TEST(ColoredGlobalMap, FetchesBoundedResidentAndSpilledTiles) {
  const auto directory = std::filesystem::temp_directory_path() /
                         "itera_lio_colored_tile_test";
  std::filesystem::remove_all(directory);
  {
    itera_lio::mapping::color::ColoredGlobalMap map(1.0F, 0U, 1U,
                                                   directory.string());
    map.addPoints({{{0.1F, 0.1F, 0.1F}, 0.0F, 1U, 0U, 0U},
                   {{1.1F, 0.1F, 0.1F}, 0.0F, 2U, 0U, 0U},
                   {{6.1F, 0.1F, 0.1F}, 0.0F, 3U, 0U, 0U},
                   {{-6.1F, 0.1F, 0.1F}, 0.0F, 4U, 0U, 0U}});
    std::vector<itera_lio::mapping::color::ColoredPoint> points;
    ASSERT_TRUE(map.getChunkPoints(0, 0, 0, 1U, points));
    ASSERT_EQ(points.size(), 1U);
    EXPECT_GE(points.front().position.x(), 0.0F);
    ASSERT_TRUE(map.getChunkPoints(-2, 0, 0, 10U, points));
    ASSERT_EQ(points.size(), 1U);
    EXPECT_EQ(points.front().r, 4U);
    ASSERT_TRUE(map.getChunkPoints(100, 0, 0, 10U, points));
    EXPECT_TRUE(points.empty());
  }
  std::filesystem::remove_all(directory);
}
