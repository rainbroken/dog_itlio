// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include "itera_lio/frontend/ikd_map.hpp"
#include "itera_lio/frontend/scan_to_map.hpp"

TEST(ScanToMap, BuildsPointToPlaneObservation)
{
  itera_lio::frontend::IkdMap map(0.1F);
  map.build(
  {
    {{0.0F, 0.0F, 0.0F}}, {{1.0F, 0.0F, 0.0F}}, {{0.0F, 1.0F, 0.0F}},
    {{1.0F, 1.0F, 0.0F}}, {{0.5F, 0.5F, 0.0F}}});
  itera_lio::sensor::LidarFrame scan;
  scan.scan_begin_time = 0.0;
  scan.scan_end_time = 0.0;
  scan.points = {{{0.5, 0.5, 0.1}, 0.0F, 0.0}};
  itera_lio::frontend::ScanToMapOptions options;
  options.plane.max_neighbor_distance = 2.0;
  options.plane.max_point_to_plane_distance = 0.2;
  const auto report = itera_lio::frontend::buildScanToMapObservations(
    scan, itera_lio::common::Pose{}, itera_lio::common::FixedExtrinsics{}, map, options);
  ASSERT_EQ(report.effective_points, 1U);
  EXPECT_NEAR(std::abs(report.observations.front().normal_W.z()), 1.0, 1e-12);
  EXPECT_NEAR(report.observations.front().residual, 0.1, 1e-6);
}

TEST(ScanToMap, SpatialRepresentativeDoesNotCensorLargerValidInnovation)
{
  using namespace itera_lio;
  frontend::IkdMap map(0.01F);
  std::vector<frontend::MapPoint> points;
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 10; ++y) {
      points.push_back({{0.1F * x, 0.1F * y, 0.F}});
    }
  }
  map.build(points);
  sensor::LidarFrame scan;
  scan.points = {{{0.2, 0.2, 0.01}, 1.F, 0.}, {{0.5, 0.5, 0.1}, 2.F, 0.}};
  frontend::ScanToMapOptions options;
  options.max_effective_points = options.max_map_insert_points = 1;
  options.selection_voxel_size = 1.;
  const auto report = frontend::buildScanToMapObservations(scan, {}, {}, map, options);
  ASSERT_EQ(report.observations.size(), 1U);
  // Both measurements pass geometric gates. Better tangential coverage must
  // survive even when its innovation is larger.
  EXPECT_FLOAT_EQ(report.observations.front().intensity, 2.F);
  EXPECT_NEAR(std::abs(report.observations.front().residual), 0.1, 1e-6);
}

TEST(ScanToMap, TangentialCoverageDoesNotPreferOneSignOfNormalNoise)
{
  using namespace itera_lio;
  frontend::IkdMap map(0.01F);
  std::vector<frontend::MapPoint> points;
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 10; ++y) {
      points.push_back({{0.1F * x, 0.1F * y, 0.3F}});
    }
  }
  map.build(points);
  frontend::ScanToMapOptions options;
  options.max_effective_points = options.max_map_insert_points = 1;
  options.selection_voxel_size = 1.;
  for (double noise : {-0.01, 0.01}) {
    sensor::LidarFrame scan;
    scan.points = {{{0.5, 0.5, 0.3 + noise}, 1.F, 0.},
      {{0.5, 0.5, 0.3 - noise}, 2.F, 0.}};
    const auto report = frontend::buildScanToMapObservations(scan, {}, {}, map, options);
    ASSERT_EQ(report.observations.size(), 1U);
    EXPECT_FLOAT_EQ(report.observations.front().intensity, 1.F);
  }
}

TEST(ScanToMap, ReusesStablePlanesAcrossIterationsAndRefreshesLargeMotion)
{
  itera_lio::frontend::IkdMap map(0.1F);
  std::vector<itera_lio::frontend::MapPoint> points;
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 8; ++y) {
      points.push_back({{static_cast<float>(x), static_cast<float>(y), 0.0F}});
    }
  }
  map.build(points);
  itera_lio::sensor::LidarFrame scan;
  scan.scan_begin_time = 0.0;
  scan.scan_end_time = 0.0;
  for (int i = 0; i < 12; ++i) {
    scan.points.push_back({{1.0 + (i % 4), 1.0 + (i / 4), 0.1}, 0.0F, 0.0});
  }
  itera_lio::frontend::ScanToMapOptions options;
  options.knn = 5;
  options.max_candidate_points = 12;
  options.max_effective_points = 2;
  options.max_map_insert_points = 3;
  options.adaptive_candidates = true;
  options.plane.max_neighbor_distance = 3.0;
  options.plane.max_point_to_plane_distance = 0.2;
  itera_lio::frontend::ScanToMapIterationCache cache;
  itera_lio::common::Pose pose;
  const itera_lio::common::FixedExtrinsics extrinsics;
  const auto first = itera_lio::frontend::buildScanToMapObservations(
    scan, pose, extrinsics, map, options, &cache);
  ASSERT_EQ(first.knn_queries, 12U);
  ASSERT_EQ(first.effective_points, 2U);
  pose.p_WI.z() = 0.01;
  const auto second = itera_lio::frontend::buildScanToMapObservations(
    scan, pose, extrinsics, map, options, &cache);
  EXPECT_LT(second.candidate_attempts, first.candidate_attempts);
  EXPECT_EQ(second.knn_queries, 0U);
  EXPECT_EQ(second.reused_planes, second.candidate_attempts);
  ASSERT_FALSE(second.insertion_candidates.empty());
  EXPECT_LE(second.insertion_candidates.size(), 3U);
  EXPECT_NEAR(second.insertion_candidates.front().point_W.z(), 0.11, 1e-6);
  EXPECT_NEAR(std::abs(second.insertion_candidates.front().residual), 0.11, 1e-6);
  pose.p_WI.z() = 0.02;
  const auto third = itera_lio::frontend::buildScanToMapObservations(
    scan, pose, extrinsics, map, options, &cache);
  EXPECT_EQ(third.knn_queries, 0U);
  EXPECT_EQ(third.reused_planes, third.candidate_attempts);
  pose.p_WI.z() = 0.15;
  const auto fourth = itera_lio::frontend::buildScanToMapObservations(
    scan, pose, extrinsics, map, options, &cache);
  EXPECT_EQ(fourth.knn_queries, fourth.candidate_attempts);
  EXPECT_EQ(fourth.reused_planes, 0U);
}

TEST(ScanToMap, RetainsDistinctPlaneNormalsInsideOneSpatialCell)
{
  itera_lio::frontend::IkdMap map(0.01F);
  std::vector<itera_lio::frontend::MapPoint> points;
  for (int ix = 0; ix < 5; ++ix) {
    for (int iy = 0; iy < 5; ++iy) {
      points.push_back({{0.1F + 0.05F * ix, 0.1F + 0.05F * iy, 0.0F}});
      points.push_back({{0.8F, 0.1F + 0.05F * iy, 0.2F + 0.05F * ix}});
    }
  }
  map.build(points);
  itera_lio::sensor::LidarFrame scan;
  scan.scan_begin_time = 0.0;
  scan.scan_end_time = 0.0;
  scan.points = {
    {{0.2, 0.2, 0.03}, 0.0F, 0.0},
    {{0.77, 0.2, 0.3}, 0.0F, 0.0}};
  itera_lio::frontend::ScanToMapOptions options;
  options.max_effective_points = 2;
  options.selection_voxel_size = 1.0;
  options.plane.max_neighbor_distance = 0.2;
  options.plane.max_point_to_plane_distance = 0.1;
  const auto report = itera_lio::frontend::buildScanToMapObservations(
    scan, itera_lio::common::Pose{}, itera_lio::common::FixedExtrinsics{}, map, options);
  ASSERT_EQ(report.effective_points, 2U);
  EXPECT_LT(std::abs(report.observations[0].normal_W.dot(report.observations[1].normal_W)), 0.1);
}

TEST(ScanToMap, RepeatedAcceptedRejectedAcceptedCallsDoNotReuseStaleResults)
{
  using namespace itera_lio;
  frontend::IkdMap map(0.01F);
  std::vector<frontend::MapPoint> points;
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 10; ++y) {
      points.push_back({{float(x), float(y), 0.F}});
    }
  }
  map.build(points);
  sensor::LidarFrame scan;
  for (int i = 0; i < 600; ++i) {
    scan.points.push_back({{1. + i % 7, 1. + (i / 7) % 7, .1}, 0.F, 0.});
  }
  frontend::ScanToMapOptions options;
  options.max_candidate_points = 600;
  options.max_effective_points = 20;
  options.plane.max_neighbor_distance = 3.;
  options.plane.max_point_to_plane_distance = .2;
  auto guard = map.prepareConcurrentRead();
  common::Pose pose;
  const auto first = frontend::buildScanToMapObservations(scan, pose, {}, map, options);
  ASSERT_GT(first.effective_points, 0U);
  pose.p_WI.z() = 20.;
  const auto rejected = frontend::buildScanToMapObservations(scan, pose, {}, map, options);
  EXPECT_EQ(rejected.effective_points, 0U);
  EXPECT_TRUE(rejected.observations.empty());
  EXPECT_TRUE(rejected.insertion_candidates.empty());
  EXPECT_EQ(rejected.reused_planes, 0U);
  EXPECT_EQ(rejected.knn_queries, rejected.candidate_attempts);
  EXPECT_EQ(rejected.rejected_points, rejected.candidate_attempts);
  pose.p_WI.z() = 0.;
  const auto again = frontend::buildScanToMapObservations(scan, pose, {}, map, options);
  EXPECT_EQ(again.knn_queries, first.knn_queries);
  EXPECT_EQ(again.rejected_points, first.rejected_points);
  ASSERT_EQ(again.observations.size(), first.observations.size());
  for (std::size_t i = 0; i < first.observations.size(); ++i) {
    EXPECT_EQ(again.observations[i].point_W, first.observations[i].point_W);
    EXPECT_EQ(again.observations[i].normal_W, first.observations[i].normal_W);
    EXPECT_EQ(again.observations[i].residual, first.observations[i].residual);
  }
}

TEST(ScanToMap, WorkerCountsPreserveObservationOrderAndCacheDecisions)
{
  using namespace itera_lio;
  frontend::IkdMap map(0.01F);
  std::vector<frontend::MapPoint> points;
  for (int x = 0; x < 12; ++x) {
    for (int y = 0; y < 12; ++y) {
      points.push_back({{float(x), float(y), 0.F}});
    }
  }
  map.build(points);
  sensor::LidarFrame scan;
  for (int i = 0; i < 900; ++i) {
    scan.points.push_back({
      {1. + i % 8, 1. + (i / 8) % 8, i % 11 == 0 ? 8. : .1},
      float(i), 0.});
  }
  frontend::ScanToMapOptions options;
  options.max_candidate_points = scan.points.size();
  options.max_effective_points = 100;
  options.max_map_insert_points = 200;
  options.plane.max_neighbor_distance = 3.;
  options.plane.max_point_to_plane_distance = .2;
  auto guard = map.prepareConcurrentRead();
  ASSERT_TRUE(map.concurrentReadPrepared());
  const auto equal_batch = [](const common::ObservationBatch & a,
      const common::ObservationBatch & b) {
      ASSERT_EQ(a.size(), b.size());
      for (std::size_t i = 0; i < a.size(); ++i) {
        EXPECT_EQ(a[i].point_I, b[i].point_I);
        EXPECT_EQ(a[i].point_W, b[i].point_W);
        EXPECT_EQ(a[i].normal_W, b[i].normal_W);
        EXPECT_EQ(a[i].centroid_W, b[i].centroid_W);
        EXPECT_EQ(a[i].residual, b[i].residual);
        EXPECT_EQ(a[i].intensity, b[i].intensity);
        EXPECT_EQ(a[i].has_point_I, b[i].has_point_I);
        EXPECT_EQ(a[i].plane_weight, b[i].plane_weight);
      }
    };
  for (const int workers : {3, 4}) {
    frontend::ScanToMapIterationCache serial_cache, parallel_cache;
    common::Pose pose;
    for (int iteration = 0; iteration < 3; ++iteration) {
      pose.p_WI.z() = iteration * .01;
      options.num_threads = 1;
      const auto serial = frontend::buildScanToMapObservations(
        scan, pose, {}, map, options, &serial_cache);
      options.num_threads = workers;
      const auto parallel = frontend::buildScanToMapObservations(
        scan, pose, {}, map, options, &parallel_cache);
      ASSERT_GT(serial.effective_points, 0U);
      EXPECT_EQ(serial.effective_points, parallel.effective_points);
      EXPECT_EQ(serial.rejected_points, parallel.rejected_points);
      EXPECT_EQ(serial.candidate_attempts, parallel.candidate_attempts);
      EXPECT_EQ(serial.accepted_candidates, parallel.accepted_candidates);
      EXPECT_EQ(serial.knn_queries, parallel.knn_queries);
      EXPECT_EQ(serial.reused_planes, parallel.reused_planes);
      EXPECT_EQ(serial.mean_abs_residual, parallel.mean_abs_residual);
      equal_batch(serial.observations, parallel.observations);
      equal_batch(serial.insertion_candidates, parallel.insertion_candidates);
    }
  }
}
