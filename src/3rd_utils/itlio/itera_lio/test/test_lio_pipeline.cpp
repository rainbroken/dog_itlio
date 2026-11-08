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

#include <algorithm>
#include <cmath>
#include <vector>

#include "itera_lio/system/lio_pipeline.hpp"

TEST(LioPipeline, InitializesAndUpdatesMap)
{
  itera_lio::sensor::LidarFrame scan;
  scan.scan_begin_time = 0.0;
  scan.scan_end_time = 0.0;
  scan.points = {
    {{0.0, 0.0, 0.0}, 0.0F, 0.0}, {{1.0, 0.0, 0.0}, 0.0F, 0.0},
    {{0.0, 1.0, 0.0}, 0.0F, 0.0}, {{1.0, 1.0, 0.0}, 0.0F, 0.0}};
  itera_lio::system::LioPipelineOptions options;
  options.registration.knn = 3;
  options.local_map.cube_size = 20.0F;
  options.local_map.move_threshold = 5.0F;
  itera_lio::system::LioPipeline pipeline(options);
  itera_lio::estimator::NominalState state;
  itera_lio::estimator::StateCovariance covariance =
    itera_lio::estimator::StateCovariance::Identity();
  const auto first = pipeline.processScan(scan, state, covariance);
  EXPECT_TRUE(first.initialized_map);
  EXPECT_GT(first.map_size, 0U);
  EXPECT_EQ(first.global_map_size, 0U);
  const auto second = pipeline.processScan(scan, state, covariance);
  EXPECT_GT(second.registration.effective_points, 0U);
  EXPECT_EQ(second.registration.total_candidate_attempts,
    second.registration.knn_queries + second.registration.reused_planes);
  EXPECT_GE(second.registration.total_candidate_attempts,
    second.registration.candidate_attempts);
  EXPECT_LE(second.registration.knn_time_ms + second.registration.plane_fit_time_ms +
    second.registration.selection_time_ms, second.registration_time_ms);
  EXPECT_TRUE(second.degeneracy.degenerate);
  EXPECT_TRUE(std::isinf(second.degeneracy.condition_number));
  EXPECT_TRUE(second.corrected);
  EXPECT_GT(second.map_size, 0U);
  EXPECT_EQ(second.global_map_size, 0U);
  EXPECT_EQ(
    second.map_candidate_points,
    second.map_inserted_points + second.map_replaced_points + second.map_rejected_points);
  EXPECT_EQ(
    second.map_updates.size(), second.map_inserted_points + second.map_replaced_points);
}

TEST(LioPipeline, FailedUpdatePreservesLocalMap)
{
  itera_lio::sensor::LidarFrame scan;
  scan.scan_begin_time = 0.0;
  scan.scan_end_time = 0.0;
  scan.points = {
    {{-2.0, 0.0, 0.0}, 1.0F, 0.0}, {{-1.8, 0.0, 0.0}, 2.0F, 0.0},
    {{-2.0, 0.2, 0.0}, 3.0F, 0.0}, {{-1.8, 0.2, 0.0}, 4.0F, 0.0}};

  itera_lio::system::LioPipelineOptions options;
  options.scan_voxel_size = 0.05F;
  options.map_voxel_size = 0.05F;
  options.registration.knn = 3;
  options.local_map.cube_size = 10.0F;
  options.local_map.move_threshold = 2.0F;
  itera_lio::system::LioPipeline pipeline(options);
  itera_lio::estimator::NominalState state;
  itera_lio::estimator::StateCovariance covariance =
    itera_lio::estimator::StateCovariance::Identity();

  const auto initialized = pipeline.processScan(scan, state, covariance);
  ASSERT_TRUE(initialized.initialized_map);
  const std::size_t local_size_before = initialized.map_size;
  state.p_WI.x() = 6.0;
  const auto shifted = pipeline.processScan(scan, state, covariance);
  EXPECT_FALSE(shifted.corrected);
  EXPECT_FALSE(shifted.map_updated);

  std::vector<itera_lio::frontend::MapPoint> local_points;
  ASSERT_TRUE(pipeline.map().getAllPoints(local_points));
  EXPECT_EQ(shifted.map_size, local_size_before);
  EXPECT_EQ(local_points.size(), local_size_before);
  EXPECT_TRUE(
    std::any_of(
      local_points.begin(), local_points.end(),
      [](const auto & point) {return point.position.x() < 0.0F;}));
}

TEST(LioPipeline, LargeCorrectionPreservesMapButStillUpdatesState)
{
  itera_lio::sensor::LidarFrame scan;
  for (int x = -3; x <= 3; ++x) {
    for (int y = -3; y <= 3; ++y) {
      scan.points.push_back({{0.2 * x, 0.2 * y, 0.0}, 1.0F, 0.0});
    }
  }
  itera_lio::system::LioPipelineOptions options;
  options.scan_voxel_size = 0.05F;
  options.map_voxel_size = 0.05F;
  options.registration.map_insert_max_translation_correction = 1e-6;
  options.registration.enable_map_insert_quality_gate = true;
  itera_lio::system::LioPipeline pipeline(options);
  itera_lio::estimator::NominalState state;
  auto covariance = itera_lio::estimator::StateCovariance::Identity().eval();
  const auto first = pipeline.processScan(scan, state, covariance);
  ASSERT_TRUE(first.initialized_map);
  state.p_WI.z() = 0.02;
  const auto result = pipeline.processScan(scan, state, covariance);
  ASSERT_TRUE(result.corrected);
  EXPECT_GT(result.frame_translation_correction, 1e-6);
  EXPECT_LT(std::abs(state.p_WI.z()), 0.02);
  EXPECT_FALSE(result.map_updated);
  EXPECT_EQ(result.map_insert_reason, "translation_correction");
  EXPECT_EQ(result.map_size, first.map_size);
  EXPECT_EQ(result.global_map_size, first.global_map_size);
}

TEST(LioPipeline, DegradedUpdateUsesTemporaryMapAndFreezesTrustedArchive)
{
  itera_lio::sensor::LidarFrame scan;
  scan.scan_begin_time = 0.0;
  scan.scan_end_time = 0.0;
  scan.points = {
    {{0.0, 0.0, 0.0}, 0.0F, 0.0}, {{1.0, 0.0, 0.0}, 0.0F, 0.0},
    {{0.0, 1.0, 0.0}, 0.0F, 0.0}, {{1.0, 1.0, 0.0}, 0.0F, 0.0}};
  itera_lio::system::LioPipelineOptions options;
  options.registration.knn = 3;
  options.tracking.minimum_effective_points = 100U;
  options.tracking.lost_effective_points = 0U;
  options.tracking.degraded_frames = 1U;
  options.tracking.lost_frames = 100U;
  options.local_map.cube_size = 20.0F;
  options.local_map.move_threshold = 5.0F;
  itera_lio::system::LioPipeline pipeline(options);
  itera_lio::estimator::NominalState state;
  itera_lio::estimator::StateCovariance covariance =
    itera_lio::estimator::StateCovariance::Identity();
  const auto initialized = pipeline.processScan(scan, state, covariance);
  ASSERT_TRUE(initialized.initialized_map);
  const auto degraded = pipeline.processScan(scan, state, covariance);
  EXPECT_EQ(
    degraded.tracking.state, itera_lio::estimator::TrackingState::kDegraded);
  EXPECT_FALSE(degraded.map_updated);
  EXPECT_TRUE(degraded.temporary_map_updated);
  EXPECT_EQ(degraded.map_size, initialized.map_size);
  EXPECT_EQ(degraded.global_map_size, initialized.global_map_size);
  EXPECT_GT(degraded.temporary_map_size, 0U);
}

TEST(LioPipeline, DisabledLoopClosureDoesNotStoreKeyframeClouds)
{
  itera_lio::sensor::LidarFrame scan;
  scan.scan_begin_time = 0.0;
  scan.scan_end_time = 0.0;
  scan.points = {
    {{0.0, 0.0, 0.0}, 0.0F, 0.0}, {{1.0, 0.0, 0.0}, 0.0F, 0.0},
    {{0.0, 1.0, 0.0}, 0.0F, 0.0}, {{1.0, 1.0, 0.0}, 0.0F, 0.0}};
  itera_lio::system::LioPipelineOptions options;
  options.backend.enabled = true;
  options.loop_closure.enabled = false;
  options.registration.knn = 3;
  options.local_map.cube_size = 20.0F;
  options.local_map.move_threshold = 5.0F;
  itera_lio::system::LioPipeline pipeline(options);
  itera_lio::estimator::NominalState state;
  itera_lio::estimator::StateCovariance covariance =
    itera_lio::estimator::StateCovariance::Identity();

  const auto result = pipeline.processScan(scan, state, covariance);
  ASSERT_TRUE(result.backend.keyframe_added);
  EXPECT_EQ(pipeline.loopClosure().store().size(), 0U);
}

TEST(LioPipeline, BootstrapUsesOnlyLocalMap)
{
  itera_lio::sensor::LidarFrame scan;
  scan.scan_begin_time = 0.0;
  scan.scan_end_time = 0.0;
  scan.points = {
    {{0.1, 0.1, 0.1}, 1.0F, 0.0}, {{0.4, 0.4, 0.4}, 2.0F, 0.0},
    {{1.1, 0.1, 0.1}, 3.0F, 0.0}};
  itera_lio::system::LioPipelineOptions options;
  options.scan_voxel_size = 0.01F;
  options.map_voxel_size = 0.01F;
  options.registration.knn = 3;
  options.local_map.cube_size = 20.0F;
  options.local_map.move_threshold = 5.0F;
  itera_lio::system::LioPipeline pipeline(options);
  itera_lio::estimator::NominalState state;
  itera_lio::estimator::StateCovariance covariance =
    itera_lio::estimator::StateCovariance::Identity();

  const auto result = pipeline.processScan(scan, state, covariance);
  ASSERT_TRUE(result.initialized_map);
  EXPECT_EQ(result.global_map_size, 0U);
  EXPECT_TRUE(result.archive_updates.empty());
  EXPECT_EQ(result.map_updates.size(), result.map_size);
}

TEST(LioPipeline, CanDisableIntensityGlobalArchive)
{
  itera_lio::sensor::LidarFrame scan;
  scan.scan_begin_time = 0.0;
  scan.scan_end_time = 0.0;
  scan.points = {
    {{0.0, 0.0, 0.0}, 0.0F, 0.0}, {{1.0, 0.0, 0.0}, 0.0F, 0.0},
    {{0.0, 1.0, 0.0}, 0.0F, 0.0}, {{1.0, 1.0, 0.0}, 0.0F, 0.0}};
  itera_lio::system::LioPipelineOptions options;
  options.registration.knn = 3;
  options.local_map.cube_size = 20.0F;
  options.local_map.move_threshold = 5.0F;
  itera_lio::system::LioPipeline pipeline(options);
  itera_lio::estimator::NominalState state;
  itera_lio::estimator::StateCovariance covariance =
    itera_lio::estimator::StateCovariance::Identity();

  const auto result = pipeline.processScan(
    scan, state, covariance, itera_lio::common::FixedExtrinsics{}, &scan);
  ASSERT_TRUE(result.initialized_map);
  EXPECT_GT(result.map_size, 0U);
  EXPECT_EQ(result.global_map_size, 0U);
  EXPECT_EQ(result.archive_inserted_points, 0U);
  EXPECT_EQ(result.dense_archive_inserted_points, 0U);
  EXPECT_EQ(result.dense_archive_size, 0U);
  EXPECT_TRUE(result.archive_updates.empty());
  EXPECT_FALSE(result.map_updates.empty());
}
