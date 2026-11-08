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

#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "itera_lio/output/evaluation_logger.hpp"
#include "itera_lio/output/graph_export.hpp"

namespace itera_lio::output
{
namespace
{

std::filesystem::path temporaryRoot()
{
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("itera_lio_evaluation_logger_test_" + std::to_string(getpid()) + "_" +
         std::to_string(nonce));
}

std::string readFile(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

std::size_t occurrences(const std::string & text, const std::string & needle)
{
  std::size_t count = 0U;
  std::size_t position = 0U;
  while ((position = text.find(needle, position)) != std::string::npos) {
    ++count;
    position += needle.size();
  }
  return count;
}

std::size_t commaSeparatedColumns(const std::string & line)
{
  return line.empty() ? 0U : occurrences(line, ",") + 1U;
}

TEST(EvaluationLogger, LocalizationExportsKeepRawAndOptimizedFramesSeparate)
{
  const auto root=temporaryRoot();
  struct Cleanup {std::filesystem::path path;~Cleanup(){std::filesystem::remove_all(path);}} cleanup{root};
  backend::Keyframe frame;frame.id=0;frame.timestamp=1;
  frame.pose_WI.p_WI=Eigen::Vector3d(1,2,3);
  auto raw=writeTrajectoryCsv({frame},root.string(),"localization_keyframes");
  ASSERT_TRUE(raw.success)<<raw.error;
  frame.pose_WI.p_WI=Eigen::Vector3d(11,12,13);
  auto optimized=writeTrajectoryCsv({frame},root.string(),"localization_optimized_keyframes");
  ASSERT_TRUE(optimized.success)<<optimized.error;
  EXPECT_EQ(std::filesystem::path(raw.output_path).filename(),"localization_keyframes.csv");
  EXPECT_EQ(std::filesystem::path(optimized.output_path).filename(),"localization_optimized_keyframes.csv");
  EXPECT_NE(readFile(raw.output_path),readFile(optimized.output_path));
  EXPECT_NE(readFile(raw.output_path).find("0,1,1,2,3,"),std::string::npos);
}

TEST(EvaluationLogger, HonorsOutputSwitchesAndFinalizesSummary)
{
  const auto root = temporaryRoot();
  EvaluationOutputOptions options;
  options.frames = false;
  options.runtime = true;
  options.events = true;
  options.loop_verifications = false;
  options.summary = true;
  options.odometry = false;
  options.parameters = false;
  options.manifest = false;
  options.warmup_sec = 0.0;

  EvaluationLogger logger;
  std::string error;
  ASSERT_TRUE(logger.initialize(root.string(), {}, options, {}, error)) << error;
  const auto run_directory = std::filesystem::path(logger.runDirectory());
  EXPECT_EQ(run_directory, std::filesystem::absolute(root));
  EXPECT_FALSE(std::filesystem::exists(run_directory / "frames.csv"));
  EXPECT_FALSE(std::filesystem::exists(run_directory / "summary.json"));

  common::RuntimeSnapshot first;
  first.wall_timestamp = 10.0;
  first.sensor_timestamp = 5.0;
  first.lidar_received = 10U;
  first.imu_received = 100U;
  first.lidar_capacity_drops = 2U;
  logger.writeRuntime(first);

  common::RuntimeSnapshot second = first;
  second.wall_timestamp = 15.0;
  second.sensor_timestamp = 10.0;
  second.local_map_points = 123U;
  second.backend_keyframes = 4U;
  second.peak_resident_memory_mib = 42.0;
  logger.writeRuntime(second);
  logger.recordRevisitRefinement(9U, 4U, 2U);
  logger.finalize(second);

  EXPECT_TRUE(std::filesystem::exists(run_directory / "runtime.csv"));
  EXPECT_TRUE(std::filesystem::exists(run_directory / "events.csv"));
  EXPECT_TRUE(std::filesystem::exists(run_directory / "summary.json"));
  EXPECT_EQ(
    occurrences(readFile(run_directory / "events.csv"), "lidar_capacity_drop"), 1U);
  const auto summary = readFile(run_directory / "summary.json");
  EXPECT_NE(summary.find("\"normal_exit\": true"), std::string::npos);
  EXPECT_NE(summary.find("\"final_local_points\": 123"), std::string::npos);
  EXPECT_NE(summary.find("\"optimization_runs\": 2"), std::string::npos);
  EXPECT_NE(summary.find("\"revisit_refinement\": {\"calls\": 1, \"verifications\": 9, \"accepted\": 4, \"committed\": 2}"), std::string::npos);
  EXPECT_NE(summary.find("\"p95\""), std::string::npos);

  std::filesystem::remove_all(root);
}

TEST(EvaluationLogger, WritesFrameLoopAndRebuildEvents)
{
  const auto root = temporaryRoot();
  EvaluationOutputOptions options;
  options.detailed = true;
  options.runtime = false;
  options.odometry = false;
  options.parameters = false;
  options.manifest = false;
  options.warmup_sec = 0.0;

  EvaluationLogger logger;
  std::string error;
  ASSERT_TRUE(logger.initialize(root.string(), {}, options, {}, error)) << error;
  const auto run_directory = std::filesystem::path(logger.runDirectory());

  sensor::LidarFrame scan;
  scan.scan_end_time = 100.0;
  system::PipelineResult result;
  result.map_updated = true;
  result.map_size = 10U;
  result.loop_candidates = 1U;
  result.loop_verifications = 1U;
  result.loop_rejected_verifications = 1U;
  result.loop_icp_initialization = "odometry";
  result.loop_rejection_reason = "fitness_threshold";
  loop::LoopVerificationResult first_verification;
  first_verification.current_id = 10U;
  first_verification.matched_id = 2U;
  first_verification.descriptor_distance = 0.1;
  first_verification.odometry_distance = 12.0;
  first_verification.odometry_vertical_distance = 0.2;
  first_verification.gravity_angle_rad = 0.03;
  first_verification.gravity_residual_rad = 0.01;
  first_verification.vertical_separation = 0.15;
  first_verification.global_correction.p_WI = Eigen::Vector3d(1.0, 2.0, 3.0);
  first_verification.initialization = "odometry";
  first_verification.rejection_reason = "fitness_threshold";
  loop::LoopVerificationResult second_verification = first_verification;
  second_verification.matched_id = 3U;
  second_verification.initialization = "descriptor_centroid";
  second_verification.rejection_reason = "insufficient_overlap";
  result.loop_verification_results = {first_verification, second_verification};
  result.loop_verifications = result.loop_verification_results.size();
  result.loop_rejected_verifications = result.loop_verification_results.size();
  result.ikd_rebuild.total_count = 1U;
  result.ikd_rebuild.last_subtree_size = 25U;
  result.ikd_rebuild.last_duration_ms = 3.5;
  common::FrameProcessingTiming processing;
  processing.core_end_to_end_time_ms = 12.0;
  common::FrameOutputTiming output;
  output.end_to_end_time_ms = 13.0;
  output.callback_to_output_ms = 15.0;
  output.callback_to_state_publish_ms = 14.0;
  logger.writeFrame(20U, scan, result, processing, output);

  common::RuntimeSnapshot final_snapshot;
  final_snapshot.wall_timestamp = 20.0;
  final_snapshot.sensor_timestamp = scan.scan_end_time;
  final_snapshot.local_map_points = result.map_size;
  logger.finalize(final_snapshot);

  const auto frames = readFile(run_directory / "frames.csv");
  EXPECT_NE(frames.find("loop_constraint_added"), std::string::npos);
  EXPECT_NE(frames.find("callback_to_state_publish_ms"), std::string::npos);
  EXPECT_NE(frames.find("state_publish_to_output_end_ms"), std::string::npos);
  EXPECT_NE(frames.find("loop_icp_initialization"), std::string::npos);
  EXPECT_NE(frames.find("odometry"), std::string::npos);
  EXPECT_EQ(frames.find("resident_memory_mib"), std::string::npos);
  const auto newline = frames.find('\n');
  ASSERT_NE(newline, std::string::npos);
  const auto second_newline = frames.find('\n', newline + 1U);
  ASSERT_NE(second_newline, std::string::npos);
  EXPECT_EQ(
    commaSeparatedColumns(frames.substr(0U, newline)),
    commaSeparatedColumns(frames.substr(newline + 1U, second_newline - newline - 1U)));
  const auto events = readFile(run_directory / "events.csv");
  EXPECT_EQ(occurrences(events, "loop_rejected"), 2U);
  EXPECT_NE(events.find("map_rebuild"), std::string::npos);
  const auto loop_verifications = readFile(run_directory / "loop_verifications.csv");
  EXPECT_EQ(occurrences(loop_verifications, "\n"), 3U);
  EXPECT_NE(loop_verifications.find("descriptor_distance"), std::string::npos);
  EXPECT_NE(loop_verifications.find("odometry_vertical_distance"), std::string::npos);
  EXPECT_NE(loop_verifications.find("gravity_residual_rad"), std::string::npos);
  EXPECT_NE(loop_verifications.find("vertical_separation"), std::string::npos);
  EXPECT_NE(loop_verifications.find("correction_px"), std::string::npos);
  EXPECT_NE(loop_verifications.find(",1,2,3,0,0,0,1,0,0,0,0,0,0,0,0,0,0\n"), std::string::npos);
  EXPECT_NE(loop_verifications.find("audit_wall_p90"), std::string::npos);
  EXPECT_NE(loop_verifications.find("audit_horizontal_p90"), std::string::npos);
  EXPECT_NE(loop_verifications.find("descriptor_centroid"), std::string::npos);
  const auto summary = readFile(run_directory / "summary.json");
  EXPECT_NE(summary.find("\"accepted\": 0"), std::string::npos);
  EXPECT_NE(summary.find("\"committed\": 0"), std::string::npos);
  EXPECT_NE(summary.find("\"rejected\": 2"), std::string::npos);

  std::filesystem::remove_all(root);
}

TEST(EvaluationLogger, CompactAndDetailedSchemasPreserveValuesAndCriticalEvents)
{
  for (const bool detailed : {false, true}) {
    const char * fixtures = std::getenv("ITERA_EVAL_FIXTURES");
    const auto root = fixtures ? std::filesystem::path(fixtures) /
      (detailed ? "detailed" : "compact") : temporaryRoot();
    EvaluationOutputOptions options;
    options.detailed = detailed;
    options.warmup_sec = 0.0;
    EvaluationLogger logger;
    std::string error;
    ASSERT_TRUE(logger.initialize(root.string(), {}, options, {}, error)) << error;
    sensor::LidarFrame scan;
    scan.scan_end_time = 100.0;
    system::PipelineResult result;
    result.correction.observation_update.accepted_observations.resize(3);
    result.correction.mean_abs_residual_after = 0.125;
    result.registration_time_ms = 7.0;
    result.map_insert_reason = "tracking";
    result.map_size = 123;
    result.ikd_rebuild.total_count = 1;
    result.ikd_rebuild.last_duration_ms = 5.0;
    result.tracking.transitioned = true;
    result.backend.loop_constraint_added = true;
    loop::LoopVerificationResult verification;
    verification.rejection_reason = "fitness_too_high";
    result.loop_verification_results.push_back(verification);
    result.loop_rejected_verifications = 1;
    system::FrameProcessingTiming processing;
    processing.core_end_to_end_time_ms = 11.0;
    FrameOutputTiming output;
    output.end_to_end_time_ms = 22.0;
    logger.writeFrame(456, scan, result, processing, output);
    result.ikd_rebuild.total_count = 2;
    result.ikd_rebuild.last_duration_ms = 150.0;
    scan.scan_end_time = 101.0;
    logger.writeFrame(456, scan, result, processing, output);
    estimator::NominalState state;
    state.p_WI = Eigen::Vector3d(1, 2, 3);
    state.v_WI.x() = 12;
    state.g_W = Eigen::Vector3d(0.1, -0.2, -9.8);
    state.b_g = Eigen::Vector3d(0.001, 0.002, 0.003);
    state.b_a = Eigen::Vector3d(0.01, 0.02, 0.03);
    const estimator::ErrorStateVector axis = estimator::ErrorStateVector::LinSpaced(1, 18);
    const estimator::StateCovariance covariance =
      axis * axis.transpose() + estimator::StateCovariance::Identity();
    logger.writeOdometry(100.0, state, covariance.diagonal());
    common::RuntimeSnapshot snapshot;
    snapshot.wall_timestamp = 110.0;
    snapshot.sensor_timestamp = 101.0;
    snapshot.lidar_capacity_drops = 2;
    snapshot.loop_worker_timed_out = 3;
    logger.writeRuntime(snapshot);
    logger.finalize(snapshot);

    for (const auto & file : {"frames.csv", "runtime.csv", "odom.csv", "loop_verifications.csv"}) {
      std::istringstream stream(readFile(root / file));
      std::string header, line;
      ASSERT_TRUE(static_cast<bool>(std::getline(stream, header)));
      const auto columns = commaSeparatedColumns(header);
      const std::string name(file);
      if (name == "frames.csv") {EXPECT_EQ(columns, detailed ? 126U : 30U);}
      if (name == "runtime.csv") {EXPECT_EQ(columns, detailed ? 67U : 32U);}
      if (name == "odom.csv") {EXPECT_EQ(columns, 38U);}
      while (std::getline(stream, line)) {EXPECT_EQ(commaSeparatedColumns(line), columns) << file;}
    }
    const auto frames = readFile(root / "frames.csv");
    EXPECT_NE(frames.find("mean_residual_after"), std::string::npos);
    EXPECT_EQ(frames.find("whitened_eig0") != std::string::npos, detailed);
    std::istringstream odometry(readFile(root / "odom.csv"));
    std::string odom_header, odom_row;
    ASSERT_TRUE(static_cast<bool>(std::getline(odometry, odom_header)));
    ASSERT_TRUE(static_cast<bool>(std::getline(odometry, odom_row)));
    std::istringstream names(odom_header), values(odom_row);
    std::map<std::string, double> recorded;
    std::string name, value;
    while (std::getline(names, name, ',')) {
      ASSERT_TRUE(static_cast<bool>(std::getline(values, value, ',')));
      recorded.emplace(name, std::stod(value));
    }
    EXPECT_DOUBLE_EQ(recorded.at("timestamp"), 100.0);
    EXPECT_DOUBLE_EQ(recorded.at("px"), 1.0);
    EXPECT_DOUBLE_EQ(recorded.at("vx"), 12.0);
    EXPECT_DOUBLE_EQ(recorded.at("gz"), -9.8);
    EXPECT_DOUBLE_EQ(recorded.at("bgy"), 0.002);
    EXPECT_DOUBLE_EQ(recorded.at("baz"), 0.03);
    for (int row = 0; row < estimator::ErrorStateLayout::kDimension; ++row) {
      EXPECT_DOUBLE_EQ(recorded.at("cov_diag_" + std::to_string(row)), covariance(row, row));
    }
    const auto events = readFile(root / "events.csv");
    EXPECT_EQ(occurrences(events, "loop_rejected"), detailed ? 2U : 0U);
    EXPECT_EQ(occurrences(events, "loop_committed"), 2U);
    EXPECT_EQ(occurrences(events, "tracking_transition"), 2U);
    EXPECT_EQ(occurrences(events, "lidar_capacity_drop"), 1U);
    EXPECT_EQ(occurrences(events, "slow_map_rebuild"), detailed ? 0U : 1U);
    EXPECT_NE(readFile(root / "summary.json").find("\"rejected\": 2"), std::string::npos);
    EXPECT_NE(readFile(root / "manifest.json").find("\"schema_version\": 4"), std::string::npos);
    if (!fixtures) {std::filesystem::remove_all(root);}
  }
}

TEST(EvaluationLogger, CompactModeRetainsVerificationEventsWithoutDetailedRows)
{
  for (const bool verification_csv : {false, true}) {
    const auto root = temporaryRoot();
    EvaluationOutputOptions options;
    options.loop_verifications = verification_csv;
    EvaluationLogger logger;
    std::string error;
    ASSERT_TRUE(logger.initialize(root.string(), {}, options, {}, error)) << error;
    sensor::LidarFrame scan;
    system::PipelineResult result;
    result.loop_rejected_verifications = 1;
    result.loop_rejection_reason = "missing_geometry";
    logger.writeFrame(0, scan, result, {}, {});
    logger.finalize({});
    EXPECT_NE(readFile(root / "events.csv").find("missing_geometry"), std::string::npos);
    std::filesystem::remove_all(root);
  }
}

}  // namespace
}  // namespace itera_lio::output
