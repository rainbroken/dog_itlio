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

#include <cmath>
#include <stdexcept>

#include "itera_lio/common/so3.hpp"
#include "itera_lio/system/lio_processor.hpp"

namespace
{

itera_lio::sensor::BufferedMeasurement imuMeasurement(
  const double timestamp, const double gyro_z = 0.0)
{
  itera_lio::sensor::BufferedMeasurement measurement;
  measurement.type = itera_lio::sensor::MeasurementType::kImu;
  measurement.imu.timestamp = timestamp;
  measurement.imu.gyro = Eigen::Vector3d(0.0, 0.0, gyro_z);
  measurement.imu.accel = Eigen::Vector3d(0.0, 0.0, 9.81);
  return measurement;
}

itera_lio::system::LioProcessorOptions testOptions()
{
  itera_lio::system::LioProcessorOptions options;
  options.imu_initialization.duration_sec = 0.1;
  options.imu_initialization.min_samples = 2U;
  options.imu_initialization.gyro_std_threshold = 1e-9;
  options.imu_initialization.accel_std_threshold = 1e-9;
  options.max_imu_interval_sec = 0.2;
  options.max_pending_lidar = 1U;
  return options;
}

TEST(LioProcessor, RejectsInvalidOptions)
{
  auto options = testOptions();
  options.imu_history_max_samples = 1U;
  EXPECT_THROW(itera_lio::system::LioProcessor processor(options), std::invalid_argument);
}

TEST(LioProcessor, InitializesOnceThenPropagates)
{
  itera_lio::system::LioProcessor processor(testOptions());

  EXPECT_FALSE(processor.process(imuMeasurement(0.0)).initialized_now);
  const auto initialized = processor.process(imuMeasurement(0.1));
  EXPECT_TRUE(initialized.initialized_now);
  EXPECT_TRUE(processor.state().R_WI.isIdentity(1e-12));

  const auto propagated = processor.process(imuMeasurement(0.2, 1.0));
  EXPECT_FALSE(propagated.initialized_now);
  EXPECT_FALSE(propagated.propagation_failed);
  EXPECT_FALSE(processor.state().R_WI.isIdentity(1e-6));
  EXPECT_EQ(processor.stats().imu_processed, 3U);
}

TEST(LioProcessor, StaticCovariancePreservesFreeGravityBiasAmbiguity)
{
  using namespace itera_lio;
  auto options = testOptions();
  options.imu_initialization.initialize_accel_bias_from_static_mean = false;
  options.imu_initialization.accel_norm_threshold = 0.0;
  options.initialize_covariance_from_static_window = true;
  for (double magnitude : {3.2, 9.7, 15.0}) {
    system::LioProcessor processor(options);
    auto first = imuMeasurement(0.0), second = imuMeasurement(0.1);
    first.imu.accel = second.imu.accel = Eigen::Vector3d(0.3, -0.7, 1.0).normalized() * magnitude;
    processor.process(first);
    ASSERT_TRUE(processor.process(second).initialized_now);
    const auto & state = processor.state();
    const auto & covariance = processor.covariance();
    EXPECT_TRUE(estimator::isValidCovariance(covariance));
    EXPECT_NEAR(state.g_W.norm(), magnitude, 1e-10);
    Eigen::Matrix<double, 3, 18> dynamics = Eigen::Matrix<double, 3, 18>::Zero();
    dynamics.block<3, 3>(0, 0) = -state.R_WI * common::Hat(first.imu.accel);
    dynamics.block<3, 3>(0, 12) = -state.R_WI;
    dynamics.block<3, 3>(0, 15).setIdentity();
    const Eigen::Matrix3d acceleration_covariance = dynamics * covariance * dynamics.transpose();
    EXPECT_TRUE(acceleration_covariance.isApprox(Eigen::Matrix3d::Identity() *
      options.imu_noise.accel_noise_density * options.imu_noise.accel_noise_density / 0.1, 1e-10));
    EXPECT_GT((covariance.block<3, 3>(12, 15).norm()), 0.01);
    EXPECT_NEAR(covariance(0, 0), 1e-6, 1e-12);
  }
}

TEST(LioProcessor, KeepsNewestPendingLidarFrame)
{
  itera_lio::system::LioProcessor processor(testOptions());
  ASSERT_FALSE(processor.process(imuMeasurement(0.0)).initialized_now);
  ASSERT_TRUE(processor.process(imuMeasurement(0.1)).initialized_now);
  itera_lio::sensor::BufferedMeasurement lidar;
  lidar.type = itera_lio::sensor::MeasurementType::kLidar;
  lidar.lidar.scan_begin_time = 1.0;
  lidar.lidar.scan_end_time = 1.1;
  lidar.lidar.points.push_back({{1.0, 0.0, 0.0}, 0.0F, 0.0});

  EXPECT_FALSE(processor.process(lidar).frame_ready);
  lidar.lidar.scan_begin_time = 2.0;
  lidar.lidar.scan_end_time = 2.1;
  EXPECT_FALSE(processor.process(lidar).frame_ready);
  EXPECT_EQ(processor.stats().lidar_processed, 2U);
  EXPECT_EQ(processor.stats().lidar_pending_drops, 1U);
}

TEST(LioProcessor, ReanchorsInsteadOfPropagatingAcrossImuGap)
{
  itera_lio::system::LioProcessor processor(testOptions());
  ASSERT_TRUE(
    processor.process(imuMeasurement(0.0)).initialization_status ==
    itera_lio::estimator::ImuInitializationStatus::kCollecting);
  ASSERT_TRUE(processor.process(imuMeasurement(0.1)).initialized_now);
  ASSERT_FALSE(processor.process(imuMeasurement(0.2, 1.0)).propagation_failed);
  const Eigen::Matrix3d rotation_before_gap = processor.state().R_WI;

  const auto gap_update = processor.process(imuMeasurement(1.0, 1.0));
  EXPECT_FALSE(gap_update.propagation_failed);
  EXPECT_EQ(processor.stats().imu_gap_resets, 1U);
  EXPECT_TRUE(processor.state().R_WI.isApprox(rotation_before_gap, 1e-12));
  EXPECT_TRUE(processor.state().v_WI.isZero(1e-12));
}

TEST(LioProcessor, CorrectsAtScanEndThenPropagatesOnlyRemainingImuInterval)
{
  auto options = testOptions();
  options.lidar_min_range = 0.0;
  options.deskew_voxel_size = 0.01F;
  options.pipeline.scan_voxel_size = 0.01F;
  options.pipeline.input_prevoxelized = true;
  options.pipeline.map_voxel_size = 0.05F;
  options.pipeline.registration.knn = 3;
  options.pipeline.local_map.cube_size = 20.0F;
  options.pipeline.local_map.move_threshold = 5.0F;
  itera_lio::system::LioProcessor processor(options);

  ASSERT_FALSE(processor.process(imuMeasurement(0.0)).initialized_now);
  ASSERT_TRUE(processor.process(imuMeasurement(0.1)).initialized_now);
  ASSERT_FALSE(processor.process(imuMeasurement(0.2, 1.0)).propagation_failed);

  itera_lio::sensor::BufferedMeasurement lidar;
  lidar.type = itera_lio::sensor::MeasurementType::kLidar;
  lidar.lidar.scan_begin_time = 0.2;
  lidar.lidar.scan_end_time = 0.25;
  lidar.lidar.points = {
    {{1.0, 0.0, 0.0}, 1.0F, 0.00},
    {{1.0, 1.0, 0.0}, 2.0F, 0.02},
    {{2.0, 0.0, 0.0}, 3.0F, 0.05}};
  ASSERT_FALSE(processor.process(std::move(lidar)).frame_ready);

  auto expected_state = processor.state();
  auto expected_covariance = processor.covariance();
  itera_lio::estimator::ImuPropagator propagator(options.imu_noise);
  ASSERT_TRUE(propagator.propagate(
    imuMeasurement(0.2, 1.0).imu, imuMeasurement(0.25, 1.0).imu,
    expected_state, expected_covariance));
  const auto update = processor.process(imuMeasurement(0.3, 1.0));
  ASSERT_TRUE(update.frame_ready);
  ASSERT_FALSE(update.frames.empty());
  const auto & frame = update.frames.front();
  EXPECT_TRUE(frame.pipeline.initialized_map);
  EXPECT_TRUE(
    frame.state_at_scan_end.R_WI.isApprox(
      itera_lio::common::ExpSO3(Eigen::Vector3d(0.0, 0.0, 0.10)), 1e-9));
  EXPECT_TRUE(
    processor.state().R_WI.isApprox(
      itera_lio::common::ExpSO3(Eigen::Vector3d(0.0, 0.0, 0.15)), 1e-9));
  EXPECT_EQ(frame.timing.repropagated_imu_samples, 1U);
  EXPECT_TRUE(frame.covariance_diagonal_at_scan_end.allFinite());
  EXPECT_TRUE((frame.covariance_diagonal_at_scan_end.array() >= 0.0).all());
  EXPECT_TRUE(frame.covariance_diagonal_at_scan_end.isApprox(
    expected_covariance.diagonal(), 1e-12));
  EXPECT_FALSE(frame.covariance_diagonal_at_scan_end.isApprox(
    processor.covariance().diagonal(), 1e-12));
  EXPECT_NEAR(frame.timing.post_scan_imu_duration_ms, 50.0, 1e-9);
}

TEST(LioProcessor, EarlyAndDelayedLidarDeliveryProduceExactScanEndState)
{
  auto options = testOptions();
  options.lidar_min_range = 0.0;
  options.deskew_voxel_size = 0.01F;
  options.pipeline.scan_voxel_size = 0.01F;
  options.pipeline.input_prevoxelized = true;
  options.pipeline.map_voxel_size = 0.05F;
  options.pipeline.registration.knn = 3;
  options.pipeline.local_map.cube_size = 20.0F;
  options.pipeline.local_map.move_threshold = 5.0F;
  itera_lio::system::LioProcessor early(options), delayed(options);
  for (auto * processor : {&early, &delayed}) {
    ASSERT_FALSE(processor->process(imuMeasurement(0.0)).initialized_now);
    ASSERT_TRUE(processor->process(imuMeasurement(0.1)).initialized_now);
    ASSERT_FALSE(processor->process(imuMeasurement(0.2, 1.0)).propagation_failed);
  }
  itera_lio::sensor::BufferedMeasurement lidar;
  lidar.type = itera_lio::sensor::MeasurementType::kLidar;
  lidar.lidar.scan_begin_time = 0.2;
  lidar.lidar.scan_end_time = 0.25;
  lidar.lidar.points = {
    {{1., 0., 0.}, 1.F, 0.}, {{1., 1., 0.}, 2.F, .02},
    {{2., 0., 0.}, 3.F, .05}};
  ASSERT_FALSE(early.process(lidar).frame_ready);
  // A frame must remain pending until the actual scan-end bracket exists.
  ASSERT_FALSE(early.process(imuMeasurement(.24, 1.)).frame_ready);
  ASSERT_FALSE(delayed.process(imuMeasurement(.24, 1.)).frame_ready);
  const auto immediate = early.process(imuMeasurement(.26, 1.));
  ASSERT_TRUE(immediate.frame_ready);
  ASSERT_FALSE(delayed.process(imuMeasurement(.26, 1.)).frame_ready);
  ASSERT_FALSE(early.process(imuMeasurement(.27, 1.)).frame_ready);
  ASSERT_FALSE(delayed.process(imuMeasurement(.27, 1.)).frame_ready);
  const auto later = delayed.process(lidar);
  ASSERT_TRUE(later.frame_ready);
  const auto & a = immediate.frames.front();
  const auto & b = later.frames.front();
  EXPECT_EQ(a.state_at_scan_end.R_WI, b.state_at_scan_end.R_WI);
  EXPECT_EQ(a.state_at_scan_end.p_WI, b.state_at_scan_end.p_WI);
  EXPECT_EQ(a.covariance_diagonal_at_scan_end, b.covariance_diagonal_at_scan_end);
  ASSERT_EQ(a.deskewed.points.size(), b.deskewed.points.size());
  for (std::size_t i = 0; i < a.deskewed.points.size(); ++i) {
    EXPECT_EQ(a.deskewed.points[i].position, b.deskewed.points[i].position);
  }
  EXPECT_EQ(early.state().R_WI, delayed.state().R_WI);
  EXPECT_EQ(early.state().p_WI, delayed.state().p_WI);
  EXPECT_EQ(early.covariance(), delayed.covariance());
}

TEST(LioProcessor, BoundsImuTrajectoryByDuration)
{
  auto options = testOptions();
  options.imu_history_duration_sec = 0.2;
  options.imu_history_max_samples = 1000U;
  itera_lio::system::LioProcessor processor(options);

  ASSERT_FALSE(processor.process(imuMeasurement(0.0)).initialized_now);
  ASSERT_TRUE(processor.process(imuMeasurement(0.1)).initialized_now);
  for (int index = 2; index <= 20; ++index) {
    ASSERT_FALSE(processor.process(imuMeasurement(0.1 * index)).propagation_failed);
  }
  EXPECT_LE(processor.stats().imu_history_samples, 4U);
}

}  // namespace
