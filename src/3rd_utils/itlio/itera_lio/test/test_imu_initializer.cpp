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

#include <cmath>
#include <limits>
#include <stdexcept>

#include "gtest/gtest.h"
#include "itera_lio/estimator/imu_initializer.hpp"

namespace
{

using itera_lio::estimator::ImuInitializationStatus;
using itera_lio::estimator::ImuInitializer;
using itera_lio::estimator::ImuInitializerOptions;

itera_lio::sensor::ImuData sample(double timestamp, double gyro_z = 0.02)
{
  itera_lio::sensor::ImuData value;
  value.timestamp = timestamp;
  value.gyro = Eigen::Vector3d(0.0, 0.0, gyro_z);
  value.accel = Eigen::Vector3d(0.0, 0.0, 9.81);
  return value;
}

TEST(ImuInitializer, RejectsInvalidOptions)
{
  ImuInitializerOptions options;
  options.duration_sec = 0.0;
  const auto construct = [&options]() {ImuInitializer initializer(options);};
  EXPECT_THROW(construct(), std::invalid_argument);
}

TEST(ImuData, AppliesLidarToImuTimeOffsetWithFastLioSign)
{
  itera_lio::sensor::ImuData sample;
  sample.timestamp = 10.0;
  const auto aligned = itera_lio::sensor::applyTimeOffset(sample, 0.1);
  EXPECT_DOUBLE_EQ(aligned.timestamp, 9.9);
  EXPECT_DOUBLE_EQ(sample.timestamp, 10.0);
}

TEST(ImuInitializer, InitializesStationaryWindow)
{
  ImuInitializerOptions options;
  options.duration_sec = 1.0;
  options.min_samples = 3U;
  options.gyro_std_threshold = 1e-6;
  options.accel_std_threshold = 1e-6;
  ImuInitializer initializer(options);

  EXPECT_EQ(initializer.addSample(sample(0.0)), ImuInitializationStatus::kCollecting);
  EXPECT_EQ(initializer.addSample(sample(0.5)), ImuInitializationStatus::kCollecting);
  EXPECT_EQ(initializer.addSample(sample(1.0)), ImuInitializationStatus::kInitialized);
  ASSERT_TRUE(initializer.isInitialized());
  EXPECT_NEAR(initializer.initializedState().b_g.z(), 0.02, 1e-12);
  EXPECT_TRUE(initializer.initializedState().b_a.isZero(1e-12));
  EXPECT_NEAR(initializer.initializedState().g_W.z(), -9.81, 1e-12);
  EXPECT_TRUE(initializer.initializedState().R_WI.isIdentity());
  EXPECT_EQ(initializer.stats().sample_count, 3U);
}

TEST(ImuInitializer, InitializesStaticAccelerationBiasFromGravityMagnitude)
{
  ImuInitializerOptions options;
  options.duration_sec = 1.0;
  options.min_samples = 3U;
  options.gyro_std_threshold = 1e-6;
  options.accel_std_threshold = 1e-6;
  options.gravity_magnitude = 9.81;
  ImuInitializer initializer(options);

  for (const double timestamp : {0.0, 0.5, 1.0}) {
    auto value = sample(timestamp, 0.0);
    value.accel = Eigen::Vector3d(0.0, 0.0, 9.61);
    ASSERT_EQ(
      initializer.addSample(value),
      timestamp < 1.0 ? ImuInitializationStatus::kCollecting :
      ImuInitializationStatus::kInitialized);
  }

  const auto & state = initializer.initializedState();
  EXPECT_TRUE(state.b_a.isApprox(Eigen::Vector3d(0.0, 0.0, -0.2), 1e-12));
  EXPECT_TRUE(
    (state.R_WI * (initializer.stats().mean_accel - state.b_a) +
    state.g_W).isZero(1e-12));
}

TEST(ImuInitializer, AlignsWorldZWithMeasuredGravity)
{
  ImuInitializerOptions options;
  options.duration_sec = 1.0;
  options.min_samples = 3U;
  options.gyro_std_threshold = 1e-6;
  options.accel_std_threshold = 1e-6;
  ImuInitializer initializer(options);

  for (const double timestamp : {0.0, 0.5, 1.0}) {
    itera_lio::sensor::ImuData value;
    value.timestamp = timestamp;
    value.accel = Eigen::Vector3d(0.0, 9.81, 0.0);
    ASSERT_EQ(
      initializer.addSample(value),
      timestamp < 1.0 ? ImuInitializationStatus::kCollecting :
      ImuInitializationStatus::kInitialized);
  }

  const auto & state = initializer.initializedState();
  const Eigen::Vector3d measured_up = initializer.stats().mean_accel.normalized();
  EXPECT_TRUE((state.R_WI * measured_up).isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_TRUE(state.g_W.isApprox(Eigen::Vector3d(0.0, 0.0, -9.81), 1e-12));
  EXPECT_FALSE(state.R_WI.isIdentity(1e-12));
}

TEST(ImuInitializer, FreeMagnitudeUsesMeasuredMeanAndBalancesStationaryDynamics)
{
  for (const double magnitude : {1.6, 9.4, 11.2}) {
    ImuInitializerOptions options;
    options.duration_sec = 1.0;
    options.min_samples = 3U;
    options.accel_norm_threshold = 0.0;
    options.initialize_accel_bias_from_static_mean = false;
    ImuInitializer initializer(options);
    const Eigen::Vector3d acceleration =
      magnitude * Eigen::Vector3d(0.3, -0.4, 0.5).normalized();
    for (const double timestamp : {0.0, 0.5, 1.0}) {
      auto value = sample(timestamp, 0.0);
      value.accel = acceleration;
      initializer.addSample(value);
    }
    ASSERT_TRUE(initializer.isInitialized());
    const auto & state = initializer.initializedState();
    EXPECT_TRUE(state.b_a.isZero());
    EXPECT_NEAR(state.g_W.norm(), magnitude, 1e-12);
    EXPECT_TRUE(state.g_W.isApprox(Eigen::Vector3d(0.0, 0.0, -magnitude), 1e-12));
    EXPECT_TRUE((state.R_WI * acceleration + state.g_W).isZero(1e-12));
    EXPECT_GT(std::abs(state.g_W.norm() - options.gravity_magnitude), 0.1);
  }
}

TEST(ImuInitializer, FreeMagnitudeStillRequiresAStableNonzeroAccelerationWindow)
{
  ImuInitializerOptions options;
  options.duration_sec = 1.0;
  options.min_samples = 3U;
  options.accel_norm_threshold = 0.0;
  options.initialize_accel_bias_from_static_mean = false;
  options.accel_std_threshold = 0.01;
  ImuInitializer initializer(options);
  for (const double timestamp : {0.0, 0.5, 1.0}) {
    auto value = sample(timestamp, 0.0);
    value.accel.setZero();
    initializer.addSample(value);
  }
  EXPECT_FALSE(initializer.isInitialized());
  initializer.reset();
  for (int i = 0; i < 10; ++i) {
    auto value = sample(0.5 * i, 0.0);
    value.accel.z() = i % 2 == 0 ? 9.4 : 9.8;
    initializer.addSample(value);
  }
  EXPECT_FALSE(initializer.isInitialized());
}

TEST(ImuInitializer, ResetsWindowWhenMotionDetected)
{
  ImuInitializerOptions options;
  options.duration_sec = 1.0;
  options.min_samples = 3U;
  options.gyro_std_threshold = 1e-3;
  options.accel_std_threshold = 1e-3;
  ImuInitializer initializer(options);

  EXPECT_EQ(initializer.addSample(sample(0.0)), ImuInitializationStatus::kCollecting);
  EXPECT_EQ(initializer.addSample(sample(0.5, 1.0)), ImuInitializationStatus::kCollecting);
  EXPECT_EQ(initializer.stats().sample_count, 1U);
  EXPECT_EQ(initializer.addSample(sample(1.0)), ImuInitializationStatus::kCollecting);
}

TEST(ImuInitializer, RejectsConstantLinearAccelerationAsStatic)
{
  ImuInitializerOptions options;
  options.duration_sec = 1.0;
  options.min_samples = 3U;
  options.gyro_std_threshold = 1e-6;
  options.accel_std_threshold = 1e-6;
  options.accel_norm_threshold = 0.05;
  ImuInitializer initializer(options);

  for (const double timestamp : {0.0, 0.5, 1.0, 1.5}) {
    auto value = sample(timestamp, 0.0);
    value.accel = Eigen::Vector3d(0.0, 0.0, 10.5);
    EXPECT_EQ(initializer.addSample(value), ImuInitializationStatus::kCollecting);
  }
  EXPECT_FALSE(initializer.isInitialized());
}

TEST(ImuInitializer, RejectsNonzeroMeanGyroDespiteLowVariance)
{
  ImuInitializerOptions options;
  options.duration_sec = 1.0;
  options.min_samples = 3U;
  options.gyro_std_threshold = 1e-6;
  options.accel_std_threshold = 1e-6;
  options.gyro_mean_threshold = 0.01;
  ImuInitializer initializer(options);

  for (const double timestamp : {0.0, 0.5, 1.0, 1.5}) {
    EXPECT_EQ(initializer.addSample(sample(timestamp, 0.02)),
      ImuInitializationStatus::kCollecting);
  }
  EXPECT_FALSE(initializer.isInitialized());
}

TEST(ImuInitializer, RejectsTimestampRegressionAndNonFiniteSample)
{
  ImuInitializer initializer;
  EXPECT_EQ(initializer.addSample(sample(2.0)), ImuInitializationStatus::kCollecting);
  EXPECT_EQ(
    initializer.addSample(sample(1.0)),
    ImuInitializationStatus::kRejectedTimestampRegression);
  auto invalid = sample(3.0);
  invalid.accel.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(initializer.addSample(invalid), ImuInitializationStatus::kRejectedInvalidSample);
  EXPECT_EQ(initializer.stats().sample_count, 1U);
}

}  // namespace
