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

#include <deque>

#include <Eigen/Geometry>

#include <cmath>
#include <limits>

#include "itera_lio/frontend/deskewer.hpp"

TEST(Deskewer, CorrectsTranslationalMotionIntoEndFrame)
{
  itera_lio::sensor::LidarFrame input;
  input.scan_begin_time = 0.0;
  input.scan_end_time = 1.0;
  input.points = {
    {Eigen::Vector3d(0.0, 0.0, 0.0), 0.0F, 0.0},
    {Eigen::Vector3d(0.0, 0.0, 0.0), 0.0F, 1.0}};
  std::deque<itera_lio::frontend::ImuPose> trajectory(2U);
  trajectory[0].timestamp = 0.0;
  trajectory[1].timestamp = 1.0;
  trajectory[1].p_WI.x() = 1.0;
  itera_lio::sensor::LidarFrame output;
  ASSERT_TRUE(
    itera_lio::frontend::deskewLidarFrame(
      input, trajectory, itera_lio::common::FixedExtrinsics{}, output));
  EXPECT_TRUE(output.points[0].position.isApprox(Eigen::Vector3d(-1.0, 0.0, 0.0), 1e-12));
  EXPECT_TRUE(output.points[1].position.isApprox(Eigen::Vector3d(0.0, 0.0, 0.0), 1e-12));
}

TEST(Deskewer, InterpolatesOutOfOrderPointTimes)
{
  itera_lio::sensor::LidarFrame input;
  input.scan_begin_time = 0.0;
  input.scan_end_time = 1.0;
  input.points = {
    {Eigen::Vector3d::Zero(), 0.0F, 0.75},
    {Eigen::Vector3d::Zero(), 0.0F, 0.25},
    {Eigen::Vector3d::Zero(), 0.0F, 1.0}};
  std::deque<itera_lio::frontend::ImuPose> trajectory(3U);
  trajectory[0].timestamp = 0.0;
  trajectory[1].timestamp = 0.5;
  trajectory[2].timestamp = 1.0;
  trajectory[1].p_WI.x() = 0.25;
  trajectory[2].p_WI.x() = 1.0;
  itera_lio::sensor::LidarFrame output;
  ASSERT_TRUE(itera_lio::frontend::deskewLidarFrame(
    input, trajectory, itera_lio::common::FixedExtrinsics{}, output));
  EXPECT_NEAR(output.points[0].position.x(), -0.375, 1e-12);
  EXPECT_NEAR(output.points[1].position.x(), -0.875, 1e-12);
  EXPECT_NEAR(output.points[2].position.x(), 0.0, 1e-12);
}

TEST(Deskewer, RejectsTrajectoryWithoutTimeCoverage)
{
  itera_lio::sensor::LidarFrame input;
  input.scan_begin_time = 0.0;
  input.scan_end_time = 1.0;
  input.points = {{Eigen::Vector3d::Zero(), 0.0F, 0.5}};
  std::deque<itera_lio::frontend::ImuPose> trajectory(2U);
  trajectory[0].timestamp = 0.2;
  trajectory[1].timestamp = 0.8;
  itera_lio::sensor::LidarFrame output;
  EXPECT_FALSE(
    itera_lio::frontend::deskewLidarFrame(
      input, trajectory, itera_lio::common::FixedExtrinsics{}, output));
}

TEST(Deskewer, InterpolatesRotationOnSo3)
{
  itera_lio::sensor::LidarFrame input;
  input.scan_begin_time = 0.0;
  input.scan_end_time = 1.0;
  input.points = {{Eigen::Vector3d(1.0, 0.0, 0.0), 0.0F, 0.5}};

  std::deque<itera_lio::frontend::ImuPose> trajectory(2U);
  trajectory[0].timestamp = 0.0;
  trajectory[1].timestamp = 1.0;
  trajectory[1].R_WI =
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  itera_lio::sensor::LidarFrame output;
  ASSERT_TRUE(
    itera_lio::frontend::deskewLidarFrame(
      input, trajectory, itera_lio::common::FixedExtrinsics{}, output));
  const double half_sqrt_two = std::sqrt(0.5);
  EXPECT_TRUE(
    output.points[0].position.isApprox(
      Eigen::Vector3d(half_sqrt_two, -half_sqrt_two, 0.0), 1e-12));
}

TEST(Deskewer, ProducesFullAndSelectedFramesInOnePass)
{
  itera_lio::sensor::LidarFrame input;
  input.scan_begin_time = 0.0;
  input.scan_end_time = 1.0;
  input.points = {
    {Eigen::Vector3d(0.0, 0.0, 0.0), 1.0F, 0.0},
    {Eigen::Vector3d(1.0, 0.0, 0.0), 2.0F, 0.5},
    {Eigen::Vector3d(2.0, 0.0, 0.0), 3.0F, 1.0}};
  std::deque<itera_lio::frontend::ImuPose> trajectory(2U);
  trajectory[0].timestamp = 0.0;
  trajectory[1].timestamp = 1.0;
  trajectory[1].p_WI.x() = 1.0;

  itera_lio::sensor::LidarFrame expected_full;
  ASSERT_TRUE(
    itera_lio::frontend::deskewLidarFrame(
      input, trajectory, itera_lio::common::FixedExtrinsics{}, expected_full));
  itera_lio::sensor::LidarFrame full;
  itera_lio::sensor::LidarFrame selected;
  ASSERT_TRUE(
    itera_lio::frontend::deskewLidarFrameAndSelect(
      input, {0U, 2U}, trajectory, itera_lio::common::FixedExtrinsics{}, full, selected));

  ASSERT_EQ(full.points.size(), expected_full.points.size());
  ASSERT_EQ(selected.points.size(), 2U);
  for (std::size_t index = 0U; index < full.points.size(); ++index) {
    EXPECT_TRUE(full.points[index].position.isApprox(expected_full.points[index].position, 1e-12));
  }
  EXPECT_TRUE(selected.points[0].position.isApprox(full.points[0].position, 1e-12));
  EXPECT_TRUE(selected.points[1].position.isApprox(full.points[2].position, 1e-12));
  EXPECT_FLOAT_EQ(selected.points[0].intensity, 1.0F);
  EXPECT_FLOAT_EQ(selected.points[1].intensity, 3.0F);
}

TEST(Deskewer, ParallelOutputIsExactForUnorderedAndBoundaryTimes)
{
  using namespace itera_lio;
  sensor::LidarFrame input;
  input.scan_begin_time = 0.0;
  input.scan_end_time = 1.0;
  std::deque<frontend::ImuPose> trajectory(21);
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    trajectory[i].timestamp = double(i) / 20.;
    trajectory[i].R_WI = Eigen::AngleAxisd(.13 * i,
      Eigen::Vector3d(1., 2., 3.).normalized()).toRotationMatrix();
    trajectory[i].p_WI = Eigen::Vector3d(.1 * i, .03 * i * i, .01 * i);
  }
  std::vector<std::size_t> selected;
  for (std::size_t i = 0; i < 10000; ++i) {
    input.points.push_back({{.03 * i, .04 * (i % 91), .02 * (i % 7)}, float(i),
      double((i * 13) % 21) / 20.});
    if (i % 3 == 0) {selected.push_back(i);}
  }
  sensor::LidarFrame serial, parallel, serial_selected, parallel_selected;
  ASSERT_TRUE(frontend::deskewLidarFrameAndSelect(input, selected, trajectory, {},
    serial, serial_selected, 1));
  ASSERT_TRUE(frontend::deskewLidarFrameAndSelect(input, selected, trajectory, {},
    parallel, parallel_selected, 3));
  ASSERT_EQ(serial.points.size(), parallel.points.size());
  ASSERT_EQ(serial_selected.points.size(), parallel_selected.points.size());
  for (std::size_t i = 0; i < serial.points.size(); ++i) {
    EXPECT_EQ(serial.points[i].position, parallel.points[i].position);
  }
  for (std::size_t i = 0; i < serial_selected.points.size(); ++i) {
    EXPECT_EQ(serial_selected.points[i].position, parallel_selected.points[i].position);
  }
}

// Regression cases for moving output validation into the deskew loop.
TEST(Deskewer, RejectsTransformOverflowWithoutClearingCompletedFrame)
{
  using namespace itera_lio;
  sensor::LidarFrame input;
  input.scan_begin_time = 0.; input.scan_end_time = 1.;
  const double large = std::numeric_limits<double>::max() * .75;
  for (int i = 0; i < 5000; ++i) {
    input.points.push_back({{large, 1., 1.}, 2.F, .5});
  }
  std::deque<frontend::ImuPose> trajectory(2);
  trajectory[0].timestamp = 0.; trajectory[1].timestamp = 1.;
  common::FixedExtrinsics extrinsics;
  extrinsics.p_IL.x() = large;
  for (const int threads : {1, 3}) {
    sensor::LidarFrame full, selected;
    EXPECT_FALSE(frontend::deskewLidarFrameAndSelect(input, {0U, 4999U}, trajectory,
      extrinsics, full, selected, threads));
    EXPECT_EQ(full.points.size(), input.points.size());
    EXPECT_EQ(selected.points.size(), 2U);
    EXPECT_FALSE(full.isValid());
  }
}

TEST(Deskewer, RejectsInteriorInvalidInputBeforeTransform)
{
  using namespace itera_lio;
  sensor::LidarFrame input;
  input.scan_begin_time = 0.; input.scan_end_time = 1.;
  input.points.resize(5000);
  for (auto & point : input.points) {point.relative_time = .5;}
  input.points[2500].intensity = std::numeric_limits<float>::infinity();
  std::deque<frontend::ImuPose> trajectory(2);
  trajectory[0].timestamp = 0.; trajectory[1].timestamp = 1.;
  for (const int threads : {1, 3}) {
    sensor::LidarFrame output;
    EXPECT_FALSE(frontend::deskewLidarFrame(input, trajectory, {}, output, threads));
    EXPECT_TRUE(output.points.empty());
  }
}

// Serial error semantics and threaded interior-invalid rejection are covered
// without relying on signaling-NaN or undefined floating-point behavior.
