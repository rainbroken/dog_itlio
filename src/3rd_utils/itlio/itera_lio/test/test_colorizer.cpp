// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>

#include "itera_lio/mapping/color/colorizer.hpp"

TEST(Colorizer, ProjectsInLidarFrameAndStoresWorldPoint) {
  itera_lio::sensor::LidarFrame scan;
  scan.scan_begin_time = 0.0;
  scan.scan_end_time = 0.0;
  scan.points = {{{0.0, 0.0, 2.0}, 7.0F, 0.0}};

  itera_lio::estimator::NominalState state;
  state.R_WI =
      Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  state.p_WI = Eigen::Vector3d(5.0, -2.0, 1.0);
  itera_lio::common::FixedExtrinsics extrinsics;
  extrinsics.R_IL =
      Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  extrinsics.p_IL = Eigen::Vector3d(0.1, 0.2, 0.3);

  sensor_msgs::msg::Image image;
  image.encoding = "rgb8";
  image.width = 3U;
  image.height = 3U;
  image.step = 9U;
  image.data.assign(27U, 0U);
  image.data[12] = 10U;
  image.data[13] = 20U;
  image.data[14] = 30U;

  itera_lio::mapping::color::CameraColorizerOptions options;
  options.width = 3;
  options.height = 3;
  options.fx = 1.0;
  options.fy = 1.0;
  options.cx = 1.0;
  options.cy = 1.0;
  options.k1 = 0.0;
  options.k2 = 0.0;
  options.p1 = 0.0;
  options.p2 = 0.0;

  std::vector<itera_lio::mapping::color::ColoredPoint> output;
  ASSERT_TRUE(itera_lio::mapping::color::colorize(
      scan, state, extrinsics, image, options, output));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.front().r, 10U);
  EXPECT_EQ(output.front().g, 20U);
  EXPECT_EQ(output.front().b, 30U);
  const Eigen::Vector3f expected =
      (state.R_WI *
           (extrinsics.R_IL * scan.points.front().position + extrinsics.p_IL) +
       state.p_WI)
          .cast<float>();
  EXPECT_TRUE(output.front().position.isApprox(expected, 1e-6F));
}

TEST(Colorizer, RejectsNonFiniteCameraCalibration) {
  itera_lio::sensor::LidarFrame scan;
  scan.scan_begin_time = 0.0;
  scan.scan_end_time = 0.0;
  scan.points = {{{0.0, 0.0, 2.0}, 7.0F, 0.0}};

  sensor_msgs::msg::Image image;
  image.encoding = "rgb8";
  image.width = 1U;
  image.height = 1U;
  image.step = 3U;
  image.data.assign(3U, 0U);

  itera_lio::mapping::color::CameraColorizerOptions options;
  options.width = 1;
  options.height = 1;
  options.fx = std::numeric_limits<double>::quiet_NaN();
  std::vector<itera_lio::mapping::color::ColoredPoint> output;
  EXPECT_FALSE(itera_lio::mapping::color::colorize(
      scan, itera_lio::estimator::NominalState{},
      itera_lio::common::FixedExtrinsics{}, image, options, output));
}

TEST(Colorizer, ResizesPaddedBgrImageAndTransformsProjectionWithPixelCenters) {
  // At half resolution, source coordinate (3, 1) maps to (1.25, 0.25).
  // The selected output pixel averages the top-right 2x2 source block.
  sensor_msgs::msg::Image image;
  image.encoding = "bgr8";
  image.width = 4;
  image.height = 4;
  image.step = 16;  // Include row padding.
  image.data.assign(64, 0);
  for (std::size_t row = 0; row < 2; ++row) {
    for (std::size_t col = 2; col < 4; ++col) {
      const auto index = row * image.step + col * 3;
      image.data[index] = static_cast<std::uint8_t>(10 + (col - 2) * 20);
      image.data[index + 1] = 40;
      image.data[index + 2] = 60;
    }
  }
  const auto original = image.data;
  itera_lio::mapping::color::CameraColorizerOptions options;
  options.width = 4;
  options.height = 4;
  options.fx = options.fy = 2.0;
  options.cx = options.cy = 1.0;
  options.k1 = options.k2 = options.p1 = options.p2 = 0.0;
  options.image_scale = 0.5;
  itera_lio::sensor::LidarFrame scan;
  scan.points = {{{2.0, 0.0, 2.0}, 7.0F, 0.0}};
  std::vector<itera_lio::mapping::color::ColoredPoint> output;
  ASSERT_TRUE(itera_lio::mapping::color::colorize(
      scan, itera_lio::estimator::NominalState{},
      itera_lio::common::FixedExtrinsics{}, image, options, output));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.front().r, 60U);
  EXPECT_EQ(output.front().g, 40U);
  EXPECT_EQ(output.front().b, 20U);
  EXPECT_TRUE(output.front().position.isApprox(Eigen::Vector3f(2, 0, 2)));
  EXPECT_EQ(image.data, original);
}

TEST(Colorizer, RejectsInvalidScale) {
  sensor_msgs::msg::Image image;
  image.encoding = "rgb8";
  image.width = image.height = 1;
  image.step = 3;
  image.data.assign(3, 0);
  itera_lio::mapping::color::CameraColorizerOptions options;
  options.width = options.height = 1;
  std::vector<itera_lio::mapping::color::ColoredPoint> output;
  for (const double scale : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::max()}) {
    options.image_scale = scale;
    EXPECT_FALSE(itera_lio::mapping::color::colorize(
        itera_lio::sensor::LidarFrame{}, itera_lio::estimator::NominalState{},
        itera_lio::common::FixedExtrinsics{}, image, options, output));
  }
}

TEST(Colorizer, UsesActualScaleAfterRoundingOddDimensions) {
  sensor_msgs::msg::Image image;
  image.encoding = "rgb8";
  image.width = 5;
  image.height = 3;
  image.step = 15;
  image.data.assign(45, 0);
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 3; col < 5; ++col) {
      image.data[row * image.step + col * 3] = 100;
    }
  }
  itera_lio::mapping::color::CameraColorizerOptions options;
  options.width = 5;
  options.height = 3;
  options.fx = options.fy = 1.0;
  options.cx = options.cy = 0.0;
  options.k1 = options.k2 = options.p1 = options.p2 = 0.0;
  options.image_scale = 0.5;
  itera_lio::sensor::LidarFrame scan;
  scan.points = {{{2.9, 0.0, 1.0}, 0.0F, 0.0}};
  std::vector<itera_lio::mapping::color::ColoredPoint> output;
  ASSERT_TRUE(itera_lio::mapping::color::colorize(
      scan, itera_lio::estimator::NominalState{},
      itera_lio::common::FixedExtrinsics{}, image, options, output));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.front().r, 100U);
}
