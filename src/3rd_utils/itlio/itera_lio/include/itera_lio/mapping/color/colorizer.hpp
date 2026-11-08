// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__MAPPING__COLOR__COLORIZER_HPP_
#define ITERA_LIO__MAPPING__COLOR__COLORIZER_HPP_

#include <Eigen/Core>

#include <cstdint>
#include <vector>

#include <sensor_msgs/msg/image.hpp>

#include "itera_lio/common/geometry.hpp"
#include "itera_lio/estimator/state.hpp"
#include "itera_lio/sensor/lidar_frame.hpp"

namespace itera_lio::mapping::color {

struct ColoredPoint {
  Eigen::Vector3f position{Eigen::Vector3f::Zero()};
  float intensity{0.0F};
  std::uint8_t r{0U};
  std::uint8_t g{0U};
  std::uint8_t b{0U};
};

struct CameraColorizerOptions {
  Eigen::Matrix3d R_CL{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d p_CL{Eigen::Vector3d::Zero()};
  double fx{907.292};
  double fy{903.016};
  double cx{632.5};
  double cy{365.557};
  double k1{0.074039};
  double k2{-0.114819};
  double p1{0.002575};
  double p2{-0.004635};
  int width{1280};
  int height{720};
  // Intrinsics and dimensions above describe the unscaled input image.
  double image_scale{1.0};
};

bool colorize(const sensor::LidarFrame &scan,
              const estimator::NominalState &state,
              const common::FixedExtrinsics &extrinsics,
              const sensor_msgs::msg::Image &image,
              const CameraColorizerOptions &options,
              std::vector<ColoredPoint> &output);

} // namespace itera_lio::mapping::color

#endif // ITERA_LIO__MAPPING__COLOR__COLORIZER_HPP_
