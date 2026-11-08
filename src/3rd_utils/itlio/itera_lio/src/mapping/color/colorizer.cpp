// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/mapping/color/colorizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace itera_lio::mapping::color {

bool colorize(const sensor::LidarFrame &scan,
              const estimator::NominalState &state,
              const common::FixedExtrinsics &extrinsics,
              const sensor_msgs::msg::Image &image,
              const CameraColorizerOptions &options,
              std::vector<ColoredPoint> &output) {
  const bool rgb_encoding = image.encoding == "rgb8";
  const std::array<double, 8U> camera_parameters = {
      options.fx, options.fy, options.cx, options.cy,
      options.k1, options.k2, options.p1, options.p2};
  if ((!rgb_encoding && image.encoding != "bgr8") || options.width <= 0 ||
      options.height <= 0 ||
      !std::isfinite(options.image_scale) || options.image_scale <= 0.0 ||
      image.width != static_cast<unsigned int>(options.width) ||
      image.height != static_cast<unsigned int>(options.height) ||
      static_cast<std::size_t>(image.step) <
          static_cast<std::size_t>(image.width) * 3U ||
      image.data.size() < static_cast<std::size_t>(image.step) * image.height ||
      !std::all_of(camera_parameters.begin(), camera_parameters.end(),
                   [](const double value) { return std::isfinite(value); }) ||
      options.fx <= 0.0 || options.fy <= 0.0 || !options.R_CL.allFinite() ||
      !options.p_CL.allFinite() || !state.isValid() || !extrinsics.isValid()) {
    return false;
  }

  // Resize only matched frames. Preserve the input message used by recording
  // and synchronization, and account for rounded output dimensions per axis.
  const double scaled_width = std::max(1.0, std::round(options.width * options.image_scale));
  const double scaled_height = std::max(1.0, std::round(options.height * options.image_scale));
  if (!std::isfinite(scaled_width) || !std::isfinite(scaled_height) ||
      scaled_width > std::numeric_limits<int>::max() ||
      scaled_height > std::numeric_limits<int>::max()) {
    return false;
  }
  const int width = static_cast<int>(scaled_width);
  const int height = static_cast<int>(scaled_height);
  cv::Mat resized;
  const std::uint8_t *pixels = image.data.data();
  std::size_t step = image.step;
  if (width != options.width || height != options.height) {
    const cv::Mat source(options.height, options.width, CV_8UC3,
                         const_cast<std::uint8_t *>(pixels), image.step);
    cv::resize(source, resized, cv::Size(width, height), 0.0, 0.0,
               options.image_scale < 1.0 ? cv::INTER_AREA : cv::INTER_LINEAR);
    pixels = resized.data;
    step = resized.step;
  }
  const double sx = static_cast<double>(width) / options.width;
  const double sy = static_cast<double>(height) / options.height;

  const Eigen::Matrix3d R_WL = state.R_WI * extrinsics.R_IL;
  const Eigen::Vector3d p_WL = state.R_WI * extrinsics.p_IL + state.p_WI;
  output.clear();
  output.reserve(scan.points.size());
  for (const auto &point : scan.points) {
    // Camera extrinsics are T_C_L, so projection is independent of the world
    // pose and the LiDAR-to-IMU transform.
    const Eigen::Vector3d point_C =
        options.R_CL * point.position + options.p_CL;
    if (!point_C.allFinite() || point_C.z() <= 0.0) {
      continue;
    }
    const double x = point_C.x() / point_C.z();
    const double y = point_C.y() / point_C.z();
    const double radius_squared = x * x + y * y;
    const double radial = 1.0 + options.k1 * radius_squared +
                          options.k2 * radius_squared * radius_squared;
    const double u =
        options.fx * (x * radial + 2.0 * options.p1 * x * y +
                      options.p2 * (radius_squared + 2.0 * x * x)) +
        options.cx;
    const double v =
        options.fy * (y * radial + options.p1 * (radius_squared + 2.0 * y * y) +
                      2.0 * options.p2 * x * y) +
        options.cy;
    // OpenCV resize maps pixel centers as (u + 0.5) * scale - 0.5.
    const double scaled_u = (u + 0.5) * sx - 0.5;
    const double scaled_v = (v + 0.5) * sy - 0.5;
    if (!std::isfinite(scaled_u) || !std::isfinite(scaled_v) ||
        scaled_u <= -0.5 || scaled_v <= -0.5 ||
        scaled_u >= width - 0.5 || scaled_v >= height - 0.5) {
      continue;
    }
    const int column = static_cast<int>(std::lround(scaled_u));
    const int row = static_cast<int>(std::lround(scaled_v));
    const std::size_t image_index = static_cast<std::size_t>(row) * step +
                                    static_cast<std::size_t>(column) * 3U;
    ColoredPoint colored;
    colored.position = (R_WL * point.position + p_WL).cast<float>();
    colored.intensity = point.intensity;
    if (rgb_encoding) {
      colored.r = pixels[image_index];
      colored.g = pixels[image_index + 1U];
      colored.b = pixels[image_index + 2U];
    } else {
      colored.b = pixels[image_index];
      colored.g = pixels[image_index + 1U];
      colored.r = pixels[image_index + 2U];
    }
    output.push_back(colored);
  }
  return true;
}

} // namespace itera_lio::mapping::color
