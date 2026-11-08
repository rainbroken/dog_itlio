// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/parameter_reader.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace itera_lio::ros
{

std::size_t ParameterReader::positiveSize(
  const std::string & name, const std::int64_t default_value) const
{
  const auto value = node_.declare_parameter<std::int64_t>(name, default_value);
  if (value <= 0) {
    throw std::invalid_argument(name + " must be positive");
  }
  return static_cast<std::size_t>(value);
}

std::size_t ParameterReader::nonNegativeSize(
  const std::string & name, const std::int64_t default_value) const
{
  const auto value = node_.declare_parameter<std::int64_t>(name, default_value);
  if (value < 0) {
    throw std::invalid_argument(name + " must be non-negative");
  }
  return static_cast<std::size_t>(value);
}

estimator::FixedExtrinsics ParameterReader::lidarExtrinsics() const
{
  const std::vector<double> translation = node_.declare_parameter<std::vector<double>>(
    "extrin_calib.extrinsic_T", {-0.011, -0.02329, 0.04412});
  const std::vector<double> rotation = node_.declare_parameter<std::vector<double>>(
    "extrin_calib.extrinsic_R", {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0});
  if (translation.size() != 3U || rotation.size() != 9U) {
    throw std::invalid_argument(
            "extrinsic calibration must contain 3 translation and 9 rotation values");
  }

  estimator::FixedExtrinsics extrinsics;
  for (std::size_t index = 0U; index < translation.size(); ++index) {
    if (!std::isfinite(translation[index])) {
      throw std::invalid_argument("extrin_calib.extrinsic_T contains a non-finite value");
    }
    extrinsics.p_IL(static_cast<Eigen::Index>(index)) = translation[index];
  }
  for (std::size_t index = 0U; index < rotation.size(); ++index) {
    if (!std::isfinite(rotation[index])) {
      throw std::invalid_argument("extrin_calib.extrinsic_R contains a non-finite value");
    }
    extrinsics.R_IL(
      static_cast<Eigen::Index>(index / 3U),
      static_cast<Eigen::Index>(index % 3U)) = rotation[index];
  }
  if (!extrinsics.isValid(1e-6)) {
    throw std::invalid_argument(
            "extrin_calib.extrinsic_R must be a valid SO(3) rotation matrix");
  }
  return extrinsics;
}

}  // namespace itera_lio::ros
