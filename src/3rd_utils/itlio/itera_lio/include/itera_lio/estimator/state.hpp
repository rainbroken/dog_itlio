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

#ifndef ITERA_LIO__ESTIMATOR__STATE_HPP_
#define ITERA_LIO__ESTIMATOR__STATE_HPP_

#include <Eigen/Core>

#include "itera_lio/common/geometry.hpp"

namespace itera_lio::estimator
{

struct ErrorStateLayout
{
  static constexpr int kRotation = 0;
  static constexpr int kPosition = 3;
  static constexpr int kVelocity = 6;
  static constexpr int kGyroscopeBias = 9;
  static constexpr int kAccelerometerBias = 12;
  static constexpr int kGravity = 15;
  static constexpr int kBlockSize = 3;
  static constexpr int kDimension = 18;
};

using ErrorStateVector = Eigen::Matrix<double, ErrorStateLayout::kDimension, 1>;
using StateCovariance = Eigen::Matrix<
  double, ErrorStateLayout::kDimension, ErrorStateLayout::kDimension>;

struct NominalState
{
  Eigen::Matrix3d R_WI{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d p_WI{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_WI{Eigen::Vector3d::Zero()};
  Eigen::Vector3d b_g{Eigen::Vector3d::Zero()};
  Eigen::Vector3d b_a{Eigen::Vector3d::Zero()};
  Eigen::Vector3d g_W{Eigen::Vector3d::Zero()};

  bool isValid(double rotation_tolerance = 1e-9) const;
};

using FixedExtrinsics = common::FixedExtrinsics;

bool isValidRotation(const Eigen::Matrix3d & rotation, double tolerance = 1e-9);
bool isValidCovariance(const StateCovariance & covariance, double tolerance = 1e-12);

}  // namespace itera_lio::estimator

#endif  // ITERA_LIO__ESTIMATOR__STATE_HPP_
