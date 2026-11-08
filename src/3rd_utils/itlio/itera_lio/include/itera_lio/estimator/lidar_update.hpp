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

#ifndef ITERA_LIO__ESTIMATOR__LIDAR_UPDATE_HPP_
#define ITERA_LIO__ESTIMATOR__LIDAR_UPDATE_HPP_

#include <Eigen/Core>

#include "itera_lio/common/observation_types.hpp"
#include "itera_lio/estimator/state.hpp"

namespace itera_lio::estimator
{

struct LidarLinearization
{
  Eigen::Matrix<double, 1, ErrorStateLayout::kDimension> jacobian{
    Eigen::Matrix<double, 1, ErrorStateLayout::kDimension>::Zero()};
  double residual{0.0};
};

LidarLinearization linearizeLidarObservation(
  const common::PlaneObservation & observation,
  const NominalState & state,
  const FixedExtrinsics & extrinsics);

}  // namespace itera_lio::estimator

#endif  // ITERA_LIO__ESTIMATOR__LIDAR_UPDATE_HPP_
