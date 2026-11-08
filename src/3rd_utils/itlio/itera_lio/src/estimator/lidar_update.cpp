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

#include "itera_lio/estimator/lidar_update.hpp"

#include "itera_lio/common/so3.hpp"

namespace itera_lio::estimator
{

LidarLinearization linearizeLidarObservation(
  const common::PlaneObservation & observation,
  const NominalState & state,
  const FixedExtrinsics & extrinsics)
{
  LidarLinearization result;
  (void)extrinsics;
  const Eigen::Vector3d point_I_nominal = observation.has_point_I ? observation.point_I :
    state.R_WI.transpose() * (observation.point_W - state.p_WI);
  const Eigen::Vector3d point_W = state.R_WI * point_I_nominal + state.p_WI;
  result.residual = observation.residual;
  if (observation.has_point_I) {
    result.residual = observation.normal_W.dot(point_W - observation.centroid_W);
  }
  result.jacobian.template block<1, 3>(0, ErrorStateLayout::kRotation) =
    (-observation.normal_W.transpose() * state.R_WI * common::Hat(point_I_nominal));
  result.jacobian.template block<1, 3>(0, ErrorStateLayout::kPosition) =
    observation.normal_W.transpose();
  return result;
}

}  // namespace itera_lio::estimator
