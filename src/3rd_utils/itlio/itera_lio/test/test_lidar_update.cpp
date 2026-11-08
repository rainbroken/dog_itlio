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

#include "itera_lio/common/so3.hpp"
#include "itera_lio/estimator/lidar_update.hpp"

TEST(LidarUpdate, JacobianMatchesRightPerturbation)
{
  itera_lio::estimator::NominalState state;
  state.R_WI = itera_lio::common::ExpSO3(Eigen::Vector3d(0.1, -0.2, 0.05));
  state.p_WI = Eigen::Vector3d(1.0, -0.5, 0.2);
  itera_lio::common::PlaneObservation observation;
  observation.point_W = Eigen::Vector3d(2.0, 1.0, 1.5);
  observation.normal_W = Eigen::Vector3d(0.0, 0.0, 1.0);
  observation.centroid_W = Eigen::Vector3d(0.0, 0.0, 1.0);
  observation.residual = observation.normal_W.dot(observation.point_W - observation.centroid_W);
  const itera_lio::estimator::FixedExtrinsics extrinsics;
  const auto linearization = itera_lio::estimator::linearizeLidarObservation(
    observation, state, extrinsics);
  const Eigen::Vector3d delta(1e-7, -2e-7, 3e-7);
  const auto perturbed_rotation = state.R_WI * itera_lio::common::ExpSO3(delta);
  const Eigen::Vector3d point_I = state.R_WI.transpose() *
    (observation.point_W - state.p_WI);
  const double analytic =
    linearization.jacobian.block<1, 3>(0, 0).dot(delta);
  const double numeric = observation.normal_W.dot(
    perturbed_rotation * point_I + state.p_WI - observation.centroid_W) -
    observation.residual;
  EXPECT_NEAR(analytic, numeric, 1e-12);
  const bool position_jacobian_ok = linearization.jacobian.block<1, 3>(0, 3).isApprox(
    observation.normal_W.transpose(), 1e-12);
  EXPECT_TRUE(position_jacobian_ok);
}
