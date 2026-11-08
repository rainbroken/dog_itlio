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

#include "itera_lio/estimator/imu_propagator.hpp"

namespace
{

itera_lio::sensor::ImuData sample(double timestamp, const Eigen::Vector3d & accel)
{
  itera_lio::sensor::ImuData value;
  value.timestamp = timestamp;
  value.accel = accel;
  return value;
}

TEST(ImuPropagator, StationaryStateRemainsAtRest)
{
  itera_lio::estimator::NominalState state;
  state.g_W = Eigen::Vector3d(0.0, 0.0, -9.81);
  itera_lio::estimator::StateCovariance covariance =
    itera_lio::estimator::StateCovariance::Zero();
  itera_lio::estimator::ImuPropagator propagator;
  const auto imu0 = sample(0.0, Eigen::Vector3d(0.0, 0.0, 9.81));
  const auto imu1 = sample(0.1, Eigen::Vector3d(0.0, 0.0, 9.81));
  ASSERT_TRUE(propagator.propagate(imu0, imu1, state, covariance));
  EXPECT_TRUE(state.p_WI.isApprox(Eigen::Vector3d::Zero(), 1e-12));
  EXPECT_TRUE(state.v_WI.isApprox(Eigen::Vector3d::Zero(), 1e-12));
  EXPECT_TRUE(covariance.isApprox(covariance.transpose(), 1e-15));
}

TEST(ImuPropagator, ConstantVelocityHasExpectedPosition)
{
  itera_lio::estimator::NominalState state;
  state.g_W.setZero();
  state.v_WI = Eigen::Vector3d(1.0, -2.0, 0.5);
  itera_lio::estimator::StateCovariance covariance =
    itera_lio::estimator::StateCovariance::Identity();
  itera_lio::estimator::ImuPropagator propagator;
  const auto imu0 = sample(1.0, Eigen::Vector3d::Zero());
  const auto imu1 = sample(1.2, Eigen::Vector3d::Zero());
  ASSERT_TRUE(propagator.propagate(imu0, imu1, state, covariance));
  EXPECT_TRUE(state.p_WI.isApprox(Eigen::Vector3d(0.2, -0.4, 0.1), 1e-12));
  EXPECT_TRUE(state.v_WI.isApprox(Eigen::Vector3d(1.0, -2.0, 0.5), 1e-12));
}

TEST(ImuPropagator, RejectsInvalidTime)
{
  itera_lio::estimator::NominalState state;
  itera_lio::estimator::StateCovariance covariance =
    itera_lio::estimator::StateCovariance::Identity();
  itera_lio::estimator::ImuPropagator propagator;
  const auto imu0 = sample(1.0, Eigen::Vector3d::Zero());
  const auto imu1 = sample(1.0, Eigen::Vector3d::Zero());
  EXPECT_FALSE(propagator.propagate(imu0, imu1, state, covariance));
}

}  // namespace
