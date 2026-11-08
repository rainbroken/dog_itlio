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

#include <Eigen/Core>

#include <limits>

#include "gtest/gtest.h"
#include "itera_lio/estimator/state.hpp"

namespace
{

using itera_lio::estimator::ErrorStateLayout;
using itera_lio::estimator::FixedExtrinsics;
using itera_lio::estimator::NominalState;
using itera_lio::estimator::StateCovariance;

TEST(State, ErrorStateLayoutIsContiguousAnd18Dimensional)
{
  EXPECT_EQ(ErrorStateLayout::kRotation, 0);
  EXPECT_EQ(ErrorStateLayout::kPosition, ErrorStateLayout::kRotation + 3);
  EXPECT_EQ(ErrorStateLayout::kVelocity, ErrorStateLayout::kPosition + 3);
  EXPECT_EQ(ErrorStateLayout::kGyroscopeBias, ErrorStateLayout::kVelocity + 3);
  EXPECT_EQ(ErrorStateLayout::kAccelerometerBias, ErrorStateLayout::kGyroscopeBias + 3);
  EXPECT_EQ(ErrorStateLayout::kGravity, ErrorStateLayout::kAccelerometerBias + 3);
  EXPECT_EQ(ErrorStateLayout::kDimension, ErrorStateLayout::kGravity + 3);
}

TEST(State, DefaultNominalStateIsFiniteAndUninitializedGravityIsZero)
{
  const NominalState state;
  EXPECT_TRUE(state.R_WI.isIdentity());
  EXPECT_TRUE(state.p_WI.isZero());
  EXPECT_TRUE(state.v_WI.isZero());
  EXPECT_TRUE(state.b_g.isZero());
  EXPECT_TRUE(state.b_a.isZero());
  EXPECT_TRUE(state.g_W.isZero());
  EXPECT_TRUE(state.isValid());
}

TEST(State, RejectsInvalidRotationAndNonFiniteValues)
{
  NominalState state;
  state.R_WI(0, 0) = 2.0;
  EXPECT_FALSE(state.isValid());

  state = NominalState{};
  state.p_WI.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(state.isValid());
}

TEST(State, FixedExtrinsicsUseLidarToImuConvention)
{
  const FixedExtrinsics extrinsics;
  const Eigen::Vector3d p_L(1.0, 2.0, 3.0);
  const Eigen::Vector3d p_I = extrinsics.R_IL * p_L + extrinsics.p_IL;
  EXPECT_TRUE(p_I.isApprox(p_L));
  EXPECT_TRUE(extrinsics.isValid());
}

TEST(State, CovarianceMustBeFiniteAndSymmetric)
{
  StateCovariance covariance = StateCovariance::Identity();
  EXPECT_TRUE(itera_lio::estimator::isValidCovariance(covariance));

  covariance(0, 1) = 1.0;
  EXPECT_FALSE(itera_lio::estimator::isValidCovariance(covariance));

  covariance = StateCovariance::Identity();
  covariance(0, 0) = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(itera_lio::estimator::isValidCovariance(covariance));

  covariance = StateCovariance::Identity();
  covariance(0, 0) = -1.0;
  EXPECT_FALSE(itera_lio::estimator::isValidCovariance(covariance));
}

}  // namespace
