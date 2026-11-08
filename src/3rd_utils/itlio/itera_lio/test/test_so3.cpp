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

TEST(So3, ExponentialLogarithmRoundTrip)
{
  const Eigen::Vector3d rotation_vector(0.2, -0.4, 0.3);
  const auto rotation = itera_lio::common::ExpSO3(rotation_vector);
  EXPECT_TRUE(itera_lio::common::LogSO3(rotation).isApprox(rotation_vector, 1e-12));
}

TEST(So3, SmallAngleExponentialIsStable)
{
  const Eigen::Vector3d rotation_vector(1e-10, -2e-10, 3e-10);
  const Eigen::Matrix3d expected =
    (Eigen::Matrix3d::Identity() + itera_lio::common::Hat(rotation_vector)).eval();
  EXPECT_LT((itera_lio::common::ExpSO3(rotation_vector) - expected).norm(), 1e-9);
}

TEST(So3, RightJacobiansAreInverses)
{
  const Eigen::Vector3d rotation_vector(0.1, 0.2, -0.15);
  EXPECT_TRUE(
    (itera_lio::common::RightJacobianSO3(rotation_vector) *
    itera_lio::common::RightJacobianInverseSO3(rotation_vector)).isApprox(
      Eigen::Matrix3d::Identity(), 1e-12));
}
