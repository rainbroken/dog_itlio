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

#include <Eigen/Geometry>

#include <cmath>
#include <deque>

#include "itera_lio/system/imu_trajectory.hpp"

TEST(ImuTrajectory, AppliesAcceptedStateCorrection)
{
  itera_lio::common::Pose before;
  itera_lio::common::Pose after;
  after.p_WI.x() = 2.0;
  after.R_WI = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::deque<itera_lio::common::ImuPose> trajectory(1U);
  trajectory[0].p_WI = Eigen::Vector3d(1.0, 0.0, 0.0);
  trajectory[0].v_WI = Eigen::Vector3d(1.0, 0.0, 0.0);
  itera_lio::system::applyStateCorrectionToTrajectory(before, after, trajectory);
  EXPECT_TRUE(trajectory[0].p_WI.isApprox(Eigen::Vector3d(2.0, 1.0, 0.0), 1e-12));
  EXPECT_TRUE(trajectory[0].v_WI.isApprox(Eigen::Vector3d(0.0, 1.0, 0.0), 1e-12));
  EXPECT_TRUE(trajectory[0].R_WI.isApprox(after.R_WI, 1e-12));
}
