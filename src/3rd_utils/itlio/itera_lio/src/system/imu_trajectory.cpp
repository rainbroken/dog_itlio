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

#include "itera_lio/system/imu_trajectory.hpp"

namespace itera_lio::system
{

void applyStateCorrectionToTrajectory(
  const common::Pose & state_before,
  const common::Pose & state_after,
  std::deque<common::ImuPose> & trajectory)
{
  if (!state_before.isValid() || !state_after.isValid()) {
    return;
  }
  const Eigen::Matrix3d correction_rotation =
    state_after.R_WI * state_before.R_WI.transpose();
  const Eigen::Vector3d correction_translation =
    state_after.p_WI - correction_rotation * state_before.p_WI;
  for (auto & pose : trajectory) {
    if (!pose.R_WI.allFinite() || !pose.p_WI.allFinite() || !pose.v_WI.allFinite()) {
      continue;
    }
    pose.R_WI = correction_rotation * pose.R_WI;
    pose.p_WI = correction_rotation * pose.p_WI + correction_translation;
    pose.v_WI = correction_rotation * pose.v_WI;
  }
}

}  // namespace itera_lio::system
