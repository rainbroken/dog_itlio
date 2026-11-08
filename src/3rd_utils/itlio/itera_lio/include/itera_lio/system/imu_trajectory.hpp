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

#ifndef ITERA_LIO__SYSTEM__IMU_TRAJECTORY_HPP_
#define ITERA_LIO__SYSTEM__IMU_TRAJECTORY_HPP_

#include <deque>

#include "itera_lio/common/geometry.hpp"

namespace itera_lio::system
{

void applyStateCorrectionToTrajectory(
  const common::Pose & state_before,
  const common::Pose & state_after,
  std::deque<common::ImuPose> & trajectory);

}  // namespace itera_lio::system

#endif  // ITERA_LIO__SYSTEM__IMU_TRAJECTORY_HPP_
