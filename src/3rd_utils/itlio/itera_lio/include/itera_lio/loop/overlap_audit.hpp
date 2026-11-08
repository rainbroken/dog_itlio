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
#ifndef ITERA_LIO__LOOP__OVERLAP_AUDIT_HPP_
#define ITERA_LIO__LOOP__OVERLAP_AUDIT_HPP_
#include <vector>
#include "itera_lio/loop/loop_verifier.hpp"
namespace itera_lio::loop
{
// Evaluate a fixed transform; never refit it on these validation points.
bool auditLoopOverlap(
  const std::vector<common::MapPoint> & held_out,
  const std::vector<common::MapPoint> & target, const common::Pose & measured,
  const Eigen::Vector3d & up_in_target, const LoopVerifierOptions & options,
  LoopVerificationResult & result);
}  // namespace itera_lio::loop
#endif  // ITERA_LIO__LOOP__OVERLAP_AUDIT_HPP_
