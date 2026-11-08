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

#include "itera_lio/backend/pose_graph.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include "itera_lio/common/so3.hpp"

namespace itera_lio::backend
{

bool Keyframe::isValid() const
{
  return std::isfinite(timestamp) && pose_WI.isValid();
}

bool PoseGraphConstraint::isValid() const
{
  if (from_id == to_id || !T_Ia_Ib.isValid() || !information.allFinite() ||
    (information - information.transpose()).norm() > 1e-9)
  {
    return false;
  }
  const Eigen::SelfAdjointEigenSolver<PoseInformationMatrix> solver(information);
  return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() > 0.0;
}

common::Pose composePoses(const common::Pose & T_A_B, const common::Pose & T_B_C)
{
  return common::Pose{
    T_A_B.R_WI * T_B_C.R_WI,
    T_A_B.R_WI * T_B_C.p_WI + T_A_B.p_WI};
}

common::Pose inversePose(const common::Pose & T_A_B)
{
  const Eigen::Matrix3d R_BA = T_A_B.R_WI.transpose();
  return common::Pose{R_BA, -R_BA * T_A_B.p_WI};
}

common::Pose relativePose(const common::Pose & T_W_A, const common::Pose & T_W_B)
{
  return composePoses(inversePose(T_W_A), T_W_B);
}

PoseInformationMatrix inverseMeasurementInformation(
  const common::Pose & measurement, const PoseInformationMatrix & information)
{
  PoseInformationMatrix residual_to_measurement = PoseInformationMatrix::Identity();
  residual_to_measurement.bottomLeftCorner<3, 3>() = -common::Hat(measurement.p_WI);
  const PoseInformationMatrix result = residual_to_measurement.transpose() *
    information * residual_to_measurement;
  return 0.5 * (result + result.transpose());
}

bool PoseGraph::addNode(const Keyframe & keyframe)
{
  if (!keyframe.isValid() || keyframe.id != nodes_.size()) {
    return false;
  }
  nodes_.push_back(keyframe);
  return true;
}

bool PoseGraph::addConstraint(const PoseGraphConstraint & constraint)
{
  if (!constraint.isValid() || constraint.from_id >= nodes_.size() ||
    constraint.to_id >= nodes_.size())
  {
    return false;
  }
  constraints_.push_back(constraint);
  if (constraint.type == ConstraintType::kLoop) {
    ++loop_constraint_count_;
  }
  return true;
}

}  // namespace itera_lio::backend
