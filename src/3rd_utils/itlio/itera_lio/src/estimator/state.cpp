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

#include "itera_lio/estimator/state.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>

namespace itera_lio::estimator
{

bool isValidRotation(const Eigen::Matrix3d & rotation, const double tolerance)
{
  if (!rotation.allFinite() || !std::isfinite(tolerance) || tolerance < 0.0) {
    return false;
  }
  const Eigen::Matrix3d orthogonality_error =
    rotation.transpose() * rotation - Eigen::Matrix3d::Identity();
  return orthogonality_error.norm() <= tolerance &&
         std::abs(rotation.determinant() - 1.0) <= tolerance;
}

bool NominalState::isValid(const double rotation_tolerance) const
{
  return isValidRotation(R_WI, rotation_tolerance) && p_WI.allFinite() &&
         v_WI.allFinite() && b_g.allFinite() && b_a.allFinite() && g_W.allFinite();
}

bool isValidCovariance(
  const StateCovariance & covariance, const double tolerance)
{
  if (!covariance.allFinite() || !std::isfinite(tolerance) || tolerance < 0.0) {
    return false;
  }
  if ((covariance - covariance.transpose()).norm() > tolerance) {
    return false;
  }

  Eigen::SelfAdjointEigenSolver<StateCovariance> eigen_solver(covariance);
  return eigen_solver.info() == Eigen::Success &&
         eigen_solver.eigenvalues().minCoeff() >= -tolerance;
}

}  // namespace itera_lio::estimator
