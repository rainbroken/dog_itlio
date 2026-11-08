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

#include "itera_lio/estimator/degeneracy_detector.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>

#include "itera_lio/common/so3.hpp"

namespace itera_lio::estimator
{

namespace
{

DegeneracyReport assessInformation(
  const Eigen::Matrix<double, 6, 6> & information,
  const DegeneracyOptions & options)
{
  DegeneracyReport report;
  if (!information.allFinite() ||
    !std::isfinite(options.minimum_eigenvalue) || options.minimum_eigenvalue <= 0.0 ||
    !std::isfinite(options.maximum_condition_number) || options.maximum_condition_number <= 1.0)
  {
    return report;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(information);
  if (solver.info() != Eigen::Success) {return report;}
  report.eigenvalues = solver.eigenvalues();
  const double minimum = report.eigenvalues.minCoeff();
  const double maximum = report.eigenvalues.maxCoeff();
  report.condition_number = minimum > 0.0 ? maximum / minimum : INFINITY;
  report.degenerate = minimum<options.minimum_eigenvalue ||
      report.condition_number> options.maximum_condition_number;
  return report;
}

}  // namespace

DegeneracyReport detectDegeneracy(
  const std::vector<common::PlaneObservation> & observations,
  const NominalState & state,
  const FixedExtrinsics & extrinsics,
  const DegeneracyOptions & options)
{
  DegeneracyReport report;
  if (observations.empty() || !state.isValid() || !extrinsics.isValid() ||
    !std::isfinite(options.minimum_eigenvalue) || options.minimum_eigenvalue <= 0.0 ||
    !std::isfinite(options.maximum_condition_number) || options.maximum_condition_number <= 1.0)
  {
    return report;
  }
  Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
  for (const auto & observation : observations) {
    const Eigen::Vector3d point_I = observation.has_point_I ? observation.point_I :
      state.R_WI.transpose() * (observation.point_W - state.p_WI);
    Eigen::Matrix<double, 1, 6> jacobian;
    jacobian << -observation.normal_W.transpose() * state.R_WI * common::Hat(point_I),
      observation.normal_W.transpose();
    information += jacobian.transpose() * jacobian;
  }
  return assessInformation(information, options);
}

DegeneracyReport detectDegeneracy(
  const common::ObservationUpdateReport & observation_update,
  const DegeneracyOptions & options)
{
  DegeneracyReport report;
  if (observation_update.accepted_observations.empty() ||
    !std::isfinite(options.minimum_eigenvalue) || options.minimum_eigenvalue <= 0.0 ||
    !std::isfinite(options.maximum_condition_number) || options.maximum_condition_number <= 1.0 ||
    !observation_update.pose_information_eigenvalues.allFinite())
  {
    report.condition_number = INFINITY;
    return report;
  }
  report.eigenvalues = observation_update.pose_information_eigenvalues;
  report.condition_number = observation_update.pose_information_condition_number;
  const double minimum = report.eigenvalues.minCoeff();
  report.degenerate = minimum<options.minimum_eigenvalue ||
      report.condition_number> options.maximum_condition_number;
  return report;
}

}  // namespace itera_lio::estimator
