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

#ifndef ITERA_LIO__ESTIMATOR__DEGENERACY_DETECTOR_HPP_
#define ITERA_LIO__ESTIMATOR__DEGENERACY_DETECTOR_HPP_

#include <Eigen/Core>

#include <vector>

#include "itera_lio/common/observation_types.hpp"
#include "itera_lio/estimator/lidar_update.hpp"

namespace itera_lio::estimator
{

struct DegeneracyOptions
{
  double minimum_eigenvalue{1e-6};
  double maximum_condition_number{1e8};
};

struct DegeneracyReport
{
  bool degenerate{true};
  Eigen::Matrix<double, 6, 1> eigenvalues{Eigen::Matrix<double, 6, 1>::Zero()};
  double condition_number{0.0};
};

DegeneracyReport detectDegeneracy(
  const std::vector<common::PlaneObservation> & observations,
  const NominalState & state,
  const FixedExtrinsics & extrinsics,
  const DegeneracyOptions & options = {});

DegeneracyReport detectDegeneracy(
  const common::ObservationUpdateReport & observation_update,
  const DegeneracyOptions & options = {});

}  // namespace itera_lio::estimator

#endif  // ITERA_LIO__ESTIMATOR__DEGENERACY_DETECTOR_HPP_
