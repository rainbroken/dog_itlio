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

#ifndef ITERA_LIO__ESTIMATOR__ESIKF_HPP_
#define ITERA_LIO__ESTIMATOR__ESIKF_HPP_

#include <cstddef>
#include <functional>
#include <vector>

#include "itera_lio/common/observation_types.hpp"
#include "itera_lio/estimator/lidar_update.hpp"

namespace itera_lio::estimator
{

struct EsikfOptions
{
  int max_iterations{4};
  double measurement_stddev{0.05};
  double rotation_convergence{1e-3};
  double translation_convergence{1e-3};
  bool enable_mahalanobis_gate{true};
  double chi_square_threshold{9.21};
  bool enable_huber{true};
  double huber_delta{2.5};
  // Static initialization defines world vertical. Keep it fixed by default so
  // scan-to-map covariance correlations cannot change the gravity reference.
  bool update_gravity{false};
  // Physical magnitude prior, independent of floor shape or altitude.
  // Zero stddev disables it; only used when update_gravity is true.
  double gravity_magnitude{9.81};
  double gravity_magnitude_stddev{0.0};
  bool freeze_weak_directions{true};
  double weak_direction_relative_threshold{1e-3};
  double weak_direction_weight{0.1};
};

struct EsikfReport
{
  bool converged{false};
  int iterations{0};
  std::size_t effective_points{0U};
  double mean_abs_residual_before{0.0};
  double mean_abs_residual_after{0.0};
  double rotation_correction{0.0};
  double translation_correction{0.0};
  std::size_t mahalanobis_rejected{0U};
  int weak_directions{0};
  common::ObservationUpdateReport observation_update;
};

using ObservationBuilder = std::function<common::RegistrationReport(const NominalState &)>;

EsikfReport correctWithLidar(
  const ObservationBuilder & build_observations,
  const EsikfOptions & options,
  NominalState & state,
  StateCovariance & covariance,
  const FixedExtrinsics & extrinsics = {});

EsikfReport correctWithLidar(
  const std::vector<common::PlaneObservation> & observations,
  const EsikfOptions & options,
  NominalState & state,
  StateCovariance & covariance,
  const FixedExtrinsics & extrinsics = {});

}  // namespace itera_lio::estimator

#endif  // ITERA_LIO__ESTIMATOR__ESIKF_HPP_
