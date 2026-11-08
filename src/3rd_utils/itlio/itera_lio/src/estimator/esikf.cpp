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

#include "itera_lio/estimator/esikf.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "itera_lio/common/so3.hpp"

namespace itera_lio::estimator
{

namespace
{

bool validOptions(const EsikfOptions & options)
{
  return options.max_iterations > 0 && std::isfinite(options.measurement_stddev) &&
         options.measurement_stddev > 0.0 && std::isfinite(options.chi_square_threshold) &&
         options.chi_square_threshold > 0.0 && std::isfinite(options.huber_delta) &&
         options.huber_delta > 0.0 &&
         std::isfinite(options.weak_direction_relative_threshold) &&
         options.weak_direction_relative_threshold >= 0.0 &&
         std::isfinite(options.weak_direction_weight) &&
         options.weak_direction_weight >= 0.0 && options.weak_direction_weight <= 1.0 &&
         std::isfinite(options.gravity_magnitude) && options.gravity_magnitude > 0.0 &&
         std::isfinite(options.gravity_magnitude_stddev) && options.gravity_magnitude_stddev >= 0.0;
}

void applyCorrection(NominalState & state, const ErrorStateVector & correction)
{
  state.R_WI = state.R_WI * common::ExpSO3(
    correction.segment<3>(ErrorStateLayout::kRotation));
  state.p_WI += correction.segment<3>(ErrorStateLayout::kPosition);
  state.v_WI += correction.segment<3>(ErrorStateLayout::kVelocity);
  state.b_g += correction.segment<3>(ErrorStateLayout::kGyroscopeBias);
  state.b_a += correction.segment<3>(ErrorStateLayout::kAccelerometerBias);
  state.g_W += correction.segment<3>(ErrorStateLayout::kGravity);
}

ErrorStateVector errorFromPrior(const NominalState & prior, const NominalState & current)
{
  ErrorStateVector error = ErrorStateVector::Zero();
  error.segment<3>(ErrorStateLayout::kRotation) = common::LogSO3(
    prior.R_WI.transpose() * current.R_WI);
  error.segment<3>(ErrorStateLayout::kPosition) = current.p_WI - prior.p_WI;
  error.segment<3>(ErrorStateLayout::kVelocity) = current.v_WI - prior.v_WI;
  error.segment<3>(ErrorStateLayout::kGyroscopeBias) = current.b_g - prior.b_g;
  error.segment<3>(ErrorStateLayout::kAccelerometerBias) = current.b_a - prior.b_a;
  error.segment<3>(ErrorStateLayout::kGravity) = current.g_W - prior.g_W;
  return error;
}

double meanResidual(
  const std::vector<common::PlaneObservation> & observations,
  const NominalState & state)
{
  if (observations.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const auto & observation : observations) {
    const Eigen::Vector3d point_I = observation.has_point_I ? observation.point_I :
      state.R_WI.transpose() * (observation.point_W - state.p_WI);
    const Eigen::Vector3d point_W = state.R_WI * point_I + state.p_WI;
    sum += std::abs(observation.normal_W.dot(point_W - observation.centroid_W));
  }
  return sum / static_cast<double>(observations.size());
}

}  // namespace

EsikfReport correctWithLidar(
  const ObservationBuilder & build_observations,
  const EsikfOptions & options,
  NominalState & state,
  StateCovariance & covariance,
  const FixedExtrinsics & extrinsics)
{
  EsikfReport report;
  if (!build_observations || !validOptions(options) || !state.isValid() ||
    !isValidCovariance(covariance) || !extrinsics.isValid())
  {
    return report;
  }

  const double measurement_variance =
    options.measurement_stddev * options.measurement_stddev;
  const bool constrain_gravity = options.update_gravity && options.gravity_magnitude_stddev > 0.0;
  const double gravity_information = constrain_gravity ?
    1.0 / (options.gravity_magnitude_stddev * options.gravity_magnitude_stddev) : 0.0;
  if (!std::isfinite(gravity_information) || (constrain_gravity && state.g_W.norm() < 1e-6)) {
    return report;
  }
  const NominalState prior_state = state;
  const StateCovariance prior_covariance = covariance;
  const auto prior_factor = prior_covariance.ldlt();
  if (prior_factor.info() != Eigen::Success) {
    return report;
  }
  const StateCovariance prior_information =
    prior_factor.solve(StateCovariance::Identity());
  if (!prior_information.allFinite()) {
    return report;
  }

  StateCovariance posterior_information = prior_information;
  Eigen::Vector3d last_rotation_correction = Eigen::Vector3d::Zero();
  std::vector<LidarLinearization> linearizations;
  std::vector<common::PlaneObservation> accepted_this_iteration;
  std::vector<double> robust_weights;
  bool have_before = false;
  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    const common::RegistrationReport scan_report = build_observations(state);
    if (scan_report.observations.empty()) {
      break;
    }

    const ErrorStateVector prior_error = errorFromPrior(prior_state, state);
    const Eigen::Matrix3d prior_chart_rotation = common::RightJacobianSO3(
      prior_error.segment<3>(ErrorStateLayout::kRotation));

    linearizations.clear();
    linearizations.reserve(scan_report.observations.size());
    accepted_this_iteration.clear();
    accepted_this_iteration.reserve(scan_report.observations.size());
    std::size_t rejected_this_iteration = 0U;
    for (const auto & observation : scan_report.observations) {
      const auto linearization = linearizeLidarObservation(observation, state, extrinsics);
      if (!linearization.jacobian.allFinite() || !std::isfinite(linearization.residual) ||
        !std::isfinite(observation.plane_weight) || observation.plane_weight <= 0.0 ||
        observation.plane_weight > 1.0) {
        ++rejected_this_iteration;
        continue;
      }
      // H is expressed at the current iterate, while P remains in the
      // propagated prior chart. d(delta_current)/d(error_prior) = Jr(phi).
      // Transport H, including its prior cross-covariance contributions.
      auto prior_chart_jacobian = linearization.jacobian;
      prior_chart_jacobian.template block<1, 3>(0, ErrorStateLayout::kRotation) =
        linearization.jacobian.template block<1, 3>(0, ErrorStateLayout::kRotation) *
        prior_chart_rotation;
      const double innovation_variance =
        (prior_chart_jacobian * prior_covariance * prior_chart_jacobian.transpose())(0, 0) +
        measurement_variance / observation.plane_weight;
      if (!std::isfinite(innovation_variance) || innovation_variance <= 0.0) {
        ++rejected_this_iteration;
        continue;
      }
      const double normalized_residual =
        linearization.residual / std::sqrt(innovation_variance);
      if (options.enable_mahalanobis_gate &&
        normalized_residual * normalized_residual > options.chi_square_threshold)
      {
        ++rejected_this_iteration;
        continue;
      }
      linearizations.push_back(linearization);
      accepted_this_iteration.push_back(observation);
    }
    report.mahalanobis_rejected += rejected_this_iteration;
    if (linearizations.empty()) {
      break;
    }
    report.effective_points = linearizations.size();
    if (!have_before) {
      double residual_sum = 0.0;
      for (const auto & observation : accepted_this_iteration) {
        residual_sum += std::abs(observation.residual);
      }
      report.mean_abs_residual_before = residual_sum /
        static_cast<double>(accepted_this_iteration.size());
      have_before = true;
    }

    StateCovariance prior_jacobian = StateCovariance::Identity();
    prior_jacobian.topLeftCorner<3, 3>() = common::RightJacobianInverseSO3(
      prior_error.segment<3>(ErrorStateLayout::kRotation));
    const StateCovariance iterated_prior_information =
      prior_jacobian.transpose() * prior_information * prior_jacobian;
    StateCovariance information = iterated_prior_information;
    ErrorStateVector rhs = ErrorStateVector::Zero();
    Eigen::Matrix<double, 6, 6> pose_information = Eigen::Matrix<double, 6, 6>::Zero();
    robust_weights.clear();
    robust_weights.reserve(linearizations.size());
    for (std::size_t index = 0; index < linearizations.size(); ++index) {
      const auto & linearization = linearizations[index];
      const double plane_weight = accepted_this_iteration[index].plane_weight;
      const double normalized_residual =
        linearization.residual * std::sqrt(plane_weight) / options.measurement_stddev;
      double robust_weight = 1.0;
      if (options.enable_huber && std::abs(normalized_residual) > options.huber_delta) {
        robust_weight = options.huber_delta / std::abs(normalized_residual);
      }
      const double weight = robust_weight * plane_weight / measurement_variance;
      robust_weights.push_back(robust_weight);
      Eigen::Matrix<double, 1, 6> pose_jacobian;
      pose_jacobian << linearization.jacobian.template block<1, 3>(
        0, ErrorStateLayout::kRotation),
        linearization.jacobian.template block<1, 3>(0, ErrorStateLayout::kPosition);
      // Point-to-plane measurements directly observe only rotation/position.
      // Keep the full joint prior and solve (velocity/bias/gravity still update
      // through cross-covariances), but skip the identically zero outer blocks.
      information.topLeftCorner<6, 6>() +=
        weight * pose_jacobian.transpose() * pose_jacobian;
      rhs.head<6>() += weight * pose_jacobian.transpose() * linearization.residual;
      pose_information += weight * pose_jacobian.transpose() * pose_jacobian;
    }

    if (options.freeze_weak_directions) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(pose_information);
      if (solver.info() == Eigen::Success) {
        const auto eigenvalues = solver.eigenvalues();
        const double largest = eigenvalues.maxCoeff();
        if (largest > 0.0 && std::isfinite(largest)) {
          Eigen::Matrix<double, 6, 1> weights = Eigen::Matrix<double, 6, 1>::Ones();
          int weak_count = 0;
          for (Eigen::Index index = 0; index < 6; ++index) {
            if (eigenvalues[index] < largest * options.weak_direction_relative_threshold) {
              weights[index] = options.weak_direction_weight;
              ++weak_count;
            }
          }
          report.weak_directions = std::max(report.weak_directions, weak_count);
          const Eigen::Matrix<double, 6, 6> attenuation =
            solver.eigenvectors() * weights.asDiagonal() * solver.eigenvectors().transpose();
          const Eigen::Matrix<double, 6, 6> constrained_information =
            solver.eigenvectors() * (eigenvalues.cwiseMax(0.0).cwiseProduct(weights)).asDiagonal() *
            solver.eigenvectors().transpose();
          // Attenuate the measurement before the joint solve. Scaling only the
          // solved pose increment leaves velocity/bias/gravity updates and the
          // posterior covariance conditioned on a correction never applied.
          information.topLeftCorner<6, 6>() += constrained_information - pose_information;
          rhs.head<6>() = attenuation * rhs.head<6>();
          pose_information = constrained_information;
        }
      }
    }

    if (constrain_gravity) {
      const double magnitude = state.g_W.norm();
      if (magnitude < 1e-6) {return EsikfReport{};}
      const Eigen::Vector3d normal = state.g_W / magnitude;
      information.bottomRightCorner<3, 3>() +=
        gravity_information * normal * normal.transpose();
      rhs.tail<3>() += gravity_information * normal * (magnitude - options.gravity_magnitude);
    }

    // The prior is centered at the propagated state, not at the current
    // iterated state. Including this term prevents repeated iterations from
    // silently turning the Kalman update into a pure least-squares solve.
    rhs += prior_jacobian.transpose() * prior_information * prior_error;

    ErrorStateVector correction = ErrorStateVector::Zero();
    if (options.update_gravity) {
      const auto information_factor = information.ldlt();
      if (information_factor.info() != Eigen::Success) {
        return EsikfReport{};
      }
      correction = information_factor.solve(-rhs);
    } else {
      constexpr int kActiveDimension = ErrorStateLayout::kGravity;
      const auto active_factor =
        information.topLeftCorner<kActiveDimension, kActiveDimension>().ldlt();
      if (active_factor.info() != Eigen::Success) {
        return EsikfReport{};
      }
      correction.head<kActiveDimension>() = active_factor.solve(-rhs.head<kActiveDimension>());
    }
    if (!correction.allFinite()) {
      return EsikfReport{};
    }

    posterior_information = information;
    // Keep the last successful report while recycling its previous storage.
    report.observation_update.accepted_observations.swap(accepted_this_iteration);
    report.observation_update.robust_weights.swap(robust_weights);
    report.observation_update.pose_information = pose_information;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> information_solver(
      pose_information);
    if (information_solver.info() == Eigen::Success) {
      report.observation_update.pose_information_eigenvalues = information_solver.eigenvalues();
      const double minimum = report.observation_update.pose_information_eigenvalues.minCoeff();
      const double maximum = report.observation_update.pose_information_eigenvalues.maxCoeff();
      report.observation_update.pose_information_condition_number =
        minimum > 0.0 ? maximum / minimum : std::numeric_limits<double>::infinity();
    }
    Eigen::Matrix<double, 6, 6> pose_covariance;
    pose_covariance.topLeftCorner<3, 3>() = prior_covariance.block<3, 3>(
      ErrorStateLayout::kRotation, ErrorStateLayout::kRotation);
    pose_covariance.topRightCorner<3, 3>() = prior_covariance.block<3, 3>(
      ErrorStateLayout::kRotation, ErrorStateLayout::kPosition);
    pose_covariance.bottomLeftCorner<3, 3>() = prior_covariance.block<3, 3>(
      ErrorStateLayout::kPosition, ErrorStateLayout::kRotation);
    pose_covariance.bottomRightCorner<3, 3>() = prior_covariance.block<3, 3>(
      ErrorStateLayout::kPosition, ErrorStateLayout::kPosition);
    pose_covariance = 0.5 * (pose_covariance + pose_covariance.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> covariance_solver(
      pose_covariance);
    if (covariance_solver.info() == Eigen::Success &&
      covariance_solver.eigenvalues().minCoeff() > 0.0)
    {
      const Eigen::Matrix<double, 6, 6> covariance_sqrt =
        covariance_solver.eigenvectors() *
        covariance_solver.eigenvalues().cwiseSqrt().asDiagonal() *
        covariance_solver.eigenvectors().transpose();
      auto & whitened = report.observation_update.whitened_pose_information;
      // Express measurement information in the prior chart before combining
      // it with the prior covariance, as in the innovation gate above.
      Eigen::Matrix<double, 6, 6> prior_to_current = Eigen::Matrix<double, 6, 6>::Identity();
      prior_to_current.topLeftCorner<3, 3>() = prior_chart_rotation;
      const Eigen::Matrix<double, 6, 6> prior_pose_information =
        prior_to_current.transpose() * pose_information * prior_to_current;
      whitened = covariance_sqrt.transpose() * prior_pose_information * covariance_sqrt;
      whitened = 0.5 * (whitened + whitened.transpose());
      report.observation_update.whitened_dof_scores = whitened.diagonal();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> whitened_solver(
        whitened);
      if (whitened_solver.info() == Eigen::Success) {
        report.observation_update.whitened_eigenvalues = whitened_solver.eigenvalues();
        report.observation_update.whitened_eigenvectors = whitened_solver.eigenvectors();
        const double whitened_minimum = whitened_solver.eigenvalues().minCoeff();
        const double whitened_maximum = whitened_solver.eigenvalues().maxCoeff();
        report.observation_update.whitened_condition_number = whitened_minimum > 0.0 ?
          whitened_maximum / whitened_minimum : std::numeric_limits<double>::infinity();
      }
    }
    report.observation_update.mahalanobis_rejected = report.mahalanobis_rejected;
    last_rotation_correction = correction.segment<3>(ErrorStateLayout::kRotation);
    applyCorrection(state, correction);
    report.rotation_correction = correction.segment<3>(ErrorStateLayout::kRotation).norm();
    report.translation_correction = correction.segment<3>(ErrorStateLayout::kPosition).norm();
    report.iterations = iteration + 1;
    if (report.rotation_correction <= options.rotation_convergence &&
      report.translation_correction <= options.translation_convergence)
    {
      report.converged = true;
      break;
    }
  }

  if (report.iterations == 0 || !posterior_information.allFinite()) {
    return report;
  }
  if (options.update_gravity) {
    const auto posterior_factor = posterior_information.ldlt();
    if (posterior_factor.info() != Eigen::Success) {
      return EsikfReport{};
    }
    covariance = posterior_factor.solve(StateCovariance::Identity());
  } else {
    constexpr int kActiveDimension = ErrorStateLayout::kGravity;
    const auto active_factor =
      posterior_information.topLeftCorner<kActiveDimension, kActiveDimension>().ldlt();
    if (active_factor.info() != Eigen::Success) {
      return EsikfReport{};
    }
    StateCovariance constrained_covariance = StateCovariance::Zero();
    constrained_covariance.topLeftCorner<kActiveDimension, kActiveDimension>() =
      active_factor.solve(Eigen::Matrix<double, kActiveDimension, kActiveDimension>::Identity());
    constrained_covariance.bottomRightCorner<ErrorStateLayout::kBlockSize,
      ErrorStateLayout::kBlockSize>() = prior_covariance.bottomRightCorner<
      ErrorStateLayout::kBlockSize, ErrorStateLayout::kBlockSize>();
    covariance = constrained_covariance;
  }
  // The Hessian above describes right perturbations at the last linearization
  // state. After injecting dtheta, express that uncertainty at R*Exp(dtheta):
  // Log(Exp(-dtheta)*Exp(dtheta+epsilon)) = Jr(dtheta)*epsilon.
  // Transport cross-covariances as well; velocity/bias/gravity use additive
  // coordinates and therefore have identity reset blocks.
  StateCovariance reset = StateCovariance::Identity();
  reset.topLeftCorner<3, 3>() = common::RightJacobianSO3(last_rotation_correction);
  covariance = reset * covariance * reset.transpose();
  covariance = (covariance + covariance.transpose()) * 0.5;
  report.mean_abs_residual_after = meanResidual(
    report.observation_update.accepted_observations, state);
  return state.isValid() && isValidCovariance(covariance) ? report : EsikfReport{};
}

EsikfReport correctWithLidar(
  const std::vector<common::PlaneObservation> & observations,
  const EsikfOptions & options,
  NominalState & state,
  StateCovariance & covariance,
  const FixedExtrinsics & extrinsics)
{
  const ObservationBuilder builder = [&observations](const NominalState &) {
      common::RegistrationReport report;
      report.input_points = observations.size();
      report.effective_points = observations.size();
      report.observations = observations;
      return report;
    };
  return correctWithLidar(builder, options, state, covariance, extrinsics);
}

}  // namespace itera_lio::estimator
