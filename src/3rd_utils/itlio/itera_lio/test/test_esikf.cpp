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

#include <cmath>
#include "itera_lio/common/so3.hpp"
#include <Eigen/LU>

#include "itera_lio/estimator/esikf.hpp"

TEST(Esikf, ReducesPositionPlaneResidual)
{
  itera_lio::estimator::NominalState state;
  state.p_WI.z() = 0.2;
  itera_lio::estimator::StateCovariance covariance =
    itera_lio::estimator::StateCovariance::Identity();
  const double prior_trace = covariance.trace();
  itera_lio::common::PlaneObservation observation;
  observation.point_W = Eigen::Vector3d(0.0, 0.0, 0.0);
  observation.normal_W = Eigen::Vector3d(0.0, 0.0, 1.0);
  observation.centroid_W = Eigen::Vector3d(0.0, 0.0, 0.0);
  observation.residual = 0.2;
  itera_lio::estimator::EsikfOptions options;
  options.max_iterations = 3;
  const auto report = itera_lio::estimator::correctWithLidar(
    {observation}, options, state, covariance);
  EXPECT_GT(report.iterations, 0);
  EXPECT_LT(report.mean_abs_residual_after, report.mean_abs_residual_before);
  EXPECT_LT(state.p_WI.z(), 0.2);
  EXPECT_TRUE(itera_lio::estimator::isValidCovariance(covariance));
  EXPECT_LT(covariance.trace(), prior_trace);
  ASSERT_EQ(report.observation_update.accepted_observations.size(), 1U);
  ASSERT_EQ(report.observation_update.robust_weights.size(), 1U);
  EXPECT_NEAR(report.observation_update.robust_weights.front(), 0.625, 1e-12);
  EXPECT_TRUE(report.observation_update.pose_information.allFinite());
  EXPECT_TRUE(report.observation_update.pose_information_eigenvalues.allFinite());
  EXPECT_TRUE(std::isinf(report.observation_update.pose_information_condition_number));
  EXPECT_TRUE(report.observation_update.whitened_pose_information.allFinite());
  EXPECT_TRUE(report.observation_update.whitened_eigenvalues.allFinite());
  EXPECT_GT(report.observation_update.whitened_dof_scores[5], 0.0);
  EXPECT_TRUE(std::isinf(report.observation_update.whitened_condition_number));
}

TEST(Esikf, InjectedRotationTransportsPosteriorAndCrossCovariance)
{
  namespace estimator = itera_lio::estimator;
  namespace common = itera_lio::common;
  estimator::NominalState state;
  estimator::StateCovariance prior = estimator::StateCovariance::Identity() * 0.04;
  prior(0, 12) = prior(12, 0) = 0.012;
  prior(1, 16) = prior(16, 1) = -0.009;
  common::PlaneObservation observation;
  observation.has_point_I = true;
  observation.point_I = Eigen::Vector3d(1.0, -0.7, 0.4);
  observation.normal_W = Eigen::Vector3d(0.2, 0.5, 0.8).normalized();
  observation.centroid_W = observation.point_I - 0.4 * observation.normal_W;
  const auto h = estimator::linearizeLidarObservation(observation, state, {}).jacobian;
  estimator::EsikfOptions options;
  options.max_iterations = 1;
  options.enable_huber = options.enable_mahalanobis_gate = false;
  options.freeze_weak_directions = false;
  options.gravity_magnitude_stddev = 0.0;
  options.update_gravity = true;
  estimator::StateCovariance posterior = prior;
  const auto report = estimator::correctWithLidar({observation}, options, state, posterior);
  ASSERT_EQ(report.iterations, 1);
  const Eigen::Vector3d injected = common::LogSO3(state.R_WI);
  ASSERT_GT(injected.norm(), 0.1);
  // Oracle: differentiate the actual retraction, independently of Jr().
  Eigen::Matrix3d numerical_reset;
  constexpr double step = 1e-6;
  for (int axis = 0; axis < 3; ++axis) {
    const Eigen::Vector3d delta = step * Eigen::Vector3d::Unit(axis);
    numerical_reset.col(axis) =
      (common::LogSO3(state.R_WI.transpose() * common::ExpSO3(injected + delta)) -
      common::LogSO3(state.R_WI.transpose() * common::ExpSO3(injected - delta))) / (2 * step);
  }
  estimator::StateCovariance transport = estimator::StateCovariance::Identity();
  transport.topLeftCorner<3, 3>() = numerical_reset;
  const estimator::StateCovariance before_injection =
    (prior.inverse() + h.transpose() * h /
    (options.measurement_stddev * options.measurement_stddev)).inverse();
  const estimator::StateCovariance expected =
    transport * before_injection * transport.transpose();
  EXPECT_LT((posterior - expected).norm(), 1e-9);
  EXPECT_GT((expected - before_injection).norm(), 1e-4);
  EXPECT_TRUE(estimator::isValidCovariance(posterior));
}

TEST(Esikf, RebuildsObservationsForEachIteration)
{
  itera_lio::estimator::NominalState state;
  state.p_WI.z() = 0.2;
  itera_lio::estimator::StateCovariance covariance =
    itera_lio::estimator::StateCovariance::Identity();
  int builder_calls = 0;
  itera_lio::estimator::EsikfOptions options;
  options.max_iterations = 4;
  options.freeze_weak_directions = false;
  const auto report = itera_lio::estimator::correctWithLidar(
    [&builder_calls](const itera_lio::estimator::NominalState & current) {
      ++builder_calls;
      itera_lio::common::RegistrationReport scan_report;
      itera_lio::common::PlaneObservation observation;
      observation.point_I = Eigen::Vector3d::Zero();
      observation.point_W = current.p_WI;
      observation.normal_W = Eigen::Vector3d::UnitZ();
      observation.centroid_W = Eigen::Vector3d::Zero();
      observation.residual = current.p_WI.z();
      observation.has_point_I = true;
      scan_report.observations.push_back(observation);
      scan_report.effective_points = 1U;
      return scan_report;
    }, options, state, covariance);
  EXPECT_GE(builder_calls, 2);
  EXPECT_EQ(report.iterations, builder_calls);
  EXPECT_LT(report.mean_abs_residual_after, report.mean_abs_residual_before);
}

TEST(Esikf, KeepsThePriorInTheIteratedPosterior)
{
  itera_lio::estimator::NominalState state;
  state.p_WI.z() = 0.2;
  itera_lio::estimator::StateCovariance covariance =
    itera_lio::estimator::StateCovariance::Identity();
  itera_lio::estimator::EsikfOptions options;
  options.max_iterations = 4;
  options.freeze_weak_directions = false;

  const auto report = itera_lio::estimator::correctWithLidar(
    [](const itera_lio::estimator::NominalState & current) {
      itera_lio::common::RegistrationReport scan_report;
      itera_lio::common::PlaneObservation observation;
      observation.point_I = Eigen::Vector3d::Zero();
      observation.point_W = current.p_WI;
      observation.normal_W = Eigen::Vector3d::UnitZ();
      observation.centroid_W = Eigen::Vector3d::Zero();
      observation.residual = current.p_WI.z();
      observation.has_point_I = true;
      scan_report.observations.push_back(observation);
      scan_report.effective_points = 1U;
      return scan_report;
    }, options, state, covariance);

  EXPECT_GT(report.iterations, 1);
  // P / (P + R) * 0.2 with P=1 and R=0.05^2 is approximately 0.0005.
  EXPECT_NEAR(state.p_WI.z(), 0.0005, 0.002);
}

TEST(Esikf, KeepsLastSuccessfulObservationsWhenLaterIterationRejectsAll)
{
  namespace estimator = itera_lio::estimator;
  namespace common = itera_lio::common;
  estimator::NominalState state;
  state.p_WI.z() = 0.2;
  auto covariance = estimator::StateCovariance::Identity().eval();
  estimator::EsikfOptions options;
  options.max_iterations = 4;
  options.freeze_weak_directions = false;
  options.rotation_convergence = 0.0;
  options.translation_convergence = 0.0;
  int calls = 0;
  const auto report = estimator::correctWithLidar(
    [&calls](const estimator::NominalState & current) {
      common::RegistrationReport scan;
      common::PlaneObservation observation;
      observation.has_point_I = true;
      observation.normal_W = Eigen::Vector3d::UnitZ();
      observation.point_W = current.p_WI;
      observation.residual = current.p_WI.z();
      observation.intensity = static_cast<float>(++calls);
      if (calls == 2) {
        observation.centroid_W.z() = 0.01;
        observation.residual -= 0.01;
      }
      if (calls == 3) {observation.plane_weight = 0.0;}
      scan.observations.push_back(observation);
      return scan;
    }, options, state, covariance);
  ASSERT_EQ(calls, 3);
  ASSERT_EQ(report.iterations, 2);
  ASSERT_EQ(report.observation_update.accepted_observations.size(), 1U);
  EXPECT_FLOAT_EQ(report.observation_update.accepted_observations.front().intensity, 2.0F);
  ASSERT_EQ(report.observation_update.robust_weights.size(), 1U);
  EXPECT_NEAR(report.mean_abs_residual_after, std::abs(state.p_WI.z() - 0.01), 1e-12);
  EXPECT_TRUE(estimator::isValidCovariance(covariance));
}

TEST(Esikf, FreezesGravityWhenLidarUpdateIsDisabled)
{
  itera_lio::estimator::NominalState state;
  state.p_WI.z() = 0.2;
  state.g_W = Eigen::Vector3d(0.0, 0.0, -9.81);
  itera_lio::estimator::StateCovariance covariance =
    itera_lio::estimator::StateCovariance::Identity();
  covariance(2, itera_lio::estimator::ErrorStateLayout::kGravity + 2) = 0.4;
  covariance(itera_lio::estimator::ErrorStateLayout::kGravity + 2, 2) = 0.4;
  const Eigen::Vector3d gravity_before = state.g_W;

  itera_lio::common::PlaneObservation observation;
  observation.point_W = Eigen::Vector3d::Zero();
  observation.normal_W = Eigen::Vector3d::UnitZ();
  observation.centroid_W = Eigen::Vector3d::Zero();
  observation.residual = state.p_WI.z();

  itera_lio::estimator::EsikfOptions options;
  options.max_iterations = 1;
  options.freeze_weak_directions = false;
  options.update_gravity = false;
  const auto report = itera_lio::estimator::correctWithLidar(
    {observation}, options, state, covariance);

  EXPECT_EQ(report.iterations, 1);
  EXPECT_TRUE(state.g_W.isApprox(gravity_before, 1e-12));
  EXPECT_LT(state.p_WI.z(), 0.2);
  EXPECT_TRUE(itera_lio::estimator::isValidCovariance(covariance));
}

TEST(Esikf, UncertainPlaneRetainsMorePriorUncertainty)
{
  namespace estimator = itera_lio::estimator;
  namespace common = itera_lio::common;
  estimator::NominalState strong_state;
  strong_state.p_WI.z() = 0.1;
  auto weak_state = strong_state;
  estimator::StateCovariance strong_covariance =
    estimator::StateCovariance::Identity() * 0.01;
  auto weak_covariance = strong_covariance;
  common::PlaneObservation observation;
  observation.normal_W = Eigen::Vector3d::UnitZ();
  observation.has_point_I = true;
  estimator::EsikfOptions options;
  options.max_iterations = 1;
  options.enable_huber = false;
  options.enable_mahalanobis_gate = false;
  options.freeze_weak_directions = false;
  const auto strong = estimator::correctWithLidar(
    {observation}, options, strong_state, strong_covariance);
  observation.plane_weight = 0.1;
  const auto weak = estimator::correctWithLidar(
    {observation}, options, weak_state, weak_covariance);
  ASSERT_EQ(strong.iterations, 1);
  ASSERT_EQ(weak.iterations, 1);
  EXPECT_GT(weak_state.p_WI.z(), strong_state.p_WI.z());
  const int z = estimator::ErrorStateLayout::kPosition + 2;
  EXPECT_GT(weak_covariance(z, z), strong_covariance(z, z));
}

TEST(Esikf, GravityMagnitudePriorUpdatesCorrelatedStateAndCovariance)
{
  namespace estimator = itera_lio::estimator;
  estimator::NominalState state;
  state.g_W = Eigen::Vector3d(0.0, 0.0, -9.3);
  state.p_WI.z() = 3.0;
  auto covariance = estimator::StateCovariance::Identity().eval();
  const int g = estimator::ErrorStateLayout::kGravity + 2;
  const int bias = estimator::ErrorStateLayout::kAccelerometerBias + 2;
  covariance(g, bias) = covariance(bias, g) = 0.4;
  itera_lio::common::PlaneObservation observation;
  observation.has_point_I = true;
  observation.normal_W = Eigen::Vector3d::UnitX();
  estimator::EsikfOptions options;
  options.max_iterations = 1;
  options.update_gravity = true;
  options.gravity_magnitude_stddev = 0.05;
  options.freeze_weak_directions = false;
  const auto report = estimator::correctWithLidar({observation}, options, state, covariance);
  ASSERT_EQ(report.iterations, 1);
  const double expected_change = -0.51 / 1.0025;
  EXPECT_NEAR(state.g_W.z(), -9.3 + expected_change, 1e-10);
  EXPECT_NEAR(state.b_a.z(), 0.4 * expected_change, 1e-10);
  EXPECT_NEAR(covariance(g, g), 0.0025 / 1.0025, 1e-10);
  EXPECT_NEAR(state.p_WI.z(), 3.0, 1e-12);
  EXPECT_TRUE(estimator::isValidCovariance(covariance));
}

TEST(Esikf, GravityMagnitudePriorPreservesDirectionAndDoesNotConstrainAltitude)
{
  namespace estimator = itera_lio::estimator;
  for (const double height : {-12.0, 0.0, 12.0}) {
    estimator::NominalState state;
    const Eigen::Vector3d direction = Eigen::Vector3d(0.1, -0.2, -1.0).normalized();
    state.g_W = 9.81 * direction;
    state.p_WI.z() = height;
    auto covariance = estimator::StateCovariance::Identity().eval();
    itera_lio::common::PlaneObservation observation;
    observation.has_point_I = true;
    observation.normal_W = Eigen::Vector3d::UnitX();
    estimator::EsikfOptions options;
    options.update_gravity = true;
    options.gravity_magnitude_stddev = 0.05;
    const auto report = estimator::correctWithLidar({observation}, options, state, covariance);
    ASSERT_GT(report.iterations, 0);
    EXPECT_TRUE(state.g_W.isApprox(9.81 * direction, 1e-12));
    EXPECT_NEAR(state.p_WI.z(), height, 1e-12);
  }
}

TEST(Esikf, WeakMeasurementUsesConsistentPoseBiasAndCovariance)
{
  namespace estimator = itera_lio::estimator;
  estimator::NominalState state;
  state.p_WI.z() = 0.2;
  auto covariance = estimator::StateCovariance::Identity().eval();
  const int z = estimator::ErrorStateLayout::kPosition + 2;
  const int bias = estimator::ErrorStateLayout::kAccelerometerBias + 2;
  covariance(z, bias) = covariance(bias, z) = 0.4;
  std::vector<itera_lio::common::PlaneObservation> observations(101);
  for (auto & observation : observations) {
    observation.has_point_I = true;
    observation.normal_W = Eigen::Vector3d::UnitX();
  }
  observations.back().normal_W = Eigen::Vector3d::UnitZ();
  observations.back().plane_weight = 0.01;
  estimator::EsikfOptions options;
  options.max_iterations = 1;
  options.measurement_stddev = 1.0;
  options.enable_huber = options.enable_mahalanobis_gate = false;
  options.freeze_weak_directions = true;
  options.weak_direction_relative_threshold = 0.001;
  options.weak_direction_weight = 0.1;
  const auto report = estimator::correctWithLidar(observations, options, state, covariance);
  ASSERT_EQ(report.iterations, 1);
  EXPECT_GT(report.weak_directions, 0);
  EXPECT_NEAR(state.p_WI.z(), 0.2 / 1.001, 1e-12);
  EXPECT_NEAR(state.b_a.z(), 0.4 * (state.p_WI.z() - 0.2), 1e-12);
  EXPECT_NEAR(covariance(z, z), 1.0 / 1.001, 1e-12);
  EXPECT_NEAR(covariance(z, bias), 0.4 / 1.001, 1e-12);
  EXPECT_TRUE(estimator::isValidCovariance(covariance));
}

TEST(Esikf, IteratedRotationPriorMatchesNumericalObjectiveGradient){
  namespace estimator = itera_lio::estimator;
  namespace common = itera_lio::common;
  estimator::NominalState state;
  auto covariance = estimator::StateCovariance::Identity().eval();
  covariance *= 0.001;
  covariance.topLeftCorner<3, 3>() << 0.04, 0.006, -0.002,
    0.006, 0.02, 0.003, -0.002, 0.003, 0.01;
  const auto prior_covariance = covariance;
  const Eigen::Matrix3d target = common::ExpSO3(Eigen::Vector3d(0.3, -0.2, 0.15));
  std::vector<common::PlaneObservation> observations;
  const std::vector<Eigen::Vector3d> points = {
    {1.0, 0.2, -0.3}, {-0.3, 1.2, 0.4}, {0.2, -0.5, 1.1}, {-0.7, -0.8, 0.5}};
  for (const auto & point : points) {
    for (int axis = 0; axis < 3; ++axis) {
      common::PlaneObservation observation;
      observation.has_point_I = true;
      observation.point_I = point;
      observation.centroid_W = target * point;
      observation.normal_W = Eigen::Vector3d::Unit(axis);
      observations.push_back(observation);
    }
  }
  estimator::EsikfOptions options;
  options.max_iterations = 30;
  options.measurement_stddev = 0.1;
  options.freeze_weak_directions = false;
  options.enable_huber = false;
  options.enable_mahalanobis_gate = false;
  options.rotation_convergence = 1e-12;
  options.translation_convergence = 1e-12;
  const auto report = estimator::correctWithLidar(observations, options, state, covariance);
  ASSERT_GT(report.iterations, 1);
  const estimator::StateCovariance prior_information = prior_covariance.inverse();
  const auto objective = [&](const Eigen::Matrix<double, 6, 1> & increment) {
      auto perturbed = state;
      perturbed.R_WI *= common::ExpSO3(increment.head<3>());
      perturbed.p_WI += increment.tail<3>();
      estimator::ErrorStateVector error = estimator::ErrorStateVector::Zero();
      error.head<3>() = common::LogSO3(perturbed.R_WI);
      error.segment<3>(3) = perturbed.p_WI;
      double cost = 0.5 * error.dot(prior_information * error);
      for (const auto & observation : observations) {
        const double residual = observation.normal_W.dot(
          perturbed.R_WI * observation.point_I + perturbed.p_WI - observation.centroid_W);
        cost += 0.5 * residual * residual / 0.01;
      }
      return cost;
    };
  Eigen::Matrix<double, 6, 1> gradient;
  constexpr double step = 1e-6;
  for (int i = 0; i < 6; ++i) {
    auto delta = Eigen::Matrix<double, 6, 1>::Zero().eval();
    delta[i] = step;
    gradient[i] = (objective(delta) - objective(-delta)) / (2.0 * step);
  }
  EXPECT_LT(gradient.norm(), 1e-5);
}


TEST(Esikf, IteratedGateMatchesNumericalPriorChartVariance)
{
  namespace estimator = itera_lio::estimator;
  namespace common = itera_lio::common;
  estimator::NominalState state;
  estimator::StateCovariance prior = estimator::StateCovariance::Identity() * 0.001;
  prior.topLeftCorner<3, 3>() << 0.08, 0.01, -0.004,
    0.01, 0.02, 0.003, -0.004, 0.003, 0.006;
  estimator::StateCovariance posterior = prior;
  estimator::EsikfOptions options;
  options.max_iterations = 2;
  options.enable_huber = options.freeze_weak_directions = false;
  options.rotation_convergence = options.translation_convergence = 0.0;
  int calls = 0;
  bool expected_rejection = false;
  const auto report = estimator::correctWithLidar(
    [&](const estimator::NominalState & current) {
      common::RegistrationReport scan;
      common::PlaneObservation observation;
      observation.has_point_I = true;
      observation.point_I = Eigen::Vector3d(1.2, -0.7, 0.4);
      observation.normal_W = Eigen::Vector3d(0.3, 0.5, 0.8).normalized();
      observation.point_W = current.R_WI * observation.point_I + current.p_WI;
      if (++calls == 1) {
        observation.centroid_W = observation.point_W - 0.3 * observation.normal_W;
      } else {
        const Eigen::Vector3d phi = common::LogSO3(current.R_WI);
        EXPECT_GT(phi.norm(), 0.01);
        // Numerically differentiate in the PRIOR rotation chart without using
        // the analytic SO(3) Jacobian under test.
        Eigen::Matrix<double, 1, 18> numerical = Eigen::Matrix<double, 1, 18>::Zero();
        constexpr double step = 1e-6;
        for (int axis = 0; axis < 3; ++axis) {
          const Eigen::Vector3d delta = step * Eigen::Vector3d::Unit(axis);
          numerical[axis] = observation.normal_W.dot(
            (common::ExpSO3(phi + delta) - common::ExpSO3(phi - delta)) *
            observation.point_I) / (2.0 * step);
        }
        numerical.segment<3>(3) = observation.normal_W.transpose();
        const auto linearization = estimator::linearizeLidarObservation(observation, current, {});
        const double noise = options.measurement_stddev * options.measurement_stddev;
        const double numeric_variance = (numerical * prior * numerical.transpose())(0, 0) + noise;
        const double old_variance =
          (linearization.jacobian * prior * linearization.jacobian.transpose())(0, 0) + noise;
        EXPECT_GT(std::abs(numeric_variance - old_variance), 1e-5);
        const double residual = std::sqrt(
          options.chi_square_threshold * 0.5 * (numeric_variance + old_variance));
        expected_rejection = residual * residual > options.chi_square_threshold * numeric_variance;
        observation.centroid_W = observation.point_W - residual * observation.normal_W;
      }
      scan.observations.push_back(observation);
      return scan;
    }, options, state, posterior);
  ASSERT_EQ(calls, 2);
  EXPECT_EQ(report.mahalanobis_rejected, expected_rejection ? 1U : 0U);
  EXPECT_EQ(report.iterations, expected_rejection ? 1 : 2);
}

TEST(Esikf, WhitenedInformationMatchesNumericalPriorChart)
{
  namespace estimator = itera_lio::estimator;
  namespace common = itera_lio::common;
  estimator::NominalState state;
  estimator::StateCovariance prior = estimator::StateCovariance::Identity() * 0.001;
  prior.topLeftCorner<3, 3>() << 0.08, 0.01, -0.004,
    0.01, 0.02, 0.003, -0.004, 0.003, 0.006;
  prior(0, 3) = prior(3, 0) = 0.001;
  auto posterior = prior;
  estimator::EsikfOptions options;
  options.max_iterations = 2;
  options.enable_huber = options.enable_mahalanobis_gate = false;
  options.freeze_weak_directions = false;
  options.rotation_convergence = options.translation_convergence = 0.0;
  Eigen::Matrix<double, 6, 6> numerical_information;
  int calls = 0;
  const auto report = estimator::correctWithLidar(
    [&](const estimator::NominalState & current) {
      common::RegistrationReport scan;
      numerical_information.setZero();
      for (const Eigen::Vector3d & point : std::vector<Eigen::Vector3d>{
          {1.2, -0.7, 0.4}, {-0.3, 1.0, 0.7}, {0.5, 0.3, -1.0}}) {
        for (int normal_axis = 0; normal_axis < 3; ++normal_axis) {
          common::PlaneObservation observation;
          observation.has_point_I = true;
          observation.point_I = point;
          observation.normal_W = Eigen::Vector3d::Unit(normal_axis);
          observation.centroid_W = common::ExpSO3(Eigen::Vector3d(0.3, -0.2, 0.1)) * point;
          scan.observations.push_back(observation);
          const Eigen::Vector3d phi = common::LogSO3(current.R_WI);
          Eigen::Matrix<double, 1, 6> numerical;
          constexpr double step = 1e-6;
          for (int axis = 0; axis < 3; ++axis) {
            const Eigen::Vector3d delta = step * Eigen::Vector3d::Unit(axis);
            numerical[axis] = observation.normal_W.dot(
              (common::ExpSO3(phi + delta) - common::ExpSO3(phi - delta)) * point) /
              (2.0 * step);
          }
          numerical.tail<3>() = observation.normal_W.transpose();
          numerical_information += numerical.transpose() * numerical /
            (options.measurement_stddev * options.measurement_stddev);
        }
      }
      if (++calls == 2) {EXPECT_GT(common::LogSO3(current.R_WI).norm(), 0.01);}
      return scan;
    }, options, state, posterior);
  ASSERT_EQ(report.iterations, 2);
  // trace(P I) is invariant under consistent changes of tangent coordinates.
  // This oracle differentiates Exp directly, not the analytic Jacobian under test.
  const double expected_trace = (prior.topLeftCorner<6, 6>() * numerical_information).trace();
  EXPECT_NEAR(report.observation_update.whitened_pose_information.trace(), expected_trace, 1e-7);
}
