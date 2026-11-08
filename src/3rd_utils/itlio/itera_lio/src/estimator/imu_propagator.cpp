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

#include "itera_lio/estimator/imu_propagator.hpp"

#include <Eigen/Cholesky>

#include <cmath>
#include <stdexcept>

#include "itera_lio/common/so3.hpp"

namespace itera_lio::estimator
{

ImuPropagator::ImuPropagator(const ImuNoiseParameters & noise)
: noise_(noise)
{
  if (!std::isfinite(noise_.gyro_noise_density) || noise_.gyro_noise_density < 0.0 ||
    !std::isfinite(noise_.accel_noise_density) || noise_.accel_noise_density < 0.0 ||
    !std::isfinite(noise_.gyro_bias_noise_density) || noise_.gyro_bias_noise_density < 0.0 ||
    !std::isfinite(noise_.accel_bias_noise_density) || noise_.accel_bias_noise_density < 0.0)
  {
    throw std::invalid_argument("IMU noise densities must be finite and non-negative");
  }
}

bool ImuPropagator::propagate(
  const sensor::ImuData & previous,
  const sensor::ImuData & current,
  NominalState & state,
  StateCovariance & covariance) const
{
  const double dt = current.timestamp - previous.timestamp;
  if (!previous.isFinite() || !current.isFinite() || !std::isfinite(dt) || dt <= 0.0 ||
    !state.isValid() || !isValidCovariance(covariance))
  {
    return false;
  }

  const Eigen::Vector3d gyro =
    0.5 * (previous.gyro + current.gyro) - state.b_g;
  const Eigen::Vector3d accel =
    0.5 * (previous.accel + current.accel) - state.b_a;
  const Eigen::Matrix3d R_mid = state.R_WI * common::ExpSO3(0.5 * gyro * dt);
  const Eigen::Vector3d world_accel = R_mid * accel + state.g_W;
  state.p_WI += state.v_WI * dt + 0.5 * world_accel * dt * dt;
  state.v_WI += world_accel * dt;
  state.R_WI = state.R_WI * common::ExpSO3(gyro * dt);

  constexpr int kNoiseDimension = 12;
  Eigen::Matrix<double, ErrorStateLayout::kDimension, ErrorStateLayout::kDimension> F =
    Eigen::Matrix<double, ErrorStateLayout::kDimension, ErrorStateLayout::kDimension>::Identity();
  Eigen::Matrix<double, ErrorStateLayout::kDimension, kNoiseDimension> G =
    Eigen::Matrix<double, ErrorStateLayout::kDimension, kNoiseDimension>::Zero();
  const int r = ErrorStateLayout::kRotation;
  const int p = ErrorStateLayout::kPosition;
  const int v = ErrorStateLayout::kVelocity;
  const int bg = ErrorStateLayout::kGyroscopeBias;
  const int ba = ErrorStateLayout::kAccelerometerBias;
  const int g = ErrorStateLayout::kGravity;
  F.block<3, 3>(r, r) -= common::Hat(gyro) * dt;
  F.block<3, 3>(r, bg) = -Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(p, v) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(v, r) = -R_mid * common::Hat(accel) * dt;
  F.block<3, 3>(v, ba) = -R_mid * dt;
  F.block<3, 3>(v, g) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(r, r) = common::ExpSO3(-gyro * dt);
  G.block<3, 3>(r, 0) = -Eigen::Matrix3d::Identity() * dt;
  G.block<3, 3>(v, 3) = -R_mid * dt;
  G.block<3, 3>(bg, 6) = Eigen::Matrix3d::Identity() * dt;
  G.block<3, 3>(ba, 9) = Eigen::Matrix3d::Identity() * dt;
  Eigen::Matrix<double, kNoiseDimension, kNoiseDimension> Q =
    Eigen::Matrix<double, kNoiseDimension, kNoiseDimension>::Zero();
  // G already contains the integration interval. For continuous white-noise
  // densities, the equivalent discrete covariance is density^2 / dt.
  const double inverse_dt = 1.0 / dt;
  Q.block<3, 3>(0, 0).diagonal().setConstant(
    noise_.gyro_noise_density * noise_.gyro_noise_density * inverse_dt);
  Q.block<3, 3>(3, 3).diagonal().setConstant(
    noise_.accel_noise_density * noise_.accel_noise_density * inverse_dt);
  Q.block<3, 3>(6, 6).diagonal().setConstant(
    noise_.gyro_bias_noise_density * noise_.gyro_bias_noise_density * inverse_dt);
  Q.block<3, 3>(9, 9).diagonal().setConstant(
    noise_.accel_bias_noise_density * noise_.accel_bias_noise_density * inverse_dt);
  covariance = F * covariance * F.transpose() + G * Q * G.transpose();
  covariance = 0.5 * (covariance + covariance.transpose());
  return state.isValid() && isValidCovariance(covariance);
}

}  // namespace itera_lio::estimator
