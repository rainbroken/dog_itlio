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

#include "itera_lio/estimator/imu_initializer.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>

namespace itera_lio::estimator
{

ImuInitializer::ImuInitializer(const ImuInitializerOptions & options)
: options_(options)
{
  if (!std::isfinite(options_.duration_sec) || options_.duration_sec <= 0.0 ||
    !std::isfinite(options_.gyro_std_threshold) || options_.gyro_std_threshold < 0.0 ||
    !std::isfinite(options_.accel_std_threshold) || options_.accel_std_threshold < 0.0 ||
    !std::isfinite(options_.gyro_mean_threshold) || options_.gyro_mean_threshold < 0.0 ||
    !std::isfinite(options_.accel_norm_threshold) || options_.accel_norm_threshold < 0.0 ||
    !std::isfinite(options_.gravity_magnitude) || options_.gravity_magnitude <= 0.0 ||
    options_.min_samples == 0U)
  {
    throw std::invalid_argument("invalid IMU initializer options");
  }
}

ImuInitializationStatus ImuInitializer::addSample(const sensor::ImuData & sample)
{
  if (initialized_) {
    return ImuInitializationStatus::kInitialized;
  }
  if (!sample.isFinite()) {
    return ImuInitializationStatus::kRejectedInvalidSample;
  }
  if (has_timestamp_ && sample.timestamp < last_timestamp_) {
    return ImuInitializationStatus::kRejectedTimestampRegression;
  }

  if (!has_timestamp_) {
    first_timestamp_ = sample.timestamp;
    window_end_timestamp_ = first_timestamp_ + options_.duration_sec;
    has_timestamp_ = true;
  }

  // Keep the sample statistics tied to the fixed interval that started at
  // the first accepted sample. The first sample after the interval is only a
  // trigger; including it would make initialization depend on IMU rate.
  constexpr double kWindowTolerance = 1e-12;
  if (sample.timestamp > window_end_timestamp_ + kWindowTolerance) {
    if (sample_count_ >= options_.min_samples && stationary() && initializeFromWindow()) {
      return ImuInitializationStatus::kInitialized;
    }
    const double timestamp = sample.timestamp;
    reset();
    first_timestamp_ = timestamp;
    last_timestamp_ = timestamp;
    window_end_timestamp_ = timestamp + options_.duration_sec;
    has_timestamp_ = true;
    accumulate(sample);
    return ImuInitializationStatus::kCollecting;
  }

  last_timestamp_ = sample.timestamp;
  accumulate(sample);

  // A moving platform must provide a fresh, contiguous stationary window.
  if (sample_count_ >= 2U && !stationary()) {
    const double timestamp = sample.timestamp;
    reset();
    first_timestamp_ = timestamp;
    last_timestamp_ = timestamp;
    window_end_timestamp_ = timestamp + options_.duration_sec;
    has_timestamp_ = true;
    accumulate(sample);
    return ImuInitializationStatus::kCollecting;
  }

  if (sample_count_ >= options_.min_samples &&
    sample.timestamp >= window_end_timestamp_ && stationary())
  {
    initializeFromWindow();
  }
  return initialized_ ? ImuInitializationStatus::kInitialized :
         ImuInitializationStatus::kCollecting;
}

bool ImuInitializer::initializeFromWindow()
{
  const double direction_norm = sum_accel_direction_.norm();
  if (direction_norm <= 1e-9) {
    return false;
  }
  // Retain the configured-magnitude initialization for explicit calibrated
  // use. In the free-magnitude mode align the measured mean with world Z so
  // the initial gravity and the zero-bias stationary dynamics agree exactly.
  const double mean_norm = stats_.mean_accel.norm();
  if (mean_norm <= 1e-9) {return false;}
  const Eigen::Vector3d accel_direction = options_.initialize_accel_bias_from_static_mean ?
    (sum_accel_direction_ / direction_norm).eval() : (stats_.mean_accel / mean_norm).eval();
  const Eigen::Quaterniond gravity_alignment = Eigen::Quaterniond::FromTwoVectors(
    accel_direction, Eigen::Vector3d::UnitZ());
  initialized_state_.R_WI = gravity_alignment.toRotationMatrix();
  initialized_state_.p_WI.setZero();
  initialized_state_.v_WI.setZero();
  initialized_state_.b_g = stats_.mean_gyro;
  if (options_.initialize_accel_bias_from_static_mean) {
    initialized_state_.b_a =
      stats_.mean_accel - options_.gravity_magnitude * accel_direction;
  } else {
    // A single static orientation cannot separate accelerometer bias from
    // gravity. Start with zero bias and estimate gravity from the measured
    // mean rather than introducing the configured magnitude implicitly.
    initialized_state_.b_a.setZero();
  }
  initialized_state_.g_W = options_.initialize_accel_bias_from_static_mean ?
    (-initialized_state_.R_WI * accel_direction * options_.gravity_magnitude).eval() :
    (-initialized_state_.R_WI * stats_.mean_accel).eval();
  initialized_ = initialized_state_.R_WI.allFinite() && initialized_state_.g_W.allFinite();
  return initialized_;
}

void ImuInitializer::reset()
{
  initialized_ = false;
  first_timestamp_ = 0.0;
  last_timestamp_ = 0.0;
  window_end_timestamp_ = 0.0;
  has_timestamp_ = false;
  sum_accel_direction_.setZero();
  gyro_m2_.setZero();
  accel_m2_.setZero();
  sample_count_ = 0U;
  stats_ = ImuInitializationStats{};
  initialized_state_ = NominalState{};
}

void ImuInitializer::accumulate(const sensor::ImuData & sample)
{
  // Welford's centered moments avoid cancellation in E[x^2] - E[x]^2,
  // which can report motion even for identical tilted accelerometer samples.
  ++sample_count_;
  const double count = static_cast<double>(sample_count_);
  const Eigen::Vector3d gyro_delta = sample.gyro - stats_.mean_gyro;
  const Eigen::Vector3d accel_delta = sample.accel - stats_.mean_accel;
  stats_.mean_gyro += gyro_delta / count;
  stats_.mean_accel += accel_delta / count;
  gyro_m2_ += gyro_delta.cwiseProduct(sample.gyro - stats_.mean_gyro);
  accel_m2_ += accel_delta.cwiseProduct(sample.accel - stats_.mean_accel);
  sum_accel_direction_ += sample.accel.normalized();
  updateStatistics();
}

void ImuInitializer::updateStatistics()
{
  stats_.sample_count = sample_count_;
  stats_.first_timestamp = first_timestamp_;
  stats_.last_timestamp = last_timestamp_;
  stats_.duration_sec = has_timestamp_ ? last_timestamp_ - first_timestamp_ : 0.0;
  if (sample_count_ == 0U) {
    stats_.mean_gyro.setZero();
    stats_.mean_accel.setZero();
    stats_.gyro_stddev.setZero();
    stats_.accel_stddev.setZero();
    return;
  }
  const double count = static_cast<double>(sample_count_);
  const Eigen::Vector3d gyro_variance =
    (gyro_m2_ / count).cwiseMax(0.0);
  const Eigen::Vector3d accel_variance =
    (accel_m2_ / count).cwiseMax(0.0);
  stats_.gyro_stddev = gyro_variance.cwiseSqrt();
  stats_.accel_stddev = accel_variance.cwiseSqrt();
}

bool ImuInitializer::stationary() const
{
  return stats_.gyro_std_norm() <= options_.gyro_std_threshold &&
         stats_.accel_std_norm() <= options_.accel_std_threshold &&
         stats_.mean_gyro_norm() <= options_.gyro_mean_threshold &&
         (options_.accel_norm_threshold == 0.0 ||
         std::abs(stats_.mean_accel_norm() - options_.gravity_magnitude) <=
         options_.accel_norm_threshold);
}

}  // namespace itera_lio::estimator
