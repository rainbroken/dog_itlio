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

#ifndef ITERA_LIO__ESTIMATOR__IMU_INITIALIZER_HPP_
#define ITERA_LIO__ESTIMATOR__IMU_INITIALIZER_HPP_

#include <Eigen/Core>

#include <cstddef>

#include "itera_lio/estimator/state.hpp"
#include "itera_lio/sensor/imu_data.hpp"

namespace itera_lio::estimator
{

struct ImuInitializerOptions
{
  double duration_sec{2.0};
  double gyro_std_threshold{0.1};
  double accel_std_threshold{0.3};
  double gyro_mean_threshold{0.15};
  // Zero disables the check against the configured gravity magnitude.
  double accel_norm_threshold{0.3};
  double gravity_magnitude{9.81};
  // Make the static state satisfy a_W = 0 under the configured gravity.
  // When false, b_a starts at zero and g_W is inferred from the static mean.
  bool initialize_accel_bias_from_static_mean{true};
  std::size_t min_samples{20U};
};

struct ImuInitializationStats
{
  std::size_t sample_count{0U};
  double first_timestamp{0.0};
  double last_timestamp{0.0};
  double duration_sec{0.0};
  Eigen::Vector3d mean_gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d mean_accel{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_stddev{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_stddev{Eigen::Vector3d::Zero()};

  double gyro_std_norm() const {return gyro_stddev.norm();}
  double accel_std_norm() const {return accel_stddev.norm();}
  double mean_gyro_norm() const {return mean_gyro.norm();}
  double mean_accel_norm() const {return mean_accel.norm();}
};

enum class ImuInitializationStatus
{
  kCollecting,
  kInitialized,
  kRejectedInvalidSample,
  kRejectedTimestampRegression,
};

class ImuInitializer
{
public:
  explicit ImuInitializer(const ImuInitializerOptions & options = {});

  ImuInitializationStatus addSample(const sensor::ImuData & sample);
  void reset();

  bool isInitialized() const {return initialized_;}
  const ImuInitializationStats & stats() const {return stats_;}
  const NominalState & initializedState() const {return initialized_state_;}

private:
  void updateStatistics();
  bool stationary() const;
  bool initializeFromWindow();

  ImuInitializerOptions options_;
  bool initialized_{false};
  double first_timestamp_{0.0};
  double last_timestamp_{0.0};
  double window_end_timestamp_{0.0};
  bool has_timestamp_{false};
  Eigen::Vector3d sum_gyro_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d sum_accel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d sum_accel_direction_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d sum_sq_gyro_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d sum_sq_accel_{Eigen::Vector3d::Zero()};
  std::size_t sample_count_{0U};
  ImuInitializationStats stats_;
  NominalState initialized_state_;
};

}  // namespace itera_lio::estimator

#endif  // ITERA_LIO__ESTIMATOR__IMU_INITIALIZER_HPP_
