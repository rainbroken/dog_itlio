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

#ifndef ITERA_LIO__ESTIMATOR__IMU_PROPAGATOR_HPP_
#define ITERA_LIO__ESTIMATOR__IMU_PROPAGATOR_HPP_

#include <Eigen/Core>

#include <cstddef>

#include "itera_lio/estimator/state.hpp"
#include "itera_lio/sensor/imu_data.hpp"

namespace itera_lio::estimator
{

struct ImuNoiseParameters
{
  double gyro_noise_density{0.01};
  double accel_noise_density{0.1};
  double gyro_bias_noise_density{0.001};
  double accel_bias_noise_density{0.01};
};

class ImuPropagator
{
public:
  explicit ImuPropagator(const ImuNoiseParameters & noise = {});

  bool propagate(
    const sensor::ImuData & previous,
    const sensor::ImuData & current,
    NominalState & state,
    StateCovariance & covariance) const;

private:
  ImuNoiseParameters noise_;
};

}  // namespace itera_lio::estimator

#endif  // ITERA_LIO__ESTIMATOR__IMU_PROPAGATOR_HPP_
