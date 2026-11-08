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

#ifndef ITERA_LIO__COMMON__SO3_HPP_
#define ITERA_LIO__COMMON__SO3_HPP_

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace itera_lio::common
{

inline Eigen::Matrix3d Hat(const Eigen::Vector3d & vector)
{
  Eigen::Matrix3d result;
  result << 0.0, -vector.z(), vector.y(),
    vector.z(), 0.0, -vector.x(),
    -vector.y(), vector.x(), 0.0;
  return result;
}

inline Eigen::Matrix3d ExpSO3(const Eigen::Vector3d & rotation_vector)
{
  const double theta = rotation_vector.norm();
  const Eigen::Matrix3d omega_hat = Hat(rotation_vector);
  const Eigen::Matrix3d omega_hat_sq = omega_hat * omega_hat;
  double a;
  double b;
  if (theta < 1e-8) {
    const double theta_sq = theta * theta;
    a = 1.0 - theta_sq / 6.0 + theta_sq * theta_sq / 120.0;
    b = 0.5 - theta_sq / 24.0 + theta_sq * theta_sq / 720.0;
  } else {
    a = std::sin(theta) / theta;
    b = (1.0 - std::cos(theta)) / (theta * theta);
  }
  return Eigen::Matrix3d::Identity() + a * omega_hat + b * omega_hat_sq;
}

inline Eigen::Vector3d LogSO3(const Eigen::Matrix3d & rotation)
{
  const Eigen::Matrix3d projected =
    0.5 * (rotation - rotation.transpose());
  const double sine = 0.5 * std::sqrt(2.0) * projected.norm();
  const double cosine = std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double theta = std::atan2(sine, cosine);
  if (theta < 1e-8) {
    return Eigen::Vector3d(projected(2, 1), projected(0, 2), projected(1, 0));
  }
  if (std::abs(M_PI - theta) < 1e-6) {
    Eigen::Vector3d axis;
    const Eigen::Vector3d diagonal(
      std::sqrt(std::max(0.0, (rotation(0, 0) + 1.0) * 0.5)),
      std::sqrt(std::max(0.0, (rotation(1, 1) + 1.0) * 0.5)),
      std::sqrt(std::max(0.0, (rotation(2, 2) + 1.0) * 0.5)));
    axis = diagonal;
    if (rotation(0, 1) < 0.0) {axis.y() = -axis.y();}
    if (rotation(0, 2) < 0.0) {axis.z() = -axis.z();}
    if (axis.norm() < 1e-8) {
      axis = Eigen::Vector3d(1.0, 0.0, 0.0);
    } else {
      axis.normalize();
    }
    return theta * axis;
  }
  return theta / std::sin(theta) *
         Eigen::Vector3d(projected(2, 1), projected(0, 2), projected(1, 0));
}

inline Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d & rotation_vector)
{
  const double theta = rotation_vector.norm();
  const Eigen::Matrix3d omega_hat = Hat(rotation_vector);
  const Eigen::Matrix3d omega_hat_sq = omega_hat * omega_hat;
  if (theta < 1e-8) {
    return Eigen::Matrix3d::Identity() - 0.5 * omega_hat + omega_hat_sq / 6.0;
  }
  return Eigen::Matrix3d::Identity() -
         (1.0 - std::cos(theta)) / (theta * theta) * omega_hat +
         (theta - std::sin(theta)) / (theta * theta * theta) * omega_hat_sq;
}

inline Eigen::Matrix3d RightJacobianInverseSO3(const Eigen::Vector3d & rotation_vector)
{
  const double theta = rotation_vector.norm();
  const Eigen::Matrix3d omega_hat = Hat(rotation_vector);
  const Eigen::Matrix3d omega_hat_sq = omega_hat * omega_hat;
  if (theta < 1e-8) {
    return Eigen::Matrix3d::Identity() + 0.5 * omega_hat + omega_hat_sq / 12.0;
  }
  const double half_theta = 0.5 * theta;
  const double coefficient =
    1.0 / (theta * theta) - (1.0 + std::cos(theta)) /
    (2.0 * theta * std::sin(theta));
  (void)half_theta;
  return Eigen::Matrix3d::Identity() + 0.5 * omega_hat + coefficient * omega_hat_sq;
}

}  // namespace itera_lio::common

#endif  // ITERA_LIO__COMMON__SO3_HPP_
