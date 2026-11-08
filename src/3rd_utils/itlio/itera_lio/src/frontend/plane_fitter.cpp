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

#include "itera_lio/frontend/plane_fitter.hpp"

#include <Eigen/Eigenvalues>

#include <cmath>
#include <stdexcept>

namespace itera_lio::frontend
{

bool fitPlane(
  const std::vector<Eigen::Vector3d> & points,
  const PlaneFitterOptions & options,
  Plane & plane)
{
  plane = Plane{};
  if (points.size() < 3U || !std::isfinite(options.max_neighbor_distance) ||
    options.max_neighbor_distance <= 0.0 ||
    !std::isfinite(options.max_point_to_plane_distance) ||
    options.max_point_to_plane_distance <= 0.0 ||
    !std::isfinite(options.max_eigenvalue_ratio) || options.max_eigenvalue_ratio <= 0.0 ||
    !std::isfinite(options.min_tangent_ratio) || options.min_tangent_ratio < 0.0 ||
    options.min_tangent_ratio > 1.0 ||
    !std::isfinite(options.max_normal_ratio) || options.max_normal_ratio <= 0.0)
  {
    return false;
  }
  for (const auto & point : points) {
    if (!point.allFinite()) {return false;}
    plane.centroid += point;
  }
  plane.centroid /= static_cast<double>(points.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto & point : points) {
    const Eigen::Vector3d difference = point - plane.centroid;
    if (difference.norm() > options.max_neighbor_distance) {return false;}
    covariance += difference * difference.transpose();
  }
  covariance /= static_cast<double>(points.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {return false;}
  plane.eigenvalues = solver.eigenvalues();
  plane.normal = solver.eigenvectors().col(0);
  if (!plane.normal.allFinite() || plane.normal.norm() < options.min_normal_norm ||
    plane.eigenvalues.z() <= 1e-12 ||
    (options.enable_quality_checks &&
    (plane.eigenvalues.y() <= 1e-12 ||
    plane.eigenvalues.y() / plane.eigenvalues.z() < options.min_tangent_ratio ||
    plane.eigenvalues.x() / plane.eigenvalues.y() > options.max_normal_ratio)) ||
    // With ordered eigenvalues and positive tangent variance, the enabled
    // lambda0/lambda1 bound already implies lambda0/lambda2 when at least
    // as strict. Retain the legacy check for other option combinations.
    ((!options.enable_quality_checks ||
    options.max_normal_ratio > options.max_eigenvalue_ratio) &&
    plane.eigenvalues.x() / plane.eigenvalues.z() > options.max_eigenvalue_ratio))
  {
    return false;
  }
  plane.normal.normalize();
  for (const auto & point : points) {
    plane.max_distance =
      std::max(plane.max_distance, std::abs(plane.normal.dot(point - plane.centroid)));
  }
  return plane.max_distance <= options.max_point_to_plane_distance;
}

}  // namespace itera_lio::frontend
