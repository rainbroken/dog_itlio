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

#ifndef ITERA_LIO__FRONTEND__PLANE_FITTER_HPP_
#define ITERA_LIO__FRONTEND__PLANE_FITTER_HPP_

#include <Eigen/Core>

#include <vector>

namespace itera_lio::frontend
{

struct PlaneFitterOptions
{
  double max_neighbor_distance{3.0};
  double max_point_to_plane_distance{0.1};
  double max_eigenvalue_ratio{0.1};
  double min_normal_norm{0.9};
  bool enable_quality_checks{true};
  double min_tangent_ratio{0.02};
  double max_normal_ratio{0.1};
};

struct Plane
{
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
  Eigen::Vector3d eigenvalues{Eigen::Vector3d::Zero()};
  double max_distance{0.0};
};

bool fitPlane(
  const std::vector<Eigen::Vector3d> & points,
  const PlaneFitterOptions & options,
  Plane & plane);

}  // namespace itera_lio::frontend

#endif  // ITERA_LIO__FRONTEND__PLANE_FITTER_HPP_
