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

#ifndef ITERA_LIO__COMMON__OBSERVATION_TYPES_HPP_
#define ITERA_LIO__COMMON__OBSERVATION_TYPES_HPP_

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace itera_lio::common
{

// Frontend-to-estimator point-to-plane observation contract.
struct PlaneObservation
{
  Eigen::Vector3d point_W{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal_W{Eigen::Vector3d::Zero()};
  Eigen::Vector3d centroid_W{Eigen::Vector3d::Zero()};
  double residual{0.0};
  Eigen::Vector3d point_I{Eigen::Vector3d::Zero()};
  float intensity{0.0F};
  bool has_point_I{false};
  // Relative information: uncertain plane normals reduce measurement precision.
  double plane_weight{1.0};
};

using ObservationBatch = std::vector<PlaneObservation>;

struct RegistrationReport
{
  std::size_t input_points{0U};
  std::size_t effective_points{0U};
  std::size_t rejected_points{0U};
  std::size_t candidate_attempts{0U};
  std::size_t knn_queries{0U};
  std::size_t reused_planes{0U};
  std::size_t accepted_candidates{0U};
  std::size_t total_candidate_attempts{0U};
  std::size_t total_accepted_candidates{0U};
  double mean_abs_residual{0.0};
  double knn_time_ms{0.0};
  double plane_fit_time_ms{0.0};
  double selection_time_ms{0.0};
  ObservationBatch insertion_candidates;
  ObservationBatch observations;
};

// Observation set and weighted pose information actually used by the
// iterated LiDAR update. Diagnostics consume this report instead of
// rebuilding an independent information matrix from raw matches.
struct ObservationUpdateReport
{
  std::vector<PlaneObservation> accepted_observations;
  std::vector<double> robust_weights;
  Eigen::Matrix<double, 6, 6> pose_information{
    Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 1> pose_information_eigenvalues{
    Eigen::Matrix<double, 6, 1>::Zero()};
  double pose_information_condition_number{0.0};
  // Dimensionless information after whitening with the propagated pose
  // covariance. This makes rotation and translation observability comparable.
  Eigen::Matrix<double, 6, 6> whitened_pose_information{
    Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 1> whitened_eigenvalues{
    Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 6> whitened_eigenvectors{
    Eigen::Matrix<double, 6, 6>::Identity()};
  Eigen::Matrix<double, 6, 1> whitened_dof_scores{
    Eigen::Matrix<double, 6, 1>::Zero()};
  double whitened_condition_number{0.0};
  std::size_t mahalanobis_rejected{0U};
};

}  // namespace itera_lio::common

#endif  // ITERA_LIO__COMMON__OBSERVATION_TYPES_HPP_
