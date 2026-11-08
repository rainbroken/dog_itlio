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
#include "itera_lio/loop/overlap_audit.hpp"
#include <Eigen/Eigenvalues>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>
#include <vector>
namespace itera_lio::loop
{
namespace
{
double p90(std::vector<double> values)
{
  if (values.empty()) {return 0.0;}
  std::sort(values.begin(), values.end());
  return values[static_cast<std::size_t>(std::ceil(.9 * values.size())) - 1U];
}
}  // namespace
bool auditLoopOverlap(
  const std::vector<common::MapPoint> & held_out,
  const std::vector<common::MapPoint> & target, const common::Pose & measured,
  const Eigen::Vector3d & up_in_target, const LoopVerifierOptions & options,
  LoopVerificationResult & result)
{
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;
  Cloud::Ptr cloud(new Cloud);
  for (const auto & p : target) {
    if (p.position.allFinite()) {
      cloud->push_back({p.position.x(), p.position.y(), p.position.z()});
    }
  }
  result.audit_points = held_out.size();
  const auto fail = [&](const char * reason) {result.rejection_reason = reason; return false;};
  if (cloud->size() < static_cast<std::size_t>(options.normal_neighbors) || held_out.empty()) {
    return fail("audit_insufficient_points");
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> tree; tree.setInputCloud(cloud);
  std::vector<int> ids(options.normal_neighbors);
  std::vector<float> distances(options.normal_neighbors);
  std::vector<double> all, walls, horizontal;
  std::size_t overlapping = 0U;
  double nearest_squared_sum = 0.0;
  const Eigen::Vector3d up = up_in_target.normalized();
  for (const auto & p : held_out) {
    const Eigen::Vector3d q = measured.R_WI * p.position.cast<double>() + measured.p_WI;
    if (!q.allFinite()) {continue;}
    const pcl::PointXYZ query(q.x(), q.y(), q.z());
    if (tree.nearestKSearch(
        query, options.normal_neighbors, ids,
        distances) != options.normal_neighbors ||
      distances.front() > options.inlier_distance * options.inlier_distance) {continue;}
    ++overlapping;
    nearest_squared_sum += distances.front();
    if (distances.back() >
      options.max_correspondence_distance * options.max_correspondence_distance) {continue;}
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (int id : ids) {
      center += (*cloud)[id].getVector3fMap().cast<double>();
    }
    center /= ids.size();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int id : ids) {
      const Eigen::Vector3d d = (*cloud)[id].getVector3fMap().cast<double>() - center;
      covariance.noalias() += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success || solver.eigenvalues()[1] <= 1e-12 ||
      solver.eigenvalues()[0] > options.max_normal_eigenvalue_ratio * solver.eigenvalues()[1])
    {
      continue;
    }
    const Eigen::Vector3d normal = solver.eigenvectors().col(0);
    const double error = std::abs(normal.dot(q - center));
    all.push_back(error);
    const double vertical = std::abs(normal.dot(up));
    if (vertical < .3) {walls.push_back(error);}
    if (vertical > .85) {horizontal.push_back(error);}
  }
  result.audit_inliers = overlapping;
  result.audit_overlap = static_cast<double>(overlapping) / held_out.size();
  result.audit_nearest_rmse = overlapping ? std::sqrt(nearest_squared_sum / overlapping) : 0.0;
  result.audit_plane_points = all.size();
  result.audit_plane_p90 = p90(all);
  result.audit_wall_points = walls.size(); result.audit_wall_p90 = p90(walls);
  result.audit_horizontal_points = horizontal.size(); result.audit_horizontal_p90 = p90(horizontal);
  const std::size_t minimum = std::max<std::size_t>(10U, options.min_inliers / 5U);
  if (overlapping < minimum || result.audit_overlap < options.min_inlier_ratio) {
    return fail("audit_insufficient_overlap");
  }
  // Reuse the existing metric tolerance; do not introduce a floor/terrain preset.
  // P90 detects a residual tail that can be hidden by an average fitting score.
  const double limit = std::min(options.inlier_distance, std::sqrt(options.max_fitness));
  // Curved or sparse geometry need not contain walls or floors. If local
  // planes are unsupported, validate held-out Euclidean error explicitly.
  if ((all.size() < minimum && result.audit_nearest_rmse > std::sqrt(options.max_fitness)) ||
    (all.size() >= minimum && result.audit_plane_p90 > limit) ||
    (walls.size() >= minimum && result.audit_wall_p90 > limit) ||
    (horizontal.size() >= minimum && result.audit_horizontal_p90 > limit))
  {
    return fail("audit_plane_residual_too_high");
  }
  return true;
}
}  // namespace itera_lio::loop
