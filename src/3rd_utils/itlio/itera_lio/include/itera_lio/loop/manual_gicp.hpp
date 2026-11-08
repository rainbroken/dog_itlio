// Copyright 2026 rainbroken
// SPDX-License-Identifier: Apache-2.0
#ifndef ITERA_LIO__LOOP__MANUAL_GICP_HPP_
#define ITERA_LIO__LOOP__MANUAL_GICP_HPP_

#include <Eigen/Core>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>
#include "itera_lio/common/geometry.hpp"

namespace itera_lio::loop {
// Own GICP with optional exact correspondence/covariance KD-tree. PCL supplies voxel sampling.
struct ManualGicpOptions {
  double max_correspondence_distance{1.5};
  double cauchy_scale{1.0};
  double rotation_epsilon{0.0008726646259971648};
  double translation_epsilon{0.0005};
  int max_iterations{40};
  int num_threads{2};
  bool xy_yaw_only{false};
};
struct ManualGicpCloud {
  struct Index;
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Matrix3d> covariances;
  std::shared_ptr<const Index> index;
};
struct ManualGicpResult {
  common::Pose T_target_source;
  bool converged{false};
  int iterations{0};
  std::size_t correspondences{0};
  double cost{0.0};
  std::string reason{"not_started"};
};
ManualGicpCloud prepareManualGicp(const std::vector<Eigen::Vector3d> & points,
  double voxel_size, int covariance_neighbors, int num_threads);
// Right SE(3) increment; exposed for independent differential/coordinate tests.
common::Pose manualGicpRetract(const common::Pose & pose,
  const Eigen::Matrix<double, 6, 1> & delta);
Eigen::Matrix<double, 3, 6> manualGicpResidualJacobian(
  const common::Pose & pose, const Eigen::Vector3d & source_point);
ManualGicpResult alignManualGicp(const ManualGicpCloud & target,
  const ManualGicpCloud & source, const common::Pose & initial,
  const ManualGicpOptions & options = {});
}  // namespace itera_lio::loop
#endif
