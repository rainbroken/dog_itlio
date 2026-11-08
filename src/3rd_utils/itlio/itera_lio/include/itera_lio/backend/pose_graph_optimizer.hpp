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

#ifndef ITERA_LIO__BACKEND__POSE_GRAPH_OPTIMIZER_HPP_
#define ITERA_LIO__BACKEND__POSE_GRAPH_OPTIMIZER_HPP_

#include <Eigen/Core>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

#include <cstdint>
#include <mutex>
#include <vector>

#include "itera_lio/backend/pose_graph.hpp"

namespace itera_lio::backend
{

struct PoseGraphOptimizerOptions
{
  int max_iterations{10};
  double convergence_threshold{1e-5};
  double damping{1e-6};
  double huber_delta{3.0};
  // Zero keeps full-batch optimization. A positive value bounds the number
  // of recent variable nodes assembled by BackendManager.
  std::size_t active_window_size{500U};
};

struct PoseGraphConstraintLinearization
{
  Eigen::Matrix<double, 6, 1> residual{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 6> from_jacobian{Eigen::Matrix<double, 6, 6>::Zero()};
  Eigen::Matrix<double, 6, 6> to_jacobian{Eigen::Matrix<double, 6, 6>::Zero()};
};

PoseGraphConstraintLinearization linearizeConstraint(
  const PoseGraphConstraint & constraint, const common::Pose & pose_WI_from,
  const common::Pose & pose_WI_to);

struct PoseGraphOptimizationReport
{
  bool optimized{false};
  bool converged{false};
  int iterations{0};
  double initial_cost{0.0};
  double final_cost{0.0};
  std::size_t active_node_count{0U};
  std::size_t symbolic_factorizations{0U};
};

class PoseGraphOptimizer
{
public:
  explicit PoseGraphOptimizer(const PoseGraphOptimizerOptions & options = {});

  bool optimize(
    std::vector<Keyframe> & nodes,
    const std::vector<PoseGraphConstraint> & constraints,
    PoseGraphOptimizationReport & report) const;

private:
  PoseGraphOptimizerOptions options_;
  mutable std::mutex solver_mutex_;
  mutable Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver_;
  mutable std::uint64_t cached_pattern_signature_{0U};
  mutable bool has_cached_pattern_{false};
};

}  // namespace itera_lio::backend

#endif  // ITERA_LIO__BACKEND__POSE_GRAPH_OPTIMIZER_HPP_
