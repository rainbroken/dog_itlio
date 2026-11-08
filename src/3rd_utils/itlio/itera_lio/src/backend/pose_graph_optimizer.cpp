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

#include "itera_lio/backend/pose_graph_optimizer.hpp"

#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <Eigen/Geometry>

#include <array>
#include <type_traits>
#include "itera_lio/backend/sparse_assembly_cache.hpp"
#ifndef ITERA_BACKEND_CACHED_SPARSE
#define ITERA_BACKEND_CACHED_SPARSE 0
#endif
#include "itera_lio/backend/backend_profile.hpp"
#ifndef ITERA_BACKEND_FIXED_BLOCKS
#define ITERA_BACKEND_FIXED_BLOCKS 0
#endif
#include <cmath>
#include <stdexcept>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <limits>

#include "itera_lio/common/so3.hpp"

namespace itera_lio::backend
{

namespace
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

common::Pose applyLeftPerturbation(const common::Pose & pose, const Vector6d & delta)
{
  return common::Pose{
    common::ExpSO3(delta.head<3>()) * pose.R_WI,
    pose.p_WI + delta.tail<3>()};
}

Vector6d constraintResidual(
  const PoseGraphConstraint & constraint, const common::Pose & pose_WI_from,
  const common::Pose & pose_WI_to)
{
  const common::Pose predicted = relativePose(pose_WI_from, pose_WI_to);
  const common::Pose error = composePoses(inversePose(constraint.T_Ia_Ib), predicted);
  Vector6d residual;
  residual.head<3>() = common::LogSO3(error.R_WI);
  residual.tail<3>() = error.p_WI;
  return residual;
}

PoseGraphConstraintLinearization constraintLinearization(
  const PoseGraphConstraint & constraint, const common::Pose & pose_WI_from,
  const common::Pose & pose_WI_to)
{
  const Eigen::Matrix3d rotation_from_transpose = pose_WI_from.R_WI.transpose();
  const Eigen::Matrix3d measurement_rotation_transpose = constraint.T_Ia_Ib.R_WI.transpose();
  const Eigen::Matrix3d rotation_to_error =
    measurement_rotation_transpose * rotation_from_transpose;
  const Eigen::Vector3d position_delta = pose_WI_to.p_WI - pose_WI_from.p_WI;
  const Eigen::Matrix3d error_rotation =
    rotation_to_error * pose_WI_to.R_WI;

  PoseGraphConstraintLinearization result;
  result.residual.head<3>() = common::LogSO3(error_rotation);
  result.residual.tail<3>() = measurement_rotation_transpose *
    (rotation_from_transpose * position_delta - constraint.T_Ia_Ib.p_WI);

  // Rotations use a left perturbation while positions are perturbed additively.
  // J_l(phi)^-1 equals J_r(-phi)^-1 for the SO(3) logarithm derivative.
  const Eigen::Matrix3d log_jacobian =
    common::RightJacobianInverseSO3(-result.residual.head<3>());
  result.from_jacobian.block<3, 3>(0, 0) = -log_jacobian * rotation_to_error;
  result.from_jacobian.block<3, 3>(3, 0) =
    rotation_to_error * common::Hat(position_delta);
  result.from_jacobian.block<3, 3>(3, 3) = -rotation_to_error;
  result.to_jacobian.block<3, 3>(0, 0) = log_jacobian * rotation_to_error;
  result.to_jacobian.block<3, 3>(3, 3) = rotation_to_error;
  return result;
}

Vector6d constraintResidual(
  const PoseGraphConstraint & constraint, const std::vector<Keyframe> & nodes)
{
  return constraintResidual(
    constraint, nodes[constraint.from_id].pose_WI, nodes[constraint.to_id].pose_WI);
}

double robustWeight(
  const Vector6d & residual, const PoseInformationMatrix & information,
  const double huber_delta)
{
  const double norm = std::sqrt(std::max(0.0, residual.dot(information * residual)));
  return norm <= huber_delta || norm == 0.0 ? 1.0 : huber_delta / norm;
}

double huberCost(const double squared, const double huber_delta)
{
  const double norm = std::sqrt(std::max(0.0, squared));
  return norm <= huber_delta ? 0.5 * squared :
         huber_delta * (norm - 0.5 * huber_delta);
}

double totalCost(
  const std::vector<Keyframe> & nodes,
  const std::vector<PoseGraphConstraint> & constraints, const double huber_delta)
{
  double cost = 0.0;
  for (const auto & constraint : constraints) {
    const Vector6d residual = constraintResidual(constraint, nodes);
    const double squared = std::max(0.0, residual.dot(constraint.information * residual));
    cost += huberCost(squared, huber_delta);
  }
  return cost;
}

bool validOptions(const PoseGraphOptimizerOptions & options)
{
  return options.max_iterations > 0 && std::isfinite(options.convergence_threshold) &&
         options.convergence_threshold > 0.0 && std::isfinite(options.damping) &&
         options.damping >= 0.0 && std::isfinite(options.huber_delta) &&
         options.huber_delta > 0.0;
}

}  // namespace

PoseGraphConstraintLinearization linearizeConstraint(
  const PoseGraphConstraint & constraint, const common::Pose & pose_WI_from,
  const common::Pose & pose_WI_to)
{
  return constraintLinearization(constraint, pose_WI_from, pose_WI_to);
}


PoseGraphOptimizer::PoseGraphOptimizer(const PoseGraphOptimizerOptions & options)
: options_(options)
{
  if (!validOptions(options_)) {
    throw std::invalid_argument("invalid pose graph optimizer options");
  }
}

bool PoseGraphOptimizer::optimize(
  std::vector<Keyframe> & nodes,
  const std::vector<PoseGraphConstraint> & constraints,
  PoseGraphOptimizationReport & report) const
{
  std::lock_guard<std::mutex> solver_lock(solver_mutex_);
  b007::Timer whole("total");
  b007::Timer setup("validation_and_setup");
  report = {};
  if (nodes.size() < 2U || constraints.empty()) {
    return false;
  }
  for (std::size_t index = 0U; index < nodes.size(); ++index) {
    if (!nodes[index].isValid() || nodes[index].id != index) {
      return false;
    }
  }
  for (const auto & constraint : constraints) {
    if (!constraint.isValid() || constraint.from_id >= nodes.size() ||
      constraint.to_id >= nodes.size())
    {
      return false;
    }
  }
  constexpr Eigen::Index block_size = 6;
  const Eigen::Index dimension = static_cast<Eigen::Index>(nodes.size() - 1U) * block_size;
  std::uint64_t pattern_signature = 1469598103934665603ULL;
  const auto hash_value = [&](const std::uint64_t value) {
      pattern_signature ^= value;
      pattern_signature *= 1099511628211ULL;
    };
  hash_value(static_cast<std::uint64_t>(dimension));
  hash_value(static_cast<std::uint64_t>(constraints.size()));
  for (const auto & constraint : constraints) {
    hash_value(static_cast<std::uint64_t>(constraint.from_id));
    hash_value(static_cast<std::uint64_t>(constraint.to_id));
  }
  report.active_node_count = nodes.size();
  setup.stop();
  b007::Timer initial("initial_cost");
  report.initial_cost = totalCost(
    nodes, constraints, options_.huber_delta);
  initial.stop();
  report.final_cost = report.initial_cost;
  bool pattern_analyzed = has_cached_pattern_ &&
    cached_pattern_signature_ == pattern_signature;
#if ITERA_BACKEND_CACHED_SPARSE
  Eigen::SparseMatrix<double> hessian(dimension,dimension);
  detail::StreamingSparseAssembly hessian_triplets;
  static const bool streaming_sparse = std::getenv("ITERA_B015_DISABLE_STREAM_SPARSE") == nullptr;
#endif
  for (int iteration = 0; iteration < options_.max_iterations; ++iteration) {
    b007::count("outer_iterations");
    b007::count("pose_constraints",constraints.size());
    b007::Timer assembly("assembly");
#if ITERA_BACKEND_CACHED_SPARSE
    // All graph constraints/options are immutable for this optimize call;
    // only numeric block values change between iterations.
    // SimplicialLDLT is declared with Eigen's default Lower mode. No other
    // consumer reads this Hessian, so its unused upper half need not be stored.
    hessian_triplets.begin(hessian,streaming_sparse,true,true);
#else
    std::vector<Eigen::Triplet<double>> hessian_triplets;
#endif
    hessian_triplets.reserve(
      constraints.size() * 4U * 36U +
      static_cast<std::size_t>(dimension));
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dimension);
#if ITERA_BACKEND_FIXED_BLOCKS
    const auto accumulate_pose_constraints = [&](auto block_tag) {
      constexpr int B=decltype(block_tag)::value;
      using Jacobian=Eigen::Matrix<double,6,B>;
      for(const auto& constraint:constraints){
        const auto linearization=constraintLinearization(constraint,nodes[constraint.from_id].pose_WI,nodes[constraint.to_id].pose_WI);
        const Vector6d& residual=linearization.residual;
        const double weight=robustWeight(residual,constraint.information,options_.huber_delta);
        std::array<std::pair<std::size_t,Jacobian>,2> jacobians;
        std::size_t count=0;
        const auto project=[](const Matrix6d& full){
          Jacobian result;
          result=full;
          return result;
        };
        if(constraint.from_id!=0U)jacobians[count++]={constraint.from_id,project(linearization.from_jacobian)};
        if(constraint.to_id!=0U)jacobians[count++]={constraint.to_id,project(linearization.to_jacobian)};
        for(std::size_t left=0;left<count;++left){
          const Eigen::Index row=static_cast<Eigen::Index>(jacobians[left].first-1U)*B;
          const Eigen::Matrix<double,B,6> weighted_transpose=weight*jacobians[left].second.transpose()*constraint.information;
          gradient.template segment<B>(row)+=weighted_transpose*residual;
          for(std::size_t right=0;right<count;++right){
            const Eigen::Index column=static_cast<Eigen::Index>(jacobians[right].first-1U)*B;
            const Eigen::Matrix<double,B,B> block=weighted_transpose*jacobians[right].second;
            for(int i=0;i<B;++i)for(int j=0;j<B;++j)hessian_triplets.emplace_back(row+i,column+j,block(i,j));
          }
        }
      }
    };
    accumulate_pose_constraints(std::integral_constant<int,6>{});
#else
    for (const auto & constraint : constraints) {
      const auto linearization = constraintLinearization(
        constraint, nodes[constraint.from_id].pose_WI, nodes[constraint.to_id].pose_WI);
      const Vector6d & residual = linearization.residual;
      const double weight = robustWeight(residual, constraint.information, options_.huber_delta);
    std::array<std::pair<std::size_t, Eigen::Matrix<double, 6, Eigen::Dynamic>>, 2> jacobians;
    std::size_t jacobian_count = 0U;
    const auto project = [&](const Matrix6d & full) {
        Eigen::Matrix<double, 6, Eigen::Dynamic> result(6, block_size);
        result = full;
        return result;
      };
    if (constraint.from_id != 0U) {
      jacobians[jacobian_count++] = {constraint.from_id, project(linearization.from_jacobian)};
    }
    if (constraint.to_id != 0U) {
      jacobians[jacobian_count++] = {constraint.to_id, project(linearization.to_jacobian)};
    }
    for (std::size_t left = 0U; left < jacobian_count; ++left) {
      const Eigen::Index row = static_cast<Eigen::Index>(jacobians[left].first - 1U) * block_size;
      const Eigen::Matrix<double, Eigen::Dynamic, 6> weighted_transpose =
        weight * jacobians[left].second.transpose() * constraint.information;
      gradient.segment(row, block_size) += weighted_transpose * residual;
      for (std::size_t right = 0U; right < jacobian_count; ++right) {
        const Eigen::Index column = static_cast<Eigen::Index>(jacobians[right].first - 1U) * block_size;
        const Eigen::MatrixXd block = weighted_transpose * jacobians[right].second;
        for (Eigen::Index block_row = 0; block_row < block_size; ++block_row) {
          for (Eigen::Index block_column = 0; block_column < block_size; ++block_column) {
              hessian_triplets.emplace_back(
                row + block_row, column + block_column, block(block_row, block_column));
            }
          }
        }
      }
    }
#endif
    for (Eigen::Index index = 0; index < dimension; ++index) {
      hessian_triplets.emplace_back(index, index, options_.damping);
    }
    assembly.stop();
    b007::Timer sparse_build("sparse_build");
#if ITERA_BACKEND_CACHED_SPARSE
    if(hessian_triplets.finish())b007::count("sparse_cache_used");
#else
    Eigen::SparseMatrix<double> hessian(dimension, dimension);
    hessian.setFromTriplets(hessian_triplets.begin(), hessian_triplets.end());
    hessian.makeCompressed();
#endif
    sparse_build.stop();
    if (!pattern_analyzed) {
      b007::Timer symbolic("symbolic");
      solver_.analyzePattern(hessian);
      if (solver_.info() != Eigen::Success) {return false;}
      pattern_analyzed = true;
      has_cached_pattern_ = true;
      cached_pattern_signature_ = pattern_signature;
      ++report.symbolic_factorizations;
    }
    b007::Timer factorization("factorization");
    solver_.factorize(hessian);
    factorization.stop();
    if (solver_.info() != Eigen::Success) {
      return false;
    }
    b007::Timer solve("solve");
    const Eigen::VectorXd step = solver_.solve(-gradient);
    solve.stop();
    if (solver_.info() != Eigen::Success || !step.allFinite()) {
      return false;
    }
    b007::Timer backtracking("backtracking");
    const double previous_cost = report.final_cost;
    double step_scale = 1.0;
    double candidate_cost = INFINITY;
    std::vector<Keyframe> candidate_nodes;
    for (int backtrack = 0; backtrack < 12; ++backtrack) {
      b007::count("backtrack_trials");
      candidate_nodes = nodes;
      for (std::size_t node_id = 1U; node_id < candidate_nodes.size(); ++node_id) {
        const Eigen::Index offset = static_cast<Eigen::Index>(node_id - 1U) * block_size;
        const Vector6d delta = step_scale * step.segment<6>(offset);
        candidate_nodes[node_id].pose_WI = applyLeftPerturbation(
          candidate_nodes[node_id].pose_WI, delta);
      }
      candidate_cost = totalCost(
        candidate_nodes, constraints, options_.huber_delta);
      if (std::isfinite(candidate_cost) && candidate_cost <= previous_cost) {
        break;
      }
      step_scale *= 0.5;
    }
    backtracking.stop();
    if (!std::isfinite(candidate_cost) || candidate_cost > previous_cost) {
      // At convergence, summation/rotation roundoff can make every trial
      // marginally more expensive. Keep the last accepted state, never the
      // increasing trial. Require prior progress, a small tested step, and
      // cost agreement at floating-point resolution; genuine failures remain
      // failures. step_scale has already been halved after the last trial.
      const double cost_resolution = 64.0 * std::numeric_limits<double>::epsilon() *
        std::max(1.0, std::abs(previous_cost));
      if (iteration > 0 && previous_cost < report.initial_cost &&
        std::isfinite(candidate_cost) && candidate_cost - previous_cost <= cost_resolution &&
        2.0 * step_scale * step.norm() <= options_.convergence_threshold)
      {
        report.converged = true;
        break;
      }
      if (std::getenv("ITERA_GRAPH_DIAGNOSTIC")) {
        std::cerr << std::setprecision(17) << "GRAPH_BACKTRACK_FAILURE nodes=" << nodes.size()
                  << " iteration=" << iteration << " initial=" << report.initial_cost
                  << " previous=" << previous_cost << " candidate=" << candidate_cost
                  << " step=" << step.norm() << " gradient=" << gradient.norm() << '\n';
      }
      return false;
    }
    nodes = std::move(candidate_nodes);
    report.iterations = iteration + 1;
    report.final_cost = candidate_cost;
    if (step_scale * step.norm() <= options_.convergence_threshold) {
      report.converged = true;
      break;
    }
  }
  report.optimized = true;
  return true;
}

}  // namespace itera_lio::backend
