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

#include "itera_lio/backend/backend_manager.hpp"
#include "itera_lio/common/worker_task_metrics.hpp"
#include <limits>

#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>

namespace itera_lio::backend
{
BackendManager::BackendManager(const BackendOptions & options)
: options_(options), keyframe_manager_(options.keyframes), optimizer_(options.optimizer)
{
  if (!std::isfinite(options_.odometry_rotation_stddev) ||
    options_.odometry_rotation_stddev <= 0.0 ||
    !std::isfinite(options_.odometry_translation_stddev) ||
    options_.odometry_translation_stddev <= 0.0 ||
    options_.worker_queue_capacity == 0U || !std::isfinite(options_.task_timeout_ms) ||
    options_.task_timeout_ms <= 0.0)
  {
    throw std::invalid_argument("invalid backend odometry uncertainty");
  }
  if (options_.enabled && options_.asynchronous_optimization) {
    worker_ = std::thread(&BackendManager::workerLoop, this);
  }
}

BackendManager::~BackendManager()
{
  cancelPending();
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    stopping_ = true;
  }
  worker_condition_.notify_all();
  if (worker_.joinable()) {worker_.join();}
}

BackendUpdate BackendManager::makeUpdateUnlocked() const
{
  BackendUpdate update;
  update.keyframe_count = graph_.nodes().size();
  update.loop_constraint_count = graph_.loopConstraintCount();
  update.odometry_constraint_count = graph_.constraints().size() - update.loop_constraint_count;
  return update;
}

void BackendManager::observeOdometryQuality(
  const common::Pose & pose, const PoseInformationMatrix & information)
{
  if (!options_.enabled || !pose.isValid()) {return;}
  PoseInformationMatrix scale = PoseInformationMatrix::Identity();
  scale.diagonal().head<3>().setConstant(options_.odometry_rotation_stddev);
  scale.diagonal().tail<3>().setConstant(options_.odometry_translation_stddev);
  PoseInformationMatrix body_to_frontend = PoseInformationMatrix::Identity();
  body_to_frontend.bottomRightCorner<3, 3>() = pose.R_WI;
  PoseInformationMatrix normalized = scale * body_to_frontend.transpose() *
    (0.5 * (information + information.transpose())) * body_to_frontend * scale;
  PoseInformationMatrix penalty = 100.0 * PoseInformationMatrix::Identity();
  if (normalized.allFinite()) {
    Eigen::SelfAdjointEigenSolver<PoseInformationMatrix> solver(normalized);
    if (solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() >= -1e-8) {
      Eigen::Matrix<double, 6, 1> variances;
      for (int i = 0; i < 6; ++i) {
        const double strength = std::max(0.0, solver.eigenvalues()[i]);
        // Bounded conservative attenuation: no direction becomes stronger
        // than the configured odometry baseline, or weaker than 1/100 of it.
        variances[i] = 1.0 / std::clamp(strength / (1.0 + strength), 0.01, 1.0);
      }
      penalty = solver.eigenvectors() * variances.asDiagonal() * solver.eigenvectors().transpose();
    }
  }
  PoseInformationMatrix body_to_world = PoseInformationMatrix::Zero();
  body_to_world.topLeftCorner<3, 3>() = pose.R_WI;
  body_to_world.bottomRightCorner<3, 3>() = pose.R_WI;
  const PoseInformationMatrix covariance = body_to_world * scale * penalty * scale *
    body_to_world.transpose();
  std::lock_guard<std::mutex> lock(state_mutex_);
  segment_quality_covariance_ += covariance;
  ++segment_quality_samples_;
}

PoseInformationMatrix BackendManager::odometryInformation(const common::Pose & current_pose) const
{
  PoseInformationMatrix covariance = PoseInformationMatrix::Zero();
  covariance.diagonal().head<3>().setConstant(
    options_.odometry_rotation_stddev * options_.odometry_rotation_stddev);
  covariance.diagonal().tail<3>().setConstant(
    options_.odometry_translation_stddev * options_.odometry_translation_stddev);
  if (segment_quality_samples_ > 0U) {
    PoseInformationMatrix world_to_body = PoseInformationMatrix::Zero();
    world_to_body.topLeftCorner<3, 3>() = current_pose.R_WI.transpose();
    world_to_body.bottomRightCorner<3, 3>() = current_pose.R_WI.transpose();
    // Edge residual at the nominal measurement uses the current IMU axes.
    covariance = world_to_body * (segment_quality_covariance_ /
      static_cast<double>(segment_quality_samples_)) * world_to_body.transpose();
  }
  const PoseInformationMatrix information = covariance.ldlt().solve(PoseInformationMatrix::Identity());
  return 0.5 * (information + information.transpose());
}

BackendUpdate BackendManager::addOdometryKeyframe(
  const double timestamp,
  const common::Pose & pose_WI)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  BackendUpdate update = makeUpdateUnlocked();
  if (!options_.enabled) {
    return update;
  }
  const std::optional<Keyframe> keyframe = keyframe_manager_.addIfNeeded(timestamp, pose_WI);
  if (!keyframe) {
    return update;
  }
  const std::optional<Keyframe> previous = graph_.nodes().empty() ?
    std::nullopt : std::optional<Keyframe>{graph_.nodes().back()};
  if (!graph_.addNode(*keyframe)) {
    return update;
  }
  if (previous) {
    const PoseGraphConstraint constraint{
      previous->id, keyframe->id, relativePose(previous->pose_WI, keyframe->pose_WI),
      odometryInformation(keyframe->pose_WI), ConstraintType::kOdometry};
    if (!graph_.addConstraint(constraint)) {
      throw std::logic_error("failed to add a valid odometry constraint");
    }
  }
  Keyframe optimized = *keyframe;
  if (graph_.hasLoopConstraint() && previous && !optimized_keyframes_.empty()) {
    optimized.pose_WI = composePoses(
      optimized_keyframes_.back().pose_WI,
      relativePose(previous->pose_WI, keyframe->pose_WI));
  }
  optimized_keyframes_.push_back(optimized);
  segment_quality_covariance_.setZero();
  segment_quality_samples_ = 0U;
  update = makeUpdateUnlocked();
  update.keyframe_added = true;
  update.keyframe = keyframe;
  return update;
}


BackendManager::EdgeKey BackendManager::edgeKey(
  const common::KeyframeId from_id, const common::KeyframeId to_id) const
{
  return from_id < to_id ? EdgeKey{from_id, to_id} : EdgeKey{to_id, from_id};
}

std::optional<BackendUpdate> BackendManager::addLoopConstraint(
  const common::KeyframeId from_id, const common::KeyframeId to_id,
  const common::Pose & T_Ia_Ib,
  const PoseInformationMatrix & information)
{
  const PoseGraphConstraint constraint{
    from_id, to_id, T_Ia_Ib, information, ConstraintType::kLoop};
  return processLoopConstraint(constraint, std::nullopt, false);
}

std::optional<BackendUpdate> BackendManager::processLoopConstraint(
  const PoseGraphConstraint & constraint,
  const std::optional<std::chrono::steady_clock::time_point> deadline,
  const bool edge_reserved,
  const std::optional<std::uint64_t> cancellation_generation)
{
  std::lock_guard<std::mutex> optimization_lock(optimization_mutex_);
  const EdgeKey key = edgeKey(constraint.from_id, constraint.to_id);
  std::vector<Keyframe> candidate_poses;
  std::vector<PoseGraphConstraint> candidate_constraints;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!options_.enabled || !constraint.isValid() ||
      constraint.from_id >= graph_.nodes().size() ||
      constraint.to_id >= graph_.nodes().size())
    {
      if (edge_reserved) {loop_edges_.erase(key);}
      return std::nullopt;
    }
    if (!edge_reserved && !loop_edges_.insert(key).second) {
      return std::nullopt;
    }
    candidate_poses = optimized_keyframes_;
    candidate_constraints = graph_.constraints();
  }
  candidate_constraints.push_back(constraint);
  PoseGraphOptimizationReport report;
  std::vector<std::size_t> local_to_global;
  if (options_.optimizer.active_window_size > 0U &&
    candidate_poses.size() > options_.optimizer.active_window_size + 1U)
  {
    const std::size_t active_start =
      candidate_poses.size() - options_.optimizer.active_window_size;
    const std::size_t older_loop_endpoint = std::min(
      static_cast<std::size_t>(constraint.from_id),
      static_cast<std::size_t>(constraint.to_id));
    // The local graph must remain a contiguous odometry chain. Selecting an
    // old loop endpoint plus only the newest window drops the boundary edge
    // and lets the whole active window move rigidly at near-zero cost.
    const std::size_t optimization_start = std::min(
      older_loop_endpoint, active_start - 1U);
    local_to_global.reserve(candidate_poses.size() - optimization_start);
    for (std::size_t id = optimization_start; id < candidate_poses.size(); ++id) {
      local_to_global.push_back(id);
    }
    std::unordered_map<std::size_t, std::size_t> global_to_local;
    global_to_local.reserve(local_to_global.size());
    std::vector<Keyframe> local_poses;
    local_poses.reserve(local_to_global.size());
    for (std::size_t local = 0U; local < local_to_global.size(); ++local) {
      global_to_local.emplace(local_to_global[local], local);
      Keyframe keyframe = candidate_poses[local_to_global[local]];
      keyframe.id = local;
      local_poses.push_back(std::move(keyframe));
    }
    std::vector<PoseGraphConstraint> local_constraints;
    for (const auto & source : candidate_constraints) {
      const auto from = global_to_local.find(source.from_id);
      const auto to = global_to_local.find(source.to_id);
      if (from == global_to_local.end() || to == global_to_local.end()) {continue;}
      auto local_constraint = source;
      local_constraint.from_id = from->second;
      local_constraint.to_id = to->second;
      local_constraints.push_back(std::move(local_constraint));
    }
    if (!optimizer_.optimize(
        local_poses, local_constraints, report))
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      loop_edges_.erase(key);
      return std::nullopt;
    }
    for (std::size_t local = 0U; local < local_poses.size(); ++local) {
      local_poses[local].id = local_to_global[local];
      candidate_poses[local_to_global[local]] = std::move(local_poses[local]);
    }
  } else if (!optimizer_.optimize(
      candidate_poses, candidate_constraints, report))
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    loop_edges_.erase(key);
    return std::nullopt;
  }
  if (
    !std::isfinite(report.final_cost) || report.final_cost > report.initial_cost + 1e-6)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    loop_edges_.erase(key);
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  if ((deadline && std::chrono::steady_clock::now() >= *deadline) ||
    (cancellation_generation &&
    cancellation_generation_.load() != *cancellation_generation))
  {
    loop_edges_.erase(key);
    return std::nullopt;
  }
  if (!graph_.addConstraint(constraint)) {
    loop_edges_.erase(key);
    return std::nullopt;
  }
  const std::size_t optimized_count = candidate_poses.size();
  for (std::size_t index = 0U; index < optimized_count; ++index) {
    optimized_keyframes_[index] = std::move(candidate_poses[index]);
  }
  for (std::size_t index = optimized_count; index < optimized_keyframes_.size(); ++index) {
    optimized_keyframes_[index].pose_WI = composePoses(
      optimized_keyframes_[index - 1U].pose_WI,
      relativePose(graph_.nodes()[index - 1U].pose_WI, graph_.nodes()[index].pose_WI));
  }
  BackendUpdate update = makeUpdateUnlocked();
  update.loop_constraint_added = true;
  update.optimization = report;
  return update;
}


bool BackendManager::enqueueLoopConstraint(
  const common::KeyframeId from_id, const common::KeyframeId to_id,
  const common::Pose & T_Ia_Ib, const PoseInformationMatrix & information)
{
  if (!options_.asynchronous_optimization) {
    return addLoopConstraint(from_id, to_id, T_Ia_Ib, information).has_value();
  }
  const PoseGraphConstraint constraint{
    from_id, to_id, T_Ia_Ib, information, ConstraintType::kLoop};
  const EdgeKey key = edgeKey(from_id, to_id);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!options_.enabled || !constraint.isValid() || from_id >= graph_.nodes().size() ||
      to_id >= graph_.nodes().size() || !loop_edges_.insert(key).second)
    {return false;}
  }

  std::optional<PoseGraphConstraint> dropped;
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (stopping_) {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      loop_edges_.erase(key);
      return false;
    }
    if (pending_tasks_.size() >= options_.worker_queue_capacity) {
      dropped = pending_tasks_.front().constraint;
      pending_tasks_.pop_front();
      ++worker_stats_.dropped;
      ++worker_stats_.cancelled;
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double, std::milli>(options_.task_timeout_ms));
    pending_tasks_.push_back(
      OptimizationTask{
        constraint, deadline, cancellation_generation_.load()});
    ++worker_stats_.submitted;
    worker_stats_.queue_depth = pending_tasks_.size();
    worker_stats_.max_queue_depth = std::max(
      worker_stats_.max_queue_depth, worker_stats_.queue_depth);
  }
  if (dropped) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    loop_edges_.erase(edgeKey(dropped->from_id, dropped->to_id));
  }
  worker_condition_.notify_one();
  return true;
}

void BackendManager::workerLoop()
{
  while (true) {
    OptimizationTask task;
    {
      std::unique_lock<std::mutex> lock(worker_mutex_);
      worker_condition_.wait(lock, [this]() {return stopping_ || !pending_tasks_.empty();});
      if (stopping_ && pending_tasks_.empty()) {return;}
      task = std::move(pending_tasks_.front());
      pending_tasks_.pop_front();
      worker_stats_.queue_depth = pending_tasks_.size();
    }
    const auto start = std::chrono::steady_clock::now();
    if (start >= task.deadline) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        loop_edges_.erase(edgeKey(task.constraint.from_id, task.constraint.to_id));
      }
      std::lock_guard<std::mutex> lock(worker_mutex_);
      ++worker_stats_.timed_out;
      continue;
    }
    auto update = processLoopConstraint(
      task.constraint, task.deadline, true, task.cancellation_generation);
    const auto finish = std::chrono::steady_clock::now();
    const double duration_ms = std::chrono::duration<double, std::milli>(finish - start).count();
    const double missing = std::numeric_limits<double>::quiet_NaN();
    common::recordWorkerTask("backend", task.constraint.from_id, task.constraint.to_id,
      duration_ms, update.has_value(), finish >= task.deadline,
      update ? update->optimization.initial_cost : missing,
      update ? update->optimization.final_cost : missing,
      update ? update->optimization.iterations : 0,
      update ? update->optimization.active_node_count : 0);
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_stats_.last_duration_ms = duration_ms;
    worker_stats_.max_duration_ms = std::max(worker_stats_.max_duration_ms, duration_ms);
    if (task.cancellation_generation != cancellation_generation_.load() || stopping_) {
      ++worker_stats_.cancelled;
    } else if (finish >= task.deadline) {
      ++worker_stats_.timed_out;
    } else {
      ++worker_stats_.completed;
      if (update) {completed_updates_.push_back(std::move(*update));}
    }
  }
}

std::vector<BackendUpdate> BackendManager::pollCompleted()
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  std::vector<BackendUpdate> updates;
  updates.reserve(completed_updates_.size());
  while (!completed_updates_.empty()) {
    updates.push_back(std::move(completed_updates_.front()));
    completed_updates_.pop_front();
  }
  return updates;
}

BackendWorkerStats BackendManager::workerStats() const
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  BackendWorkerStats stats = worker_stats_;
  stats.queue_depth = pending_tasks_.size();
  return stats;
}

void BackendManager::cancelPending()
{
  std::vector<EdgeKey> cancelled;
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    cancelled.reserve(pending_tasks_.size());
    for (const auto & task : pending_tasks_) {
      cancelled.push_back(edgeKey(task.constraint.from_id, task.constraint.to_id));
    }
    worker_stats_.cancelled += pending_tasks_.size();
    pending_tasks_.clear();
    worker_stats_.queue_depth = 0U;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  cancellation_generation_.fetch_add(1U);
  for (const auto & key : cancelled) {
    loop_edges_.erase(key);
  }
}

PoseGraph BackendManager::graph() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return graph_;
}

std::vector<Keyframe> BackendManager::optimizedKeyframes() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return optimized_keyframes_;
}

common::Pose BackendManager::mapToOdometry() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (optimized_keyframes_.empty()) {return {};}
  return composePoses(optimized_keyframes_.back().pose_WI,
    inversePose(graph_.nodes().back().pose_WI));
}



std::pair<std::vector<Keyframe>, std::vector<Keyframe>> BackendManager::previewKeyframes() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return {graph_.nodes(), optimized_keyframes_};
}

}  // namespace itera_lio::backend
