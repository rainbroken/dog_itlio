// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "itera_lio/loop/loop_closure_manager.hpp"
#include "itera_lio/loop/loop_pair_archive.hpp"
#include <filesystem>
#include <iostream>

#include <cmath>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <unordered_set>
#include <unordered_map>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "itera_lio/loop/submap_builder.hpp"
#include "itera_lio/common/worker_task_metrics.hpp"

namespace itera_lio::loop
{

namespace
{

struct VoxelKey
{
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto mix = [](const int value) {
        return std::hash<std::uint32_t>{}(static_cast<std::uint32_t>(value));
      };
    return mix(key.x) ^ (mix(key.y) << 1U) ^ (mix(key.z) << 2U);
  }
};

void sampleKeyframeCloud(KeyframeCloud & cloud, const LoopClosureOptions & options)
{
  if (cloud.points_I.size() <= options.max_keyframe_points) {
    return;
  }
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
  occupied.reserve(options.max_keyframe_points);
  std::vector<common::MapPoint> sampled;
  sampled.reserve(options.max_keyframe_points);
  const std::size_t count = cloud.points_I.size();
  const auto try_add = [&](const std::size_t source_index) {
      const auto & point = cloud.points_I[source_index];
      if (!point.position.allFinite() || !std::isfinite(point.intensity)) {
        return;
      }
      const VoxelKey key{
        static_cast<int>(std::floor(point.position.x() / options.keyframe_voxel_size)),
        static_cast<int>(std::floor(point.position.y() / options.keyframe_voxel_size)),
        static_cast<int>(std::floor(point.position.z() / options.keyframe_voxel_size))};
      if (occupied.emplace(key).second) {
        sampled.push_back(point);
      }
    };
  // The proportional pass avoids bias toward the start of an unordered scan.
  for (std::size_t index = 0U;
    index < options.max_keyframe_points && sampled.size() < options.max_keyframe_points; ++index)
  {
    try_add(index * count / options.max_keyframe_points);
  }
  // Voxel collisions can leave capacity. Scan every source point once so a
  // sparse region omitted by the proportional pass still has a chance to enter.
  for (std::size_t index = 0U;
    index < count && sampled.size() < options.max_keyframe_points; ++index)
  {
    try_add(index);
  }
  cloud.points_I = std::move(sampled);
}

}  // namespace

LoopClosureManager::LoopClosureManager(const LoopClosureOptions & options)
: options_(options),
  store_(options.max_resident_keyframe_clouds, options.archive_directory),
  detector_(options.detector), verifier_(options.verifier)
{
  if (!std::isfinite(options_.keyframe_voxel_size) || options_.keyframe_voxel_size <= 0.0F ||
    options_.max_keyframe_points == 0U ||
    !std::isfinite(options_.min_attempt_interval_sec) || options_.min_attempt_interval_sec < 0.0 ||
    options_.confirmation_count == 0U ||
    !std::isfinite(options_.confirmation_translation_tolerance) ||
    options_.confirmation_translation_tolerance <= 0.0 ||
    !std::isfinite(options_.confirmation_rotation_tolerance_rad) ||
    options_.confirmation_rotation_tolerance_rad <= 0.0 ||
    options_.strong_evidence.min_inliers == 0U ||
    !std::isfinite(options_.strong_evidence.max_descriptor_distance) ||
    options_.strong_evidence.max_descriptor_distance <= 0.0 ||
    !std::isfinite(options_.strong_evidence.max_fitness) ||
    options_.strong_evidence.max_fitness <= 0.0 ||
    !std::isfinite(options_.strong_evidence.min_reciprocal_inlier_ratio) ||
    options_.strong_evidence.min_reciprocal_inlier_ratio <= 0.0 ||
    options_.strong_evidence.min_reciprocal_inlier_ratio > 1.0 ||
    !std::isfinite(options_.strong_evidence.max_geometry_condition_number) ||
    options_.strong_evidence.max_geometry_condition_number <= 1.0 ||
    !std::isfinite(options_.strong_evidence.max_gravity_residual_rad) ||
    options_.strong_evidence.max_gravity_residual_rad <= 0.0 ||
    !std::isfinite(options_.strong_evidence.max_vertical_separation) ||
    options_.strong_evidence.max_vertical_separation <= 0.0 ||
    options_.worker_queue_capacity == 0U || !std::isfinite(options_.task_timeout_ms) ||
    options_.task_timeout_ms <= 0.0)
  {
    throw std::invalid_argument("invalid loop keyframe cloud options");
  }
  if (options_.enabled) {
    worker_ = std::thread(&LoopClosureManager::workerLoop, this);
  }
}

LoopClosureManager::~LoopClosureManager()
{
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    stopping_ = true;
    worker_stats_.cancelled += pending_tasks_.size();
    pending_tasks_.clear();
    ++cancellation_generation_;
  }
  worker_condition_.notify_all();
  if (worker_.joinable()) {worker_.join();}
}

void LoopClosureManager::submit(VerificationTask task)
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  if (stopping_) {return;}
  if (pending_tasks_.size() >= options_.worker_queue_capacity) {
    pending_tasks_.pop_front();
    ++worker_stats_.dropped;
    ++worker_stats_.cancelled;
  }
  task.cancellation_generation = cancellation_generation_;
  pending_tasks_.push_back(std::move(task));
  ++worker_stats_.submitted;
  worker_stats_.queue_depth = pending_tasks_.size();
  worker_stats_.max_queue_depth = std::max(
    worker_stats_.max_queue_depth, worker_stats_.queue_depth);
  worker_condition_.notify_one();
}

void LoopClosureManager::workerLoop()
{
  while (true) {
    VerificationTask task;
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
      std::lock_guard<std::mutex> lock(worker_mutex_);
      ++worker_stats_.timed_out;
      continue;
    }
    if (!options_.diagnostic_pair_directory.empty() &&
      diagnostic_pair_count_ < options_.diagnostic_pair_limit) {
      const auto directory=std::filesystem::path(options_.diagnostic_pair_directory) /
        (std::to_string(diagnostic_pair_count_++)+"_"+std::to_string(task.candidate.current_id)+
        "_"+std::to_string(task.candidate.matched_id));
      try {
        saveLoopPair(directory,{options_.verifier,task.candidate,task.current,task.matched,task.submap});
      } catch (const std::exception & error) {
        std::cerr << "Loop-pair archive failed: " << error.what() << std::endl;
      }
    }
    auto result = verifier_.verify(task.candidate, task.current, task.matched, task.submap);
    const auto finish = std::chrono::steady_clock::now();
    const double duration_ms = std::chrono::duration<double, std::milli>(finish - start).count();
    common::recordWorkerTask("loop", task.candidate.current_id, task.candidate.matched_id,
      duration_ms, result.isValid(), finish >= task.deadline, 0.0, 0.0, 0, 0);
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_stats_.last_duration_ms = duration_ms;
    worker_stats_.max_duration_ms = std::max(worker_stats_.max_duration_ms, duration_ms);
    if (task.cancellation_generation != cancellation_generation_ || stopping_) {
      ++worker_stats_.cancelled;
      continue;
    }
    if (finish >= task.deadline) {
      ++worker_stats_.timed_out;
      continue;
    }
    completed_results_.push_back(std::move(result));
    ++worker_stats_.completed;
  }
}

std::vector<LoopVerificationResult> LoopClosureManager::pollCompleted()
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  std::vector<LoopVerificationResult> results;
  results.reserve(completed_results_.size());
  while (!completed_results_.empty()) {
    results.push_back(std::move(completed_results_.front()));
    completed_results_.pop_front();
  }
  return results;
}

LoopWorkerStats LoopClosureManager::workerStats() const
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  LoopWorkerStats stats = worker_stats_;
  stats.queue_depth = pending_tasks_.size();
  return stats;
}

void LoopClosureManager::cancelPending()
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  worker_stats_.cancelled += pending_tasks_.size();
  pending_tasks_.clear();
  worker_stats_.queue_depth = 0U;
  ++cancellation_generation_;
}

LoopClosureUpdate LoopClosureManager::addKeyframe(KeyframeCloud cloud)
{
  LoopClosureUpdate update;
  if (!options_.enabled) {return update;}
  sampleKeyframeCloud(cloud, options_);
  cloud.descriptor = detector_.describe(cloud);
  cloud.ring_key = detector_.ringKey(cloud.descriptor);
  if (!store_.add(std::move(cloud))) {return update;}
  update.keyframe_stored = true;
  if (store_.size() < 2U) {return update;}
  const auto & current = store_.clouds().back();
  bool nearby_revisit = false;
  for (const auto & past : store_.clouds()) {
    if (current.keyframe.id <= past.keyframe.id ||
      current.keyframe.id - past.keyframe.id < options_.detector.min_keyframe_separation ||
      current.keyframe.timestamp - past.keyframe.timestamp <
      options_.detector.min_time_separation_sec)
    {continue;}
    const auto delta = current.keyframe.pose_WI.p_WI - past.keyframe.pose_WI.p_WI;
    if (delta.head<2>().norm() <= 3.0 && std::abs(delta.z()) <=
      options_.detector.max_odometry_vertical_separation)
    {
      nearby_revisit = true;
      break;
    }
  }
  if (last_attempt_timestamp_ >= 0.0 &&
    current.keyframe.timestamp - last_attempt_timestamp_ <
    (nearby_revisit ? 0.5 : options_.min_attempt_interval_sec))
  {return update;}
  update.candidates = detector_.detect(current, store_.clouds());
  if (!update.candidates.empty()) {last_attempt_timestamp_ = current.keyframe.timestamp;}
  // No verification can consume these clouds. Keep the attempt timestamp
  // semantics above; do not copy/reload/transform a current submap unnecessarily.
  if (update.candidates.empty()) {return update;}
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double, std::milli>(options_.task_timeout_ms));
  KeyframeCloud verification_current = current;
  verification_current.points_I = buildCurrentLoopSubmap(store_, current.keyframe.id, 5U);
  for (const auto & candidate : update.candidates) {
    KeyframeCloud matched;
    if (store_.get(candidate.matched_id, matched)) {
      auto submap = buildLoopSubmap(
        store_, matched.keyframe.id, current.keyframe.id, options_.submap_half_window,
        options_.detector.min_keyframe_separation);
      submit(
        VerificationTask{
          candidate, verification_current, std::move(matched), std::move(submap), deadline, 0U});
    }
  }
  return update;
}

std::vector<LoopVerificationResult> LoopClosureManager::reverifyOptimized(
  const std::vector<backend::Keyframe> & optimized_keyframes) const
{
  std::vector<LoopVerificationResult> results;
  if (!options_.enabled || optimized_keyframes.size() != store_.size() ||
    optimized_keyframes.size() < options_.detector.min_keyframe_separation + 2U)
  {return results;}

  KeyframeCloudStore optimized_store;
  for (std::size_t id = 0U; id < optimized_keyframes.size(); ++id) {
    KeyframeCloud cloud;
    if (!store_.get(id, cloud) || !optimized_keyframes[id].isValid() ||
      optimized_keyframes[id].id != id ||
      optimized_keyframes[id].timestamp != cloud.keyframe.timestamp)
    {return {};}
    cloud.keyframe = optimized_keyframes[id];
    if (!optimized_store.add(std::move(cloud))) {return {};}
  }

  LoopVerifierOptions local_options = options_.verifier;
  local_options.local_refinement = true;
  local_options.source_voxel_size = 0.12;
  local_options.target_voxel_size = 0.12;
  local_options.gicp_voxel_size = 0.12;
  local_options.max_source_points = 6000U;
  local_options.max_target_points = 20000U;
  local_options.max_translation_jump = 0.60;
  local_options.max_rotation_rad = 0.08;
  local_options.max_fitness = 0.01;
  local_options.inlier_distance = 0.25;
  local_options.min_geometry_eigenvalue_ratio = 0.001;
  local_options.max_correspondence_distance = std::min(
    local_options.max_correspondence_distance, 0.45);
  local_options.min_inlier_ratio = std::max(local_options.min_inlier_ratio, 0.35);
  local_options.min_reciprocal_inlier_ratio = std::max(
    local_options.min_reciprocal_inlier_ratio, 0.60);
  local_options.require_planar_normal_agreement = true;
  const LoopVerifier local_verifier(local_options);
  constexpr std::size_t kMaxVerifications = 300U;
  const auto & clouds = optimized_store.clouds();
  // Revisit means overlap of OBSERVED geometry, not proximity of scanners.
  // A long corridor may first be seen from a distant room/doorway.
  std::vector<std::unordered_set<VoxelKey, VoxelKeyHash>> support(clouds.size());
  std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash> index;
  for (std::size_t id = 0; id < clouds.size(); ++id) {
    for (const auto & point : clouds[id].points_I) {
      if (point.position.squaredNorm() > 3600.0F) {continue;}
      const auto & pose = clouds[id].keyframe.pose_WI;
      const Eigen::Vector3d p = pose.R_WI * point.position.cast<double>() + pose.p_WI;
      support[id].insert({static_cast<int>(std::floor(p.x())),
          static_cast<int>(std::floor(p.y())), static_cast<int>(std::floor(p.z()))});
    }
    for (const auto & voxel : support[id]) {index[voxel].push_back(id);}
  }
  // Cover the whole revisit, including its endpoint. Previously a global
  // budget and a four-edge commit limit concentrated work near the endpoint.
  for (std::size_t current_id = clouds.size(); current_id-- >
    options_.detector.min_keyframe_separation && results.size() < kMaxVerifications;)
  {
    const auto & current = clouds[current_id];
    std::vector<LoopCandidate> candidates;
    std::vector<std::size_t> overlaps(clouds.size(), 0U);
    for (const auto & voxel : support[current_id]) {
      for (const auto id : index.at(voxel)) {++overlaps[id];}
    }
    for (std::size_t matched_id = 0U;
      matched_id + options_.detector.min_keyframe_separation <= current_id; ++matched_id)
    {
      const auto & matched = clouds[matched_id];
      if (current.keyframe.timestamp - matched.keyframe.timestamp <
        options_.detector.min_time_separation_sec) {continue;}
      const auto delta = current.keyframe.pose_WI.p_WI - matched.keyframe.pose_WI.p_WI;
      if (overlaps[matched_id] < 15U || std::abs(delta.z()) >
        options_.detector.max_odometry_vertical_separation) {continue;}
      auto candidate = detector_.compare(current, matched);
      // Descriptor heights and the sensor's local Z axis change with handheld
      // pitch and occlusion. They are not gates for this initialized LOCAL
      // registration. The full geometric/gravity/correction gates remain.
      if (!std::isfinite(candidate.descriptor_distance)) {candidate.descriptor_distance = 1e6;}
      if (candidate.isValid()) {candidates.push_back(candidate);}
    }
    std::sort(candidates.begin(), candidates.end(),
      [&](const auto & lhs, const auto & rhs) {
        const auto score = [&](const auto & candidate) {
            return static_cast<double>(overlaps[candidate.matched_id]) /
              std::sqrt(static_cast<double>(support[current_id].size()) *
              static_cast<double>(support[candidate.matched_id].size()));
          };
        if (score(lhs) != score(rhs)) {return score(lhs) > score(rhs);}
        return lhs.matched_id < rhs.matched_id;
      });
    if (candidates.size() > 3U) {candidates.resize(3U);}
    if (candidates.empty()) {continue;}
    KeyframeCloud source = current;
    source.points_I = buildCurrentLoopSubmap(optimized_store, current_id, 3U);
    for (const auto & candidate : candidates) {
      auto submap = buildLoopSubmap(
        optimized_store, candidate.matched_id, current_id,
        3U, options_.detector.min_keyframe_separation);
      // Restrict the historical map to the predicted source footprint BEFORE
      // registration. The old reciprocal gate compared a corridor view with
      // an entire historical room, rejecting genuine partial-view revisits.
      pcl::PointCloud<pcl::PointXYZ>::Ptr footprint(new pcl::PointCloud<pcl::PointXYZ>);
      const auto & past_pose = clouds[candidate.matched_id].keyframe.pose_WI;
      for (const auto & point : source.points_I) {
        const Eigen::Vector3d p = past_pose.R_WI.transpose() *
          (current.keyframe.pose_WI.R_WI * point.position.cast<double>() +
          current.keyframe.pose_WI.p_WI - past_pose.p_WI);
        footprint->push_back(pcl::PointXYZ(p.x(), p.y(), p.z()));
      }
      pcl::KdTreeFLANN<pcl::PointXYZ> footprint_tree;
      footprint_tree.setInputCloud(footprint);
      std::vector<int> neighbor(1);
      std::vector<float> distance(1);
      submap.erase(std::remove_if(submap.begin(), submap.end(), [&](const auto & point) {
          return footprint_tree.nearestKSearch(pcl::PointXYZ(point.position.x(),
            point.position.y(), point.position.z()), 1, neighbor, distance) == 0 || distance[0] > 1.0F;
        }), submap.end());
      results.push_back(local_verifier.verify(
        candidate, source, clouds[candidate.matched_id], submap));
      if (results.size() >= kMaxVerifications) {break;}
    }
  }
  return results;
}

}  // namespace itera_lio::loop
