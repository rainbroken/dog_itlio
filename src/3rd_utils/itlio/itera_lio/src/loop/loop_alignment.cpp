// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0 (the "License");

#include "loop_alignment.hpp"
#include "itera_lio/backend/pose_graph.hpp"
#include "itera_lio/common/so3.hpp"
#include "itera_lio/loop/gicp_profile.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#ifndef ITERA_LIO_CACHE_LOOP_SOURCE
#define ITERA_LIO_CACHE_LOOP_SOURCE 0
#endif

namespace itera_lio::loop::verification
{
namespace
{

Eigen::Matrix4f poseMatrix(const common::Pose & pose)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = pose.R_WI.cast<float>();
  transform.block<3, 1>(0, 3) = pose.p_WI.cast<float>();
  return transform;
}

void scoreAlignment(
  GicpHypothesis & hypothesis,
  const VerificationTree & target_tree,
  const LoopVerifierOptions & options)
{
  const double max_squared = options.inlier_distance *
    options.inlier_distance;
  double squared_error = 0.0;
#if ITERA_LIO_EXACT_LOOP_SCORING
  target_tree.prepareScores();
  std::vector<float> squared(hypothesis.aligned.size(),-1.f);
  #pragma omp parallel for num_threads(options.num_threads) schedule(static)
  for(std::ptrdiff_t i=0;i<static_cast<std::ptrdiff_t>(hypothesis.aligned.size());++i){
    float d;if(target_tree.nearestWithin(hypothesis.aligned[static_cast<std::size_t>(i)],max_squared,d))squared[static_cast<std::size_t>(i)]=d;
  }
  // Sum in the original source order, preserving the acceptance arithmetic.
  for(float d:squared)if(d>=0){++hypothesis.inliers;squared_error+=d;}
#else
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  for (const auto & point : hypothesis.aligned) {
    if (target_tree.nearestKSearch(point, 1, indices, distances) > 0 &&
      distances[0] <= max_squared)
    {
      ++hypothesis.inliers;
      squared_error += distances[0];
    }
  }
#endif
  if (hypothesis.inliers > 0U) {
    hypothesis.mean_squared_error = squared_error /
      static_cast<double>(hypothesis.inliers);
  }
}

common::Pose gravityAlignedCorrection(
  const common::Pose & measured,
  const KeyframeCloud & current,
  const KeyframeCloud & matched)
{
  const auto corrected_current_pose = backend::composePoses(
    matched.keyframe.pose_WI, measured);
  const auto correction = backend::composePoses(
    corrected_current_pose, backend::inversePose(current.keyframe.pose_WI));
  const double yaw = std::atan2(correction.R_WI(1, 0), correction.R_WI(0, 0));
  common::Pose aligned;
  aligned.R_WI = Eigen::AngleAxisd(
    yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  aligned.p_WI = Eigen::Vector3d(
    correction.p_WI.x(), correction.p_WI.y(), 0.0);
  return aligned;
}

template<class Prepared,class Factory,class PointCount>
Prepared prepareSource(const std::vector<Eigen::Vector3d>& points,
  const LoopVerifierOptions& options,Factory factory,PointCount point_count)
{
#if ITERA_LIO_CACHE_LOOP_SOURCE
  // Explicit diagnostic ablation; each replay process starts with empty cache.
  static const bool bypass=[](){const char* p=std::getenv("ITERA_LIO_DISABLE_SOURCE_CACHE");return p&&p[0]=='1'&&p[1]=='\0';}();
  if(bypass)return factory();
  struct Entry {
    std::uint64_t hash,age;
    double voxel;
    int neighbors,threads;
    std::vector<Eigen::Vector3d> input;
    Prepared prepared;
  };
  // Thread-owned, immutable prepared entries; no locks, stale shared pointers,
  // or assumptions about keyframe IDs / pose versions. Full input equality is
  // mandatory even on a hash hit. Bound both entry count and stored point count.
  struct Cache {std::vector<Entry> entries;std::uint64_t clock{0};};
  thread_local Cache cache;
  std::uint64_t hash=1469598103934665603ULL;
  for(const auto& p:points)for(int a=0;a<3;++a){std::uint64_t bits;std::memcpy(&bits,&p[a],sizeof(bits));hash=(hash^bits)*1099511628211ULL;}
  const auto equal=[&](const Entry& e){
    if(e.hash!=hash||e.input.size()!=points.size()||e.neighbors!=options.normal_neighbors||e.threads!=options.num_threads||std::memcmp(&e.voxel,&options.gicp_voxel_size,sizeof(double))!=0)return false;
    for(std::size_t i=0;i<points.size();++i)if(std::memcmp(e.input[i].data(),points[i].data(),3*sizeof(double))!=0)return false;
    return true;
  };
  for(auto& e:cache.entries)if(equal(e)){e.age=++cache.clock;b005::count("prepare_source_cache_hits");return e.prepared;}
  b005::count("prepare_source_cache_misses");
  auto prepared=factory();
  constexpr std::size_t point_budget=32768;
  const std::size_t required=points.size()+point_count(prepared);
  if(required>point_budget)return prepared;
  auto stored=[&](){std::size_t n=0;for(const auto& e:cache.entries)n+=e.input.size()+point_count(e.prepared);return n;};
  while(!cache.entries.empty()&&(cache.entries.size()>=8||stored()+required>point_budget)){
    const auto oldest=std::min_element(cache.entries.begin(),cache.entries.end(),[](const Entry& a,const Entry& b){return a.age<b.age;});cache.entries.erase(oldest);b005::count("prepare_source_cache_evictions");
  }
  cache.entries.push_back({hash,++cache.clock,options.gicp_voxel_size,options.normal_neighbors,options.num_threads,points,std::move(prepared)});
  return cache.entries.back().prepared;
#else
  (void)point_count;
  return factory();
#endif
}

}  // namespace

Cloud::Ptr transformCloud(
  const Cloud::Ptr & cloud, const common::Pose & transform)
{
  Cloud::Ptr transformed(new Cloud);
  transformed->reserve(cloud->size());
  for (const auto & point : *cloud) {
    const Eigen::Vector3d position = transform.R_WI *
      Eigen::Vector3d(point.x, point.y, point.z) + transform.p_WI;
    transformed->push_back(pcl::PointXYZ(
      static_cast<float>(position.x()),
      static_cast<float>(position.y()),
      static_cast<float>(position.z())));
  }
  return transformed;
}

GicpClouds prepareGicp(
  const Cloud::Ptr & source, const Cloud::Ptr & target,
  const LoopVerifierOptions & options)
{
  b005::Timer prep("verifier_prepare");
  std::vector<Eigen::Vector3d> source_points, target_points;
  for (const auto & p : *source) {source_points.emplace_back(p.x, p.y, p.z);}
  for (const auto & p : *target) {target_points.emplace_back(p.x, p.y, p.z);}
  GicpClouds clouds;
  clouds.manual_source=prepareSource<std::shared_ptr<const ManualGicpCloud>>(source_points,options,
    [&](){return std::make_shared<const ManualGicpCloud>(prepareManualGicp(source_points,options.gicp_voxel_size,options.normal_neighbors,options.num_threads));},
    [](const std::shared_ptr<const ManualGicpCloud>& c){return c->points.size();});
  clouds.manual_target=prepareManualGicp(target_points, options.gicp_voxel_size,
    options.normal_neighbors, options.num_threads);
  return clouds;
}

GicpHypothesis alignHypothesis(
  const Cloud::Ptr & source, const GicpClouds & clouds,
  const common::Pose & initial, const char * initialization,
  const LoopVerifierOptions & options,
  const VerificationTree & target_tree)
{
  b005::Context context(initialization);
  b005::Timer hypothesis_timer("hypothesis_total");
  GicpHypothesis hypothesis;
  hypothesis.initialization = initialization;
  ManualGicpOptions manual;
  manual.max_correspondence_distance=options.max_correspondence_distance;
  manual.cauchy_scale=options.robust_kernel_scale;
  manual.max_iterations=options.max_iterations;
  manual.num_threads=options.num_threads;
  b005::Timer alignment_timer("align_total");
  const auto result=alignManualGicp(clouds.manual_target,*clouds.manual_source,initial,manual);
  alignment_timer.stop();
  b005::recordPose(initialization,result.T_target_source,result.converged,result.iterations,result.reason);
  b005::count("stop_"+result.reason);
  if (!result.converged) {return hypothesis;}
  hypothesis.converged=true;
  hypothesis.transform=poseMatrix(result.T_target_source);
  hypothesis.aligned=*transformCloud(source,result.T_target_source);
  scoreAlignment(hypothesis,target_tree,options);
  return hypothesis;
}

GicpHypothesis alignGravityHypothesis(
  const Cloud::Ptr & source, const GicpClouds & clouds,
  const common::Pose & initial_measured, const char * initialization,
  const KeyframeCloud & current, const KeyframeCloud & matched,
  const LoopVerifierOptions & options,
  const VerificationTree & target_tree)
{
  b005::Context context(initialization);
  b005::Timer hypothesis_timer("hypothesis_total");
  GicpHypothesis hypothesis;
  hypothesis.initialization = initialization;
  ManualGicpOptions manual;
  manual.max_correspondence_distance=options.max_correspondence_distance;
  manual.cauchy_scale=options.robust_kernel_scale;
  manual.max_iterations=options.max_iterations;
  manual.num_threads=options.num_threads;
  manual.xy_yaw_only=true;
  const auto initial=gravityAlignedCorrection(initial_measured,current,matched);
  b005::Timer alignment_timer("align_total");
  const auto result=alignManualGicp(clouds.manual_target,*clouds.manual_source,initial,manual);
  alignment_timer.stop();
  b005::recordPose(initialization,backend::relativePose(matched.keyframe.pose_WI,backend::composePoses(result.T_target_source,current.keyframe.pose_WI)),result.converged,result.iterations,result.reason);
  b005::count("stop_"+result.reason);
  if (!result.converged) {return hypothesis;}
  const auto measured=backend::relativePose(matched.keyframe.pose_WI,
    backend::composePoses(result.T_target_source,current.keyframe.pose_WI));
  hypothesis.converged=true;
  hypothesis.transform=poseMatrix(measured);
  hypothesis.aligned=*transformCloud(source,measured);
  scoreAlignment(hypothesis,target_tree,options);
  return hypothesis;
}

CorrectionMetrics correctionMetrics(
  const common::Pose & measured,
  const KeyframeCloud & current,
  const KeyframeCloud & matched)
{
  CorrectionMetrics metrics;
  const auto corrected_current_pose = backend::composePoses(
    matched.keyframe.pose_WI, measured);
  metrics.global_correction = backend::composePoses(
    corrected_current_pose, backend::inversePose(current.keyframe.pose_WI));
  metrics.translation =
    (corrected_current_pose.p_WI - current.keyframe.pose_WI.p_WI).norm();
  metrics.rotation = common::LogSO3(metrics.global_correction.R_WI).norm();
  const Eigen::Vector3d corrected_gravity_axis =
    metrics.global_correction.R_WI * Eigen::Vector3d::UnitZ();
  metrics.gravity_residual = std::acos(std::clamp(
    Eigen::Vector3d::UnitZ().dot(corrected_gravity_axis), -1.0, 1.0));
  metrics.vertical_separation = std::abs(Eigen::Vector3d::UnitZ().dot(
    corrected_current_pose.p_WI - matched.keyframe.pose_WI.p_WI));
  return metrics;
}

void evaluateCorrectionConsistency(
  GicpHypothesis & hypothesis,
  const KeyframeCloud & current,
  const KeyframeCloud & matched,
  const LoopVerifierOptions & options)
{
  if (!hypothesis.converged) {return;}
  const auto & matrix = hypothesis.transform;
  Eigen::Quaterniond orientation(matrix.block<3, 3>(0, 0).cast<double>());
  orientation.normalize();
  const common::Pose measured{
    orientation.toRotationMatrix(), matrix.block<3, 1>(0, 3).cast<double>()};
  const auto metrics = correctionMetrics(measured, current, matched);
  hypothesis.correction_consistent =
    metrics.translation <= options.max_translation_jump &&
    metrics.rotation <= options.max_rotation_rad &&
    metrics.gravity_residual <= options.max_gravity_tilt_correction_rad &&
    metrics.vertical_separation <= options.max_vertical_correction;
}

bool preferHypothesis(
  const GicpHypothesis & candidate, const GicpHypothesis & selected,
  std::size_t source_size, double min_inlier_ratio)
{
  if (!candidate.converged) {return false;}
  if (!selected.converged) {return true;}
  const auto has_overlap = [source_size, min_inlier_ratio](const GicpHypothesis & hypothesis) {
      return static_cast<double>(hypothesis.inliers) /
             static_cast<double>(source_size) >= min_inlier_ratio;
    };
  const bool candidate_has_overlap = has_overlap(candidate);
  const bool selected_has_overlap = has_overlap(selected);
  if (candidate_has_overlap != selected_has_overlap) {return candidate_has_overlap;}
  if (candidate.correction_consistent != selected.correction_consistent) {
    return candidate.correction_consistent;
  }

  if (candidate.inliers == 0U || selected.inliers == 0U) {
    return candidate.inliers > selected.inliers;
  }
  return candidate.mean_squared_error < selected.mean_squared_error;
}

}  // namespace itera_lio::loop::verification
