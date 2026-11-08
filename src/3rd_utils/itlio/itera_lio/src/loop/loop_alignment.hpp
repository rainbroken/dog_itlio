// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__LOOP__LOOP_ALIGNMENT_HPP_
#define ITERA_LIO__LOOP__LOOP_ALIGNMENT_HPP_

#include "itera_lio/loop/loop_verifier.hpp"
#include "itera_lio/loop/manual_gicp.hpp"
#include "itera_lio/loop/exact_kdtree.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <cmath>
#include <memory>
#include <vector>

#ifndef ITERA_LIO_EXACT_LOOP_SCORING
#define ITERA_LIO_EXACT_LOOP_SCORING 0
#endif

// Package-private hypothesis generation, preprocessing and consistent scoring.
namespace itera_lio::loop::verification
{
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

// Preserve PCL KNN for the final normal/geometry gates. 1NN scores use the
// identical float L2 metric, a hard distance bound and deterministic tie break.
class VerificationTree {
public:
  void setInputCloud(const Cloud::Ptr& cloud,bool normals=true){
    cloud_=cloud;native_ready_=false;legacy_ready_=false;
#if !ITERA_LIO_EXACT_LOOP_SCORING
    legacy_.setInputCloud(cloud);legacy_ready_=true;
#endif
    (void)normals;
  }
  // Called serially before the parallel scoring regions. The point cloud is
  // immutable for this verify call; all subsequent queries are read-only.
  void prepareScores()const{
#if ITERA_LIO_EXACT_LOOP_SCORING
    if(native_ready_)return;
    std::vector<detail::ExactKdTree::Point> points;points.reserve(cloud_->size());
    for(const auto&p:*cloud_)points.push_back({p.x,p.y,p.z});
    native_.build(std::move(points));native_ready_=true;
#endif
  }
  int nearestKSearch(const pcl::PointXYZ& p,int k,std::vector<int>& ids,std::vector<float>& distances)const{
#if ITERA_LIO_EXACT_LOOP_SCORING
    if(k==1){prepareScores();float d=INFINITY;const int id=native_.nearest({p.x,p.y,p.z},INFINITY,&d);
      ids.resize(1);distances.resize(1);ids[0]=id;distances[0]=d;return id>=0?1:0;}
#endif
    if(!legacy_ready_){legacy_.setInputCloud(cloud_);legacy_ready_=true;}
    return legacy_.nearestKSearch(p,k,ids,distances);
  }
  bool nearestWithin(const pcl::PointXYZ& p,double bound,float& distance)const{
#if ITERA_LIO_EXACT_LOOP_SCORING
    return native_.nearest({p.x,p.y,p.z},bound,&distance)>=0;
#else
    std::vector<int> ids(1);std::vector<float> distances(1);
    const bool found=legacy_.nearestKSearch(p,1,ids,distances)>0 && distances[0]<=bound;
    if(found)distance=distances[0];return found;
#endif
  }
private:
  Cloud::ConstPtr cloud_;
  mutable pcl::KdTreeFLANN<pcl::PointXYZ> legacy_;
  mutable detail::ExactKdTree native_;
  mutable bool native_ready_{false};
  mutable bool legacy_ready_{false};
};

struct GicpHypothesis
{
  const char * initialization{"none"};
  Eigen::Matrix4f transform{Eigen::Matrix4f::Identity()};
  Cloud aligned;
  std::size_t inliers{0U};
  double mean_squared_error{INFINITY};
  bool converged{false};
  bool correction_consistent{false};
};

struct GicpClouds
{
  std::shared_ptr<const ManualGicpCloud> manual_source;
  ManualGicpCloud manual_target;
};

struct CorrectionMetrics
{
  common::Pose global_correction;
  double translation{0.0};
  double rotation{0.0};
  double gravity_residual{0.0};
  double vertical_separation{0.0};
};

Cloud::Ptr transformCloud(const Cloud::Ptr & cloud, const common::Pose & transform);
GicpClouds prepareGicp(
  const Cloud::Ptr & source, const Cloud::Ptr & target, const LoopVerifierOptions & options);
GicpHypothesis alignHypothesis(
  const Cloud::Ptr & source, const GicpClouds & clouds, const common::Pose & initial,
  const char * initialization, const LoopVerifierOptions & options,
  const VerificationTree & target_tree);
GicpHypothesis alignGravityHypothesis(
  const Cloud::Ptr & source, const GicpClouds & clouds, const common::Pose & initial_measured,
  const char * initialization, const KeyframeCloud & current, const KeyframeCloud & matched,
  const LoopVerifierOptions & options, const VerificationTree & target_tree);
CorrectionMetrics correctionMetrics(
  const common::Pose & measured, const KeyframeCloud & current, const KeyframeCloud & matched);
void evaluateCorrectionConsistency(
  GicpHypothesis & hypothesis, const KeyframeCloud & current, const KeyframeCloud & matched,
  const LoopVerifierOptions & options);
bool preferHypothesis(
  const GicpHypothesis & candidate, const GicpHypothesis & selected,
  std::size_t source_size, double min_inlier_ratio);
}  // namespace itera_lio::loop::verification

#endif  // ITERA_LIO__LOOP__LOOP_ALIGNMENT_HPP_
