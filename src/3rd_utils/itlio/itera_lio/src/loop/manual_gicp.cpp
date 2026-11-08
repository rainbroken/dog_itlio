// Copyright 2026 rainbroken
// SPDX-License-Identifier: Apache-2.0
#include "itera_lio/loop/manual_gicp.hpp"
#include "itera_lio/common/so3.hpp"
#include "itera_lio/loop/gicp_profile.hpp"
#include "itera_lio/loop/exact_kdtree.hpp"
#ifndef ITERA_MANUAL_NATIVE_SEARCH
#define ITERA_MANUAL_NATIVE_SEARCH 0
#endif
#ifndef ITERA_MANUAL_PARALLEL_SEARCH
#define ITERA_MANUAL_PARALLEL_SEARCH 0
#endif
#include <Eigen/Cholesky>
#include <Eigen/LU>
#ifndef ITERA_MANUAL_DIRECT_EIGEN
#define ITERA_MANUAL_DIRECT_EIGEN 0
#endif
#ifndef ITERA_MANUAL_COMPACT_NORMALS
#define ITERA_MANUAL_COMPACT_NORMALS 0
#endif
#include <cstdio>
#include <cstdlib>
#ifndef ITERA_MANUAL_WARM_SEARCH
#define ITERA_MANUAL_WARM_SEARCH 0
#endif
#ifndef ITERA_MANUAL_DIRECT_INVERSE
#define ITERA_MANUAL_DIRECT_INVERSE 0
#endif
#ifndef ITERA_MANUAL_NATIVE_KNN
#define ITERA_MANUAL_NATIVE_KNN 0
#endif
#include <Eigen/Eigenvalues>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace itera_lio::loop {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
struct ManualGicpCloud::Index {
  Cloud::Ptr cloud{new Cloud};
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
#if ITERA_MANUAL_NATIVE_SEARCH
  detail::ExactKdTree native_tree;
#endif
};
namespace {
struct Match {
  Eigen::Vector3d source;
  Eigen::Vector3d target;
  Eigen::Matrix3d precision;
};
struct System {
  Matrix6 H{Matrix6::Zero()};
  Vector6 g{Vector6::Zero()};
  double cost{0.0};
};
struct MatchWorkspace {
  std::vector<Match> slots, matches;
  std::vector<unsigned char> valid;
};
double robustCost(double e, double scale) {return scale * std::log1p(e / scale);}
const std::vector<Match>& correspondences(const ManualGicpCloud & target,
  const ManualGicpCloud & source, const common::Pose & pose, double max_distance, int num_threads,
  std::vector<int>& hints, MatchWorkspace& workspace)
{
  b005::Timer phase("correspondence");
  b005::count("nn_queries",source.points.size());
  auto& matches = workspace.matches;
  matches.clear();
  matches.reserve(source.points.size());
  // A hint supplies only a valid upper bound. The exact tree traversal still
  // searches all potentially closer/tied nodes and keeps the original ID tie rule.
  static const bool warm_search = std::getenv("ITERA_B015_DISABLE_WARM_SEARCH") == nullptr;
#if !ITERA_MANUAL_NATIVE_SEARCH && !ITERA_MANUAL_PARALLEL_SEARCH
  (void)num_threads;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  for (std::size_t i=0; i<source.points.size(); ++i) {
    const Eigen::Vector3d x=pose.R_WI * source.points[i] + pose.p_WI;
    pcl::PointXYZ query(static_cast<float>(x.x()),static_cast<float>(x.y()),static_cast<float>(x.z()));
    if (target.index->tree.nearestKSearch(query,1,indices,distances)!=1 ||
      distances[0]>max_distance*max_distance) {continue;}
    const auto j=static_cast<std::size_t>(indices[0]);
    const Eigen::Matrix3d covariance=target.covariances[j] +
      pose.R_WI * source.covariances[i] * pose.R_WI.transpose();
    Eigen::LDLT<Eigen::Matrix3d> factor(covariance);
    if (factor.info()!=Eigen::Success || !factor.isPositive()) {continue;}
    const Eigen::Matrix3d precision=factor.solve(Eigen::Matrix3d::Identity());
    if (precision.allFinite()) {matches.push_back({source.points[i],target.points[j],precision});}
  }
#else
  const std::size_t count=source.points.size();
  auto& slots = workspace.slots;
  auto& valid = workspace.valid;
  slots.resize(count);
  valid.assign(count,0);
  const int threads=ITERA_MANUAL_PARALLEL_SEARCH?num_threads:1;
  #pragma omp parallel num_threads(threads) if(threads>1)
  {
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    #pragma omp for schedule(static)
    for(std::ptrdiff_t k=0;k<static_cast<std::ptrdiff_t>(count);++k){
      const auto i=static_cast<std::size_t>(k);
      const Eigen::Vector3d x=pose.R_WI*source.points[i]+pose.p_WI;
      const float qx=static_cast<float>(x.x()),qy=static_cast<float>(x.y()),qz=static_cast<float>(x.z());
      int neighbor=-1;
#if ITERA_MANUAL_NATIVE_SEARCH
      neighbor=target.index->native_tree.nearest({qx,qy,qz},max_distance*max_distance,nullptr,warm_search?hints[i]:-1);
      hints[i]=neighbor;
#else
      pcl::PointXYZ query(qx,qy,qz);
      if(target.index->tree.nearestKSearch(query,1,indices,distances)==1 && distances[0]<=max_distance*max_distance)neighbor=indices[0];
#endif
      if(neighbor<0)continue;
      const auto j=static_cast<std::size_t>(neighbor);
      const Eigen::Matrix3d covariance=target.covariances[j]+pose.R_WI*source.covariances[i]*pose.R_WI.transpose();
#if ITERA_MANUAL_DIRECT_INVERSE
      // Both inputs are SPD regularized plane covariances (eigenvalues >= .001).
      // Keep a fallback for externally constructed clouds / near singular metrics.
      Eigen::Matrix3d precision;
      const double determinant=covariance.determinant();
      if(determinant>1e-10 && covariance(0,0)>0 &&
         covariance(0,0)*covariance(1,1)-covariance(0,1)*covariance(1,0)>0){
        precision=covariance.inverse();
      }else{
        Eigen::LDLT<Eigen::Matrix3d> factor(covariance);
        if(factor.info()!=Eigen::Success||!factor.isPositive())continue;
        precision=factor.solve(Eigen::Matrix3d::Identity());
      }
#else
      Eigen::LDLT<Eigen::Matrix3d> factor(covariance);
      if(factor.info()!=Eigen::Success||!factor.isPositive())continue;
      const Eigen::Matrix3d precision=factor.solve(Eigen::Matrix3d::Identity());
#endif
      if(precision.allFinite()){slots[i]={source.points[i],target.points[j],precision};valid[i]=1;}
    }
  }
  for(std::size_t i=0;i<count;++i)if(valid[i])matches.push_back(slots[i]);
#endif
  return matches;
}
System linearize(const std::vector<Match> & matches, const common::Pose & pose,
  const ManualGicpOptions & options)
{
  b005::Timer phase("linearize");
  b005::count("linearized_matches",matches.size());
  // Per-thread accumulators and ordered reduction avoid concurrent matrix writes.
  std::vector<System> partial(static_cast<std::size_t>(options.num_threads));
  #pragma omp parallel num_threads(options.num_threads)
  {
    int thread=0;
#ifdef _OPENMP
    thread=omp_get_thread_num();
#endif
    auto & local=partial[static_cast<std::size_t>(thread)];
    #pragma omp for schedule(static)
    for (std::ptrdiff_t i=0;i<static_cast<std::ptrdiff_t>(matches.size());++i) {
      const auto & m=matches[static_cast<std::size_t>(i)];
      const Eigen::Vector3d r=pose.R_WI*m.source + pose.p_WI - m.target;
      const double e=std::max(0.0,0.5*r.dot(m.precision*r));
      const double weight=options.cauchy_scale/(options.cauchy_scale+e);
#if ITERA_MANUAL_COMPACT_NORMALS
      const Eigen::Matrix3d M=pose.R_WI.transpose()*m.precision*pose.R_WI;
      const Eigen::Vector3d v=pose.R_WI.transpose()*m.precision*r;
      // C=M*[p]x. Explicit cross products skip the known zeros in [p]x.
      Eigen::Matrix3d C;
      C.col(0)=m.source.z()*M.col(1)-m.source.y()*M.col(2);
      C.col(1)=m.source.x()*M.col(2)-m.source.z()*M.col(0);
      C.col(2)=m.source.y()*M.col(0)-m.source.x()*M.col(1);
      Eigen::Matrix3d Hrr;
      Hrr.row(0)=m.source.z()*C.row(1)-m.source.y()*C.row(2);
      Hrr.row(1)=m.source.x()*C.row(2)-m.source.z()*C.row(0);
      Hrr.row(2)=m.source.y()*C.row(0)-m.source.x()*C.row(1);
      local.H.topLeftCorner<3,3>().noalias()+=weight*Hrr;
      local.H.topRightCorner<3,3>().noalias()-=weight*C.transpose();
      local.H.bottomLeftCorner<3,3>().noalias()-=weight*C;
      local.H.bottomRightCorner<3,3>().noalias()+=weight*M;
      local.g.head<3>().noalias()+=weight*m.source.cross(v);
      local.g.tail<3>().noalias()+=weight*v;
#else
      const auto J=manualGicpResidualJacobian(pose,m.source);
      local.H.noalias()+=weight*J.transpose()*m.precision*J;
      local.g.noalias()+=weight*J.transpose()*m.precision*r;
#endif
      local.cost+=robustCost(e,options.cauchy_scale);
    }
  }
  System total;
  for (const auto & part:partial) {total.H+=part.H;total.g+=part.g;total.cost+=part.cost;}
  return total;
}
double frozenCost(const std::vector<Match> & matches,const common::Pose & pose,double scale)
{
  b005::Timer phase("frozen_cost");
  b005::count("evaluated_matches",matches.size());
  double cost=0.0;
  for (const auto & m:matches) {
    const Eigen::Vector3d r=pose.R_WI*m.source+pose.p_WI-m.target;
    cost+=robustCost(std::max(0.0,0.5*r.dot(m.precision*r)),scale);
  }
  return cost;
}
}  // namespace
ManualGicpCloud prepareManualGicp(const std::vector<Eigen::Vector3d> & points,
  double voxel_size,int covariance_neighbors,int num_threads)
{
  if (!std::isfinite(voxel_size)||voxel_size<=0.0||covariance_neighbors<5||num_threads<1) {
    throw std::invalid_argument("invalid manual GICP preprocessing options");
  }
  b005::Timer phase("prepare_total");
  b005::count("prepare_input_points",points.size());
  b005::Timer voxel("prepare_voxel");
  ManualGicpCloud output;
  auto index=std::make_shared<ManualGicpCloud::Index>();
  Cloud::Ptr input(new Cloud);
  for (const auto & point:points) {
    if (point.allFinite() && point.cwiseAbs().maxCoeff()<std::numeric_limits<float>::max()) {
      input->push_back(pcl::PointXYZ(static_cast<float>(point.x()),static_cast<float>(point.y()),static_cast<float>(point.z())));
    }
  }
  if (input->empty()) {output.index=index;return output;}
  pcl::VoxelGrid<pcl::PointXYZ> sampler;
  sampler.setInputCloud(input);
  const float leaf=static_cast<float>(voxel_size);
  sampler.setLeafSize(leaf,leaf,leaf);
  sampler.filter(*index->cloud);
  if (index->cloud->empty()) {output.index=index;return output;}
  voxel.stop();
  b005::Timer tree("prepare_tree");
#if !ITERA_MANUAL_NATIVE_SEARCH || !ITERA_MANUAL_NATIVE_KNN
  index->tree.setInputCloud(index->cloud);
#endif
#if ITERA_MANUAL_NATIVE_SEARCH
  std::vector<detail::ExactKdTree::Point> native_points;
  native_points.reserve(index->cloud->size());
  for(const auto& point:*index->cloud)native_points.push_back({point.x,point.y,point.z});
  index->native_tree.build(std::move(native_points));
#endif
  tree.stop();
  b005::Timer cov("prepare_covariance");
  b005::count("prepare_output_points",index->cloud->size());
  for (const auto & p:*index->cloud) {output.points.emplace_back(p.x,p.y,p.z);}
  output.covariances.resize(output.points.size(),Eigen::Matrix3d::Identity());
  #pragma omp parallel num_threads(num_threads)
  {
    std::vector<int> ids(static_cast<std::size_t>(covariance_neighbors));
    std::vector<float> distances(static_cast<std::size_t>(covariance_neighbors));
    #pragma omp for schedule(static)
    for (std::ptrdiff_t i=0;i<static_cast<std::ptrdiff_t>(output.points.size());++i) {
#if ITERA_MANUAL_NATIVE_KNN && ITERA_MANUAL_NATIVE_SEARCH
      const auto& p=(*index->cloud)[static_cast<std::size_t>(i)];
      const int n=index->native_tree.nearestK({p.x,p.y,p.z},covariance_neighbors,ids,distances);
#else
      const int n=index->tree.nearestKSearch((*index->cloud)[static_cast<std::size_t>(i)],covariance_neighbors,ids,distances);
#endif
    if (n<5) {continue;}
    Eigen::Vector3d center=Eigen::Vector3d::Zero();
    for (int j=0;j<n;++j) {center+=output.points[static_cast<std::size_t>(ids[j])];}
    center/=n;
    Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();
    for (int j=0;j<n;++j) {
      const Eigen::Vector3d d=output.points[static_cast<std::size_t>(ids[j])]-center;
      covariance.noalias()+=d*d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen;
#if ITERA_MANUAL_DIRECT_EIGEN
    eigen.computeDirect(covariance);
#else
    eigen.compute(covariance);
#endif
    if (eigen.info()!=Eigen::Success || eigen.eigenvalues()[1]<=1e-12) {continue;}
    const Eigen::Vector3d normal=eigen.eigenvectors().col(0);
    // Planar covariance regularization; this is a geometric metric, not sensor covariance.
    output.covariances[static_cast<std::size_t>(i)]=Eigen::Matrix3d::Identity()-0.999*normal*normal.transpose();
  }
  } // parallel covariance region
  output.index=index;
  return output;
}
common::Pose manualGicpRetract(const common::Pose & pose,const Vector6 & delta)
{
  const Eigen::Vector3d omega=delta.head<3>();
  return {pose.R_WI*common::ExpSO3(omega),
    pose.p_WI+pose.R_WI*common::RightJacobianSO3(-omega)*delta.tail<3>()};
}
Eigen::Matrix<double,3,6> manualGicpResidualJacobian(const common::Pose & pose,
  const Eigen::Vector3d & source_point)
{
  Eigen::Matrix<double,3,6> J;
  J.leftCols<3>()=-pose.R_WI*common::Hat(source_point);
  J.rightCols<3>()=pose.R_WI;
  return J;
}
ManualGicpResult alignManualGicp(const ManualGicpCloud & target,
  const ManualGicpCloud & source,const common::Pose & initial,const ManualGicpOptions & options)
{
  if (!std::isfinite(options.max_correspondence_distance)||options.max_correspondence_distance<=0.0||
    !std::isfinite(options.cauchy_scale)||options.cauchy_scale<=0.0||options.max_iterations<1||
    options.num_threads<1||!std::isfinite(options.rotation_epsilon)||options.rotation_epsilon<=0.0||
    !std::isfinite(options.translation_epsilon)||options.translation_epsilon<=0.0) {
    throw std::invalid_argument("invalid manual GICP alignment options");
  }
  ManualGicpResult result;
  result.T_target_source=initial;
  if (!initial.isValid(1e-6)) {result.reason="invalid_initial_pose";return result;}
  if (!target.index||!source.index||target.points.size()<6||source.points.size()<6||
    target.covariances.size()!=target.points.size()||source.covariances.size()!=source.points.size()) {
    result.reason="insufficient_points";return result;
  }
  if (options.xy_yaw_only && (std::abs(initial.p_WI.z())>1e-9 ||
    (initial.R_WI*Eigen::Vector3d::UnitZ()-Eigen::Vector3d::UnitZ()).norm()>1e-9)) {
    result.reason="invalid_planar_initial_pose";return result;
  }
  const std::array<int,6> full{0,1,2,3,4,5};
  const std::array<int,6> planar{2,3,4,0,0,0};
  const auto & active=options.xy_yaw_only?planar:full;
  const int dimension=options.xy_yaw_only?3:6;
  std::vector<int> hints(source.points.size(),-1);
  MatchWorkspace workspace;
  double lambda=1e-4;
  for (int iteration=0;iteration<options.max_iterations;++iteration) {
    b005::count("outer_iterations");
    const auto& matches=correspondences(target,source,result.T_target_source,options.max_correspondence_distance,options.num_threads,hints,workspace);
    result.iterations=iteration+1;
    result.correspondences=matches.size();
    if (matches.size()<6) {result.reason="insufficient_correspondences";return result;}
    const auto system=linearize(matches,result.T_target_source,options);
    result.cost=system.cost;
    if (!system.H.allFinite()||!system.g.allFinite()||!std::isfinite(system.cost)) {
      result.reason="nonfinite_system";return result;
    }
    Eigen::MatrixXd H(dimension,dimension);
    Eigen::VectorXd g(dimension);
    for (int i=0;i<dimension;++i) {
      g[i]=system.g[active[i]];
      for (int j=0;j<dimension;++j) {H(i,j)=system.H(active[i],active[j]);}
    }
    const Eigen::VectorXd diagonal=H.diagonal().cwiseMax(1.0);
    if ((g.array().abs()/diagonal.array().sqrt()).maxCoeff()<1e-10) {
      result.converged=true;result.reason="stationary";return result;
    }
    bool accepted=false;
    for (int trial=0;trial<12;++trial) {
      b005::count("lm_trials");
      b005::Timer solve("lm_solve_retract");
      Eigen::MatrixXd damped=H;
      damped.diagonal().array()+=lambda*diagonal.array();
      Eigen::LDLT<Eigen::MatrixXd> factor(damped);
      if (factor.info()!=Eigen::Success||!factor.isPositive()) {b005::count("lm_rejected");lambda*=10.0;continue;}
      const Eigen::VectorXd local=factor.solve(-g);
      if (!local.allFinite()) {b005::count("lm_rejected");lambda*=10.0;continue;}
      Vector6 delta=Vector6::Zero();
      for (int i=0;i<dimension;++i) {delta[active[i]]=local[i];}
      const auto trial_pose=manualGicpRetract(result.T_target_source,delta);
      solve.stop();
      const double cost=frozenCost(matches,trial_pose,options.cauchy_scale);
      if (!trial_pose.isValid(1e-6)||!std::isfinite(cost)||cost>system.cost) {b005::count("lm_rejected");lambda*=10.0;continue;}
#ifdef ITERA_MANUAL_TRACE_ITERATIONS
      if(const char* file=std::getenv("ITERA_ITERATION_TRACE")){
        FILE* trace=std::fopen(file,"a");if(trace){std::fprintf(trace,"%s,%d,%zu,%.17g,%.17g,%.17g,%.17g\n",b005::prefix.c_str(),iteration,matches.size(),system.cost,cost,delta.head<3>().norm(),delta.tail<3>().norm());std::fclose(trace);}
      }
#endif
      result.T_target_source=trial_pose;
      result.cost=cost;
      accepted=true;
      // Large damping can produce a tiny step without convergence; do not accept that as proof.
      const bool converged=lambda<=1.0 && delta.head<3>().norm()<options.rotation_epsilon &&
        delta.tail<3>().norm()<options.translation_epsilon;
      lambda=std::max(1e-12,lambda*0.25);
      if (converged) {result.converged=true;result.reason="small_accepted_step";return result;}
      break;
    }
    if (!accepted) {b005::count("no_descent");result.reason="no_descent_step";return result;}
  }
  result.reason="iteration_limit";
  return result;
}
}  // namespace itera_lio::loop
