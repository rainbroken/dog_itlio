// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "itera_lio/loop/loop_verifier.hpp"
#include "itera_lio/loop/overlap_audit.hpp"
#include "loop_alignment.hpp"
#include "itera_lio/backend/pose_graph.hpp"
#include "itera_lio/common/so3.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace itera_lio::loop
{
namespace
{
using verification::Cloud;
using verification::VerificationTree;
using verification::GicpHypothesis;
using verification::prepareGicp;
using verification::transformCloud;
using verification::alignHypothesis;
using verification::alignGravityHypothesis;
using verification::evaluateCorrectionConsistency;
using verification::preferHypothesis;
using verification::correctionMetrics;

Cloud::Ptr toPcl(const std::vector<common::MapPoint> & points)
{
  Cloud::Ptr cloud(new Cloud);
  cloud->reserve(points.size());
  for (const auto & point : points) {
    cloud->push_back(pcl::PointXYZ(point.position.x(), point.position.y(), point.position.z()));
  }
  return cloud;
}
}  // namespace

LoopVerifier::LoopVerifier(const LoopVerifierOptions & options)
: options_(options)
{
  if (options_.registration_engine != "manual_gicp") {
    throw std::invalid_argument("loop registration engine must be manual_gicp");
  }
  if (options_.min_points == 0U || options_.min_inliers == 0U ||
    !std::isfinite(options_.max_fitness) || options_.max_fitness <= 0.0 ||
    !std::isfinite(options_.max_translation_jump) || options_.max_translation_jump <= 0.0 ||
    !std::isfinite(options_.max_rotation_rad) || options_.max_rotation_rad <= 0.0 ||
    !std::isfinite(options_.max_correspondence_distance) ||
    options_.max_correspondence_distance <= 0.0 ||
    !std::isfinite(options_.min_inlier_ratio) || options_.min_inlier_ratio <= 0.0 ||
    options_.min_inlier_ratio > 1.0 ||
    !std::isfinite(options_.min_reciprocal_inlier_ratio) ||
    options_.min_reciprocal_inlier_ratio <= 0.0 ||
    options_.min_reciprocal_inlier_ratio > 1.0 ||
    !std::isfinite(options_.min_geometry_eigenvalue_ratio) ||
    options_.min_geometry_eigenvalue_ratio <= 0.0 ||
    options_.min_geometry_eigenvalue_ratio >= 1.0 ||
    !std::isfinite(options_.max_normal_eigenvalue_ratio) ||
    options_.max_normal_eigenvalue_ratio <= 0.0 ||
    options_.max_normal_eigenvalue_ratio >= 1.0 ||
    !std::isfinite(options_.max_gravity_tilt_correction_rad) ||
    options_.max_gravity_tilt_correction_rad <= 0.0 ||
    !std::isfinite(options_.max_vertical_correction) ||
    options_.max_vertical_correction <= 0.0 ||
    !std::isfinite(options_.inlier_distance) || options_.inlier_distance <= 0.0 ||
    !std::isfinite(options_.gicp_voxel_size) || options_.gicp_voxel_size <= 0.0 ||
    !std::isfinite(options_.robust_kernel_scale) || options_.robust_kernel_scale <= 0.0 ||
    options_.num_threads <= 0 ||
    options_.normal_neighbors < 5 || options_.max_iterations <= 0 ||
    options_.max_source_points < options_.min_points ||
    options_.max_source_points < options_.min_inliers ||
    options_.max_target_points < options_.min_points)
  {
    throw std::invalid_argument("invalid loop verifier options");
  }
  if (!std::isfinite(options_.source_voxel_size) || options_.source_voxel_size <= 0.0 ||
    !std::isfinite(options_.target_voxel_size) || options_.target_voxel_size <= 0.0)
  {throw std::invalid_argument("invalid loop sampling resolution");}
}

LoopVerificationResult LoopVerifier::verify(
  const LoopCandidate & candidate, const KeyframeCloud & current,
  const KeyframeCloud & matched,
  const std::vector<common::MapPoint> & submap_points) const
{
  LoopVerificationResult result;
  result.current_id = candidate.current_id;
  result.matched_id = candidate.matched_id;
  result.descriptor_distance = candidate.descriptor_distance;
  result.odometry_distance = candidate.odometry_distance;
  result.odometry_vertical_distance = candidate.odometry_vertical_distance;
  result.gravity_angle_rad = candidate.gravity_angle_rad;
  if (!candidate.isValid() || !current.isValid() || !matched.isValid() ||
    candidate.current_id != current.keyframe.id || candidate.matched_id != matched.keyframe.id)
  {
    result.rejection_reason = "invalid_input";
    return result;
  }
  if (current.points_I.size() < options_.min_points ||
    submap_points.size() < options_.min_points)
  {
    result.rejection_reason = "insufficient_points";
    return result;
  }
  std::vector<common::MapPoint> fitting_points, held_out;
  fitting_points.reserve(current.points_I.size());
  held_out.reserve(current.points_I.size() / 5U);
  // Split BEFORE voxel filtering, normal estimation and all hypotheses.
  // The same held-out points validate every selected initialization.
  for (std::size_t i = 0; i < current.points_I.size(); ++i) {
    (i % 5U == 0U ? held_out : fitting_points).push_back(current.points_I[i]);
  }
  const auto raw_source = toPcl(fitting_points);
  pcl::VoxelGrid<pcl::PointXYZ> source_voxel;
  source_voxel.setInputCloud(raw_source);
  const float source_leaf = static_cast<float>(options_.source_voxel_size);
  source_voxel.setLeafSize(source_leaf, source_leaf, source_leaf);
  Cloud::Ptr sampled_source(new Cloud);
  source_voxel.filter(*sampled_source);
  Cloud::Ptr source(new Cloud);
  if (sampled_source->size() > options_.max_source_points) {
    source->reserve(options_.max_source_points);
    for (std::size_t index = 0U; index < options_.max_source_points; ++index) {
      source->push_back(
        (*sampled_source)[
          index * sampled_source->size() / options_.max_source_points]);
    }
  } else {
    source = sampled_source;
  }
  if (source->size() < options_.min_points) {
    result.rejection_reason = "insufficient_source_points";
    return result;
  }
  const auto raw_target = toPcl(submap_points);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(raw_target);
  const float target_leaf = static_cast<float>(options_.target_voxel_size);
  voxel.setLeafSize(target_leaf, target_leaf, target_leaf);
  Cloud::Ptr target(new Cloud);
  voxel.filter(*target);
  if (target->size() > options_.max_target_points) {
    Cloud::Ptr bounded_target(new Cloud);
    bounded_target->reserve(options_.max_target_points);
    for (std::size_t index = 0U; index < options_.max_target_points; ++index) {
      bounded_target->push_back((*target)[index * target->size() / options_.max_target_points]);
    }
    target = bounded_target;
  }
  if (target->size() < options_.min_points) {
    result.rejection_reason = "insufficient_submap_points";
    return result;
  }

  // Descriptor-centroid initialization remains useful after large odometry
  // drift, while odometry is more reliable when partial views bias centroids.
  common::Pose descriptor_initial;
  descriptor_initial.R_WI = Eigen::AngleAxisd(
    candidate.yaw_offset_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Vector3d source_centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_centroid = Eigen::Vector3d::Zero();
  for (const auto & point : *source) {
    source_centroid += Eigen::Vector3d(point.x, point.y, point.z);
  }
  for (const auto & point : *target) {
    target_centroid += Eigen::Vector3d(point.x, point.y, point.z);
  }
  source_centroid /= static_cast<double>(source->size());
  target_centroid /= static_cast<double>(target->size());
  descriptor_initial.p_WI =
    target_centroid - descriptor_initial.R_WI * source_centroid;
  const common::Pose odometry_initial = backend::relativePose(
    matched.keyframe.pose_WI, current.keyframe.pose_WI);
  common::Pose odometry_rotation_centroid_initial = odometry_initial;
  odometry_rotation_centroid_initial.p_WI =
    target_centroid - odometry_initial.R_WI * source_centroid;

  VerificationTree tree;
  tree.setInputCloud(target);
  const auto clouds = prepareGicp(source, target, options_);
  GicpHypothesis selected;
  auto odometry_hypothesis = alignHypothesis(
    source, clouds, odometry_initial, "odometry", options_, tree);
  evaluateCorrectionConsistency(
    odometry_hypothesis, current, matched, options_);
  if (preferHypothesis(
      odometry_hypothesis, selected, source->size(), options_.min_inlier_ratio))
  {
    selected = odometry_hypothesis;
  }
  if (!options_.local_refinement) {
    auto odometry_rotation_centroid_hypothesis = alignHypothesis(
      source, clouds, odometry_rotation_centroid_initial,
      "odometry_rotation_centroid", options_, tree);
    evaluateCorrectionConsistency(
      odometry_rotation_centroid_hypothesis, current, matched, options_);
    if (preferHypothesis(
        odometry_rotation_centroid_hypothesis, selected,
        source->size(), options_.min_inlier_ratio))
    {
      selected = odometry_rotation_centroid_hypothesis;
    }
    auto descriptor_hypothesis = alignHypothesis(
      source, clouds, descriptor_initial, "descriptor_centroid", options_, tree);
    evaluateCorrectionConsistency(
      descriptor_hypothesis, current, matched, options_);
    if (preferHypothesis(
        descriptor_hypothesis, selected, source->size(), options_.min_inlier_ratio))
    {
      selected = descriptor_hypothesis;
    }

    // A 6-DoF GICP can trade roll, pitch, or height for a lower residual in a
    // repetitive corridor. Run two additional hypotheses in the gravity frame,
    // where the estimator may change only world x/y/yaw. The unchanged world z
    // still contributes to correspondence distances and the later geometry
    // checks, so a cross-floor alias cannot be hidden by this constraint.
    const auto source_world = transformCloud(source, current.keyframe.pose_WI);
    const auto target_world = transformCloud(target, matched.keyframe.pose_WI);
    const auto world_clouds = prepareGicp(source_world, target_world, options_);
    auto odometry_gravity_hypothesis = alignGravityHypothesis(
      source, world_clouds, odometry_rotation_centroid_initial,
      "odometry_rotation_centroid_gravity", current, matched, options_, tree);
    evaluateCorrectionConsistency(
      odometry_gravity_hypothesis, current, matched, options_);
    if (preferHypothesis(
        odometry_gravity_hypothesis, selected,
        source->size(), options_.min_inlier_ratio))
    {
      selected = odometry_gravity_hypothesis;
    }
    auto descriptor_gravity_hypothesis = alignGravityHypothesis(
      source, world_clouds, descriptor_initial,
      "descriptor_centroid_gravity", current, matched, options_, tree);
    evaluateCorrectionConsistency(
      descriptor_gravity_hypothesis, current, matched, options_);
    if (preferHypothesis(
        descriptor_gravity_hypothesis, selected,
        source->size(), options_.min_inlier_ratio))
    {
      selected = descriptor_gravity_hypothesis;
    }
  }
  result.initialization = selected.initialization;
  // GICP returns T_matched_current. The graph edge below needs its inverse.
  if (!selected.converged) {
    result.rejection_reason = "gicp_not_converged";
    return result;
  }

  if (options_.local_refinement) {
    // A small local step can land on a different repeated structure. Probe a
    // second basin only to reject ambiguity, never to rescue a local update
    // with a global relocation. Compare clipped loss over ALL source points:
    // scoring only inliers rewards alignments that discard difficult points.
    const auto alternative = alignHypothesis(source, clouds,
      odometry_rotation_centroid_initial, "local_ambiguity_probe", options_, tree);
    const auto clipped_loss = [&](const GicpHypothesis &hypothesis) {
      return hypothesis.inliers * hypothesis.mean_squared_error +
        (source->size() - hypothesis.inliers) * options_.inlier_distance * options_.inlier_distance;
    };
    if (alternative.converged && alternative.inliers >= selected.inliers &&
      clipped_loss(alternative) <= clipped_loss(selected))
    {
      const double separation = (alternative.transform.block<3, 1>(0, 3) -
        selected.transform.block<3, 1>(0, 3)).norm();
      Eigen::Quaterniond relative_rotation((alternative.transform.block<3, 3>(0, 0) *
        selected.transform.block<3, 3>(0, 0).transpose()).cast<double>());
      relative_rotation.normalize();
      if (separation > options_.max_translation_jump ||
        Eigen::AngleAxisd(relative_rotation).angle() > options_.max_rotation_rad)
      {
        result.rejection_reason = "ambiguous_local_alignment";
        return result;
      }
    }
  }

  const auto & matrix = selected.transform;
  Eigen::Quaterniond orientation(matrix.block<3, 3>(0, 0).cast<double>());
  orientation.normalize();
  common::Pose measured{
    orientation.toRotationMatrix(), matrix.block<3, 1>(0, 3).cast<double>()};
  // Keep the complete GICP transform.  The previous yaw-only projection
  // discarded corridor roll/pitch corrections before geometric validation and
  // could leave the two passes as tilted duplicate walls.  Gravity consistency
  // is still checked by the gates below.
  const auto aligned = *transformCloud(source, measured);
  const auto metrics = correctionMetrics(measured, current, matched);
  result.global_correction = metrics.global_correction;
  result.translation_correction = metrics.translation;
  result.rotation_correction = metrics.rotation;
  result.gravity_residual_rad = metrics.gravity_residual;
  result.vertical_separation = metrics.vertical_separation;
  if (result.gravity_residual_rad > options_.max_gravity_tilt_correction_rad) {
    result.rejection_reason = "gravity_tilt_inconsistent";
    return result;
  }
  if (result.vertical_separation > options_.max_vertical_correction) {
    result.rejection_reason = "vertical_correction_too_large";
    return result;
  }

  const double max_squared = options_.inlier_distance *
    options_.inlier_distance;
  double squared_error = 0.0;
  Eigen::Matrix<double, 6, 6> geometry = Eigen::Matrix<double, 6, 6>::Zero();
  double squared_range_sum = 0.0;
  for (const auto & point : aligned) {
    squared_range_sum += Eigen::Vector3d(point.x, point.y, point.z).squaredNorm();
  }
  const double geometry_length_scale = std::max(
    1.0, std::sqrt(squared_range_sum / static_cast<double>(aligned.size())));
  std::vector<int> indices(static_cast<std::size_t>(options_.normal_neighbors));
  std::vector<float> distances(static_cast<std::size_t>(options_.normal_neighbors));
  pcl::KdTreeFLANN<pcl::PointXYZ> source_tree;
  if (options_.require_planar_normal_agreement) {
    source_tree.setInputCloud(aligned.makeShared());
  }
  std::vector<int> source_indices(static_cast<std::size_t>(options_.normal_neighbors));
  std::vector<float> source_distances(static_cast<std::size_t>(options_.normal_neighbors));
  for (const auto & point : aligned) {
    const int neighbors = tree.nearestKSearch(
      point, options_.normal_neighbors, indices, distances);
    if (neighbors < options_.normal_neighbors ||
      distances[0] > max_squared) {continue;}
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int index = 0; index < neighbors; ++index) {
      const auto & neighbor = (*target)[static_cast<std::size_t>(indices[index])];
      centroid += Eigen::Vector3d(neighbor.x, neighbor.y, neighbor.z);
    }
    centroid /= static_cast<double>(neighbors);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int index = 0; index < neighbors; ++index) {
      const auto & neighbor = (*target)[static_cast<std::size_t>(indices[index])];
      const Eigen::Vector3d delta =
        Eigen::Vector3d(neighbor.x, neighbor.y, neighbor.z) - centroid;
      covariance.noalias() += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> normal_solver(covariance);
    if (normal_solver.info() != Eigen::Success ||
      normal_solver.eigenvalues()[1] <= 0.0 ||
      normal_solver.eigenvalues()[0] >
      options_.max_normal_eigenvalue_ratio * normal_solver.eigenvalues()[1])
    {
      continue;
    }
    const Eigen::Vector3d normal = normal_solver.eigenvectors().col(0);
    if (options_.require_planar_normal_agreement) {
      const int source_neighbors = source_tree.nearestKSearch(
        point, options_.normal_neighbors, source_indices, source_distances);
      if (source_neighbors < options_.normal_neighbors) {continue;}
      Eigen::Vector3d source_centroid = Eigen::Vector3d::Zero();
      for (int index = 0; index < source_neighbors; ++index) {
        const auto & neighbor = aligned[static_cast<std::size_t>(source_indices[index])];
        source_centroid += Eigen::Vector3d(neighbor.x, neighbor.y, neighbor.z);
      }
      source_centroid /= static_cast<double>(source_neighbors);
      Eigen::Matrix3d source_covariance = Eigen::Matrix3d::Zero();
      for (int index = 0; index < source_neighbors; ++index) {
        const auto & neighbor = aligned[static_cast<std::size_t>(source_indices[index])];
        const Eigen::Vector3d delta =
          Eigen::Vector3d(neighbor.x, neighbor.y, neighbor.z) - source_centroid;
        source_covariance.noalias() += delta * delta.transpose();
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> source_solver(source_covariance);
      if (source_solver.info() != Eigen::Success ||
        source_solver.eigenvalues()[1] <= 0.0 ||
        source_solver.eigenvalues()[0] >
        options_.max_normal_eigenvalue_ratio * source_solver.eigenvalues()[1] ||
        std::abs(normal.dot(source_solver.eigenvectors().col(0))) < 0.966)
      {continue;}
    }
    ++result.inliers;
    squared_error += options_.require_planar_normal_agreement ?
      std::pow(normal.dot(
        Eigen::Vector3d(point.x, point.y, point.z) - centroid), 2) :
      distances[0];
    const Eigen::Vector3d p(point.x, point.y, point.z);
    Eigen::Matrix<double, 1, 6> jacobian;
    jacobian << -normal.transpose() * common::Hat(p), normal.transpose();
    jacobian.head<3>() /= geometry_length_scale;
    geometry.noalias() += jacobian.transpose() * jacobian;
  }
  if (result.inliers < options_.min_inliers ||
    static_cast<double>(result.inliers) / source->size() < options_.min_inlier_ratio)
  {
    result.rejection_reason = "insufficient_overlap";
    return result;
  }
  result.fitness = squared_error / static_cast<double>(result.inliers);  // m^2
  if (!std::isfinite(result.fitness) || result.fitness > options_.max_fitness) {
    result.rejection_reason = "fitness_too_high";
    return result;
  }
  VerificationTree reverse_tree;
  reverse_tree.setInputCloud(aligned.makeShared(),false);
#if ITERA_LIO_EXACT_LOOP_SCORING
  reverse_tree.prepareScores();
#endif
  std::vector<int> reverse_indices(1);
  std::vector<float> reverse_distances(1);
  std::size_t reverse_inliers = 0U;
#if ITERA_LIO_EXACT_LOOP_SCORING
  #pragma omp parallel for num_threads(options_.num_threads) schedule(static) reduction(+:reverse_inliers)
  for(std::ptrdiff_t i=0;i<static_cast<std::ptrdiff_t>(target->size());++i){
    float d;if(reverse_tree.nearestWithin((*target)[static_cast<std::size_t>(i)],max_squared,d))++reverse_inliers;
  }
#else
  for (const auto & point : *target) {
    if (reverse_tree.nearestKSearch(point, 1, reverse_indices, reverse_distances) > 0 &&
      reverse_distances[0] <= max_squared)
    {
      ++reverse_inliers;
    }
  }
#endif
  result.reciprocal_inlier_ratio = static_cast<double>(reverse_inliers) /
    static_cast<double>(target->size());
  if (result.reciprocal_inlier_ratio < options_.min_reciprocal_inlier_ratio) {
    result.rejection_reason = "insufficient_reciprocal_overlap";
    return result;
  }
  double min_eigenvalue = 0.0;
  double max_eigenvalue = 0.0;
  // Every graph pose is 6-DoF, including when planar normals are checked.
  {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(geometry);
    if (solver.info() != Eigen::Success) {
      result.rejection_reason = "invalid_geometry";
      return result;
    }
    min_eigenvalue = solver.eigenvalues().minCoeff();
    max_eigenvalue = solver.eigenvalues().maxCoeff();
  }
  if (max_eigenvalue <= 0.0) {
    result.rejection_reason = "invalid_geometry";
    return result;
  }
  result.geometry_condition_number = min_eigenvalue > 0.0 ?
    max_eigenvalue / min_eigenvalue : INFINITY;
  if (min_eigenvalue <=
    options_.min_geometry_eigenvalue_ratio * max_eigenvalue)
  {
    result.rejection_reason = "degenerate_geometry";
    return result;
  }
  // Keep the correction bound as a final safety gate. Computing geometry first
  // preserves the evidence needed to distinguish a true large-drift closure
  // from a repetitive-corridor alias without relaxing acceptance criteria.
  if (result.translation_correction > options_.max_translation_jump ||
    result.rotation_correction > options_.max_rotation_rad)
  {
    result.rejection_reason = "excessive_pose_correction";
    return result;
  }

  if (!auditLoopOverlap(held_out, submap_points, measured,
      matched.keyframe.pose_WI.R_WI.transpose() * Eigen::Vector3d::UnitZ(), options_, result))
  {
    return result;
  }

  // All stages use the same geometry-derived weighting. Average correlated
  // submap support instead of claiming N independent observations, undo the
  // observability-only length normalization, and transport the mixed tangent
  // chart through measurement inversion. This remains an approximate weight,
  // not a calibrated posterior covariance of the entire SLAM system.
  Eigen::Matrix<double, 6, 6> scale = Eigen::Matrix<double, 6, 6>::Identity();
  scale.diagonal().head<3>().setConstant(geometry_length_scale);
  const auto forward_information = (scale * geometry * scale /
    (static_cast<double>(result.inliers) * std::max(0.0025, result.fitness))).eval();
  result.information = backend::inverseMeasurementInformation(measured, forward_information);
  result.T_Ia_Ib = backend::inversePose(measured);
  result.accepted = result.T_Ia_Ib.isValid() && result.information.allFinite();
  result.rejection_reason = result.accepted ? "accepted" : "invalid_transform";
  return result;
}

}  // namespace itera_lio::loop
