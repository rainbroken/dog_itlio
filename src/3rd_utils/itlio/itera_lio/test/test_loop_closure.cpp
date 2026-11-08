// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include <cmath>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <limits>
#include <cstring>
#include <thread>

#include "itera_lio/backend/pose_graph.hpp"
#include "itera_lio/loop/loop_closure_manager.hpp"
#include "itera_lio/loop/submap_builder.hpp"
#include "itera_lio/loop/overlap_audit.hpp"

namespace
{

itera_lio::loop::KeyframeCloud makeCloud(const std::size_t id, const std::size_t point_count)
{
  itera_lio::loop::KeyframeCloud cloud;
  cloud.keyframe.id = id;
  cloud.keyframe.timestamp = static_cast<double>(id);
  cloud.keyframe.pose_WI = itera_lio::common::Pose{
    Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
  cloud.points_I.reserve(point_count);
  for (std::size_t index = 0U; index < point_count; ++index) {
    const float coordinate = static_cast<float>(index % 100U) * 0.05F;
    cloud.points_I.push_back(
      {Eigen::Vector3f(coordinate, static_cast<float>(index / 100U) * 0.05F, 1.0F), 1.0F});
  }
  return cloud;
}

itera_lio::loop::KeyframeCloud rotateCloud(
  itera_lio::loop::KeyframeCloud cloud, const double yaw)
{
  const Eigen::Matrix3f rotation =
    Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ()).toRotationMatrix();
  for (auto & point : cloud.points_I) {
    point.position = rotation * point.position;
  }
  return cloud;
}

}  // namespace

TEST(LoopOverlapAudit, ChecksFixedTransformOnWallsAndHorizontalSurfaces)
{
  std::vector<itera_lio::common::MapPoint> target, held_out;
  for (int i=-20; i<=20; ++i) {
    for (int j=-20; j<=20; ++j) {
      target.push_back({Eigen::Vector3f(.08F*i,.08F*j,0.F),1.F});
      target.push_back({Eigen::Vector3f(3.F,.08F*i,.08F*j),1.F});
      if (i%3==0 && j%3==0) {
        held_out.push_back({Eigen::Vector3f(.08F*i+.013F,.08F*j+.017F,0.F),1.F});
        held_out.push_back({Eigen::Vector3f(3.F,.08F*i+.013F,.08F*j+.017F),1.F});
      }
    }
  }
  itera_lio::loop::LoopVerifierOptions options;
  options.max_fitness=.03;
  itera_lio::loop::LoopVerificationResult result;
  itera_lio::common::Pose fixed;
  ASSERT_TRUE(itera_lio::loop::auditLoopOverlap(held_out,target,fixed,
    Eigen::Vector3d::UnitZ(),options,result));
  EXPECT_GT(result.audit_wall_points,20U);
  EXPECT_GT(result.audit_horizontal_points,20U);
  EXPECT_LT(result.audit_plane_p90,.001);
  // Most points remain within the 0.45 m NN gate, but the horizontal
  // surfaces disagree by 0.25 m. No registration is allowed to hide this.
  fixed.p_WI.z()=.25;
  EXPECT_FALSE(itera_lio::loop::auditLoopOverlap(held_out,target,fixed,
    Eigen::Vector3d::UnitZ(),options,result));
  EXPECT_STREQ(result.rejection_reason,"audit_plane_residual_too_high");
  EXPECT_NEAR(result.audit_horizontal_p90,.25,.001);
  EXPECT_LT(result.audit_wall_p90,.001);

  // A small but well-supported horizontal class must not be hidden by
  // a large wall class whose residual is unaffected by vertical translation.
  std::vector<itera_lio::common::MapPoint> mostly_walls = held_out;
  for (int repeat=0; repeat<20; ++repeat) {
    for (const auto & point : held_out) {
      if (point.position.x()>2.F) {mostly_walls.push_back(point);}
    }
  }
  EXPECT_FALSE(itera_lio::loop::auditLoopOverlap(mostly_walls,target,fixed,
    Eigen::Vector3d::UnitZ(),options,result));
  EXPECT_LT(result.audit_plane_p90,.001);
  EXPECT_NEAR(result.audit_horizontal_p90,.25,.001);
}

TEST(LoopOverlapAudit, RejectsMissingOverlap)
{
  auto cloud=makeCloud(0,500);
  itera_lio::common::Pose fixed;
  fixed.p_WI.x()=50.;
  itera_lio::loop::LoopVerificationResult result;
  EXPECT_FALSE(itera_lio::loop::auditLoopOverlap(cloud.points_I,cloud.points_I,
    fixed,Eigen::Vector3d::UnitZ(),{},result));
  EXPECT_STREQ(result.rejection_reason,"audit_insufficient_overlap");
}

TEST(LoopClosureManager, BoundsStoredCloudAndBuildsDescriptor)
{
  itera_lio::loop::LoopClosureOptions options;
  options.enabled = true;
  options.detector.min_keyframe_separation = 1U;
  options.detector.min_time_separation_sec = 0.1;
  options.max_keyframe_points = 20U;
  options.keyframe_voxel_size = 0.20F;
  itera_lio::loop::LoopClosureManager manager(options);

  const auto first = manager.addKeyframe(makeCloud(0U, 1000U));
  ASSERT_TRUE(first.keyframe_stored);
  ASSERT_EQ(manager.store().size(), 1U);
  ASSERT_EQ(manager.store().clouds().front().points_I.size(), 20U);
  EXPECT_FALSE(manager.store().clouds().front().descriptor.empty());

  const auto second = manager.addKeyframe(makeCloud(1U, 1000U));
  EXPECT_TRUE(second.keyframe_stored);
  EXPECT_EQ(manager.store().size(), 2U);
  EXPECT_FALSE(second.candidates.empty());
}

TEST(LoopDetector, RequiresKeyframeAndTemporalSeparation)
{
  itera_lio::loop::LoopDetectorOptions options;
  options.min_keyframe_separation = 2U;
  options.min_time_separation_sec = 30.0;
  itera_lio::loop::LoopDetector detector(options);
  auto old = makeCloud(0U, 20U);
  auto near_in_time = makeCloud(1U, 20U);
  auto distant = makeCloud(2U, 20U);
  auto current = makeCloud(3U, 20U);
  old.descriptor = detector.describe(old);
  near_in_time.descriptor = detector.describe(near_in_time);
  distant.descriptor = detector.describe(distant);
  current.descriptor = detector.describe(current);
  near_in_time.keyframe.timestamp = 29.0;
  distant.keyframe.pose_WI.p_WI.x() = 10.0;
  current.keyframe.timestamp = 35.0;
  const auto candidates = detector.detect(current, {old, near_in_time, distant, current});
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().matched_id, 0U);
}

TEST(LoopDetector, RejectsCrossFloorAndGravityInconsistentAliases)
{
  itera_lio::loop::LoopDetectorOptions options;
  options.min_keyframe_separation = 1U;
  options.min_time_separation_sec = 0.1;
  options.max_odometry_vertical_separation = 1.5;
  options.max_gravity_angle_rad = 0.2;
  itera_lio::loop::LoopDetector detector(options);
  auto same_floor = makeCloud(0U, 100U);
  auto cross_floor = makeCloud(1U, 100U);
  auto tilted = makeCloud(2U, 100U);
  auto current = makeCloud(3U, 100U);
  cross_floor.keyframe.pose_WI.p_WI.z() = 3.0;
  tilted.keyframe.pose_WI.R_WI =
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()).toRotationMatrix();
  current.keyframe.timestamp = 10.0;
  same_floor.descriptor = detector.describe(same_floor);
  cross_floor.descriptor = detector.describe(cross_floor);
  tilted.descriptor = detector.describe(tilted);
  current.descriptor = detector.describe(current);

  const auto candidates = detector.detect(
    current, {same_floor, cross_floor, tilted, current});

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().matched_id, same_floor.keyframe.id);
  EXPECT_NEAR(candidates.front().odometry_vertical_distance, 0.0, 1e-9);
  EXPECT_NEAR(candidates.front().gravity_angle_rad, 0.0, 1e-9);
}

TEST(LoopDetector, FindsRotatedPlaceDespiteMoreThanEightMetersOdometryDrift)
{
  constexpr double kYaw = 0.5 * 3.14159265358979323846;
  itera_lio::loop::LoopDetectorOptions options;
  options.min_keyframe_separation = 1U;
  options.min_time_separation_sec = 0.1;
  options.azimuth_bins = 72;
  options.radial_bins = 20;
  options.max_descriptor_distance = 0.05;
  itera_lio::loop::LoopDetector detector(options);

  auto past = makeCloud(0U, 0U);
  past.keyframe.timestamp = 0.0;
  for (int ring = 1; ring <= 10; ++ring) {
    for (int sector = 0; sector < 36; ++sector) {
      const double angle = static_cast<double>(sector) * 2.0 * 3.14159265358979323846 / 36.0;
      const float radius = static_cast<float>(ring * 0.7);
      const float height = static_cast<float>(0.07 * ring + 0.02 * sector);
      past.points_I.push_back(
      {
        {radius * static_cast<float>(std::cos(angle)),
          radius * static_cast<float>(std::sin(angle)), height}, 1.0F});
    }
  }
  auto current = rotateCloud(past, kYaw);
  current.keyframe.id = 40U;
  current.keyframe.timestamp = 40.0;
  current.keyframe.pose_WI.p_WI = Eigen::Vector3d(12.0, -1.0, 0.0);
  past.descriptor = detector.describe(past);
  current.descriptor = detector.describe(current);

  const auto comparison = detector.compare(current, past);
  EXPECT_TRUE(comparison.isValid()) << comparison.descriptor_distance;
  EXPECT_LT(comparison.descriptor_distance, options.max_descriptor_distance);
  const auto candidates = detector.detect(current, {past});
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().matched_id, 0U);
  EXPECT_GT(candidates.front().odometry_distance, 8.0);
  EXPECT_LT(candidates.front().descriptor_distance, options.max_descriptor_distance);
  EXPECT_NEAR(candidates.front().yaw_offset_rad, -kYaw, 0.10);
}

TEST(LoopClosureManager, RechecksRevisitsUsingOptimizedPoses)
{
  itera_lio::loop::LoopClosureOptions options;
  options.enabled = true;
  options.detector.min_keyframe_separation = 30U;
  options.detector.min_time_separation_sec = 30.0;
  options.verifier.max_iterations = 2;
  itera_lio::loop::LoopClosureManager manager(options);
  std::vector<itera_lio::backend::Keyframe> optimized;
  for (std::size_t id = 0U; id < 42U; ++id) {
    auto cloud = makeCloud(id, 5000U);
    cloud.keyframe.pose_WI.p_WI.x() = static_cast<double>(id) * 10.0;
    if (id == 31U) {
      const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
      for (auto & point : cloud.points_I) {
        point.position = (rotation.transpose() *
          (point.position.cast<double>() - Eigen::Vector3d(7.9, 0.0, 0.0))).cast<float>();
      }
    }
    optimized.push_back(cloud.keyframe);
    ASSERT_TRUE(manager.addKeyframe(std::move(cloud)).keyframe_stored);
  }
  optimized[31].pose_WI.p_WI.x() = 8.0;
  optimized[31].pose_WI.R_WI =
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto verifications = manager.reverifyOptimized(optimized);
  ASSERT_FALSE(verifications.empty());
  EXPECT_EQ(verifications.front().current_id, 31U);
  EXPECT_EQ(verifications.front().matched_id, 0U);
  EXPECT_GT(verifications.front().odometry_distance, 3.0);
  EXPECT_GT(verifications.front().gravity_angle_rad, options.detector.max_gravity_angle_rad);
}

TEST(LoopVerifier, RejectsSingleWallWithPlanarAgreement)
{
  auto matched = makeCloud(0U, 0U);
  auto current = makeCloud(40U, 0U);
  current.keyframe.timestamp = 40.0;
  for (int y = -12; y <= 12; ++y) {
    for (int z = -8; z <= 8; ++z) {
      const Eigen::Vector3f point(0.0F, y * 0.12F, z * 0.12F);
      matched.points_I.push_back({point, 1.0F});
      current.points_I.push_back({point, 1.0F});
    }
  }
  itera_lio::loop::LoopVerifierOptions options;
  options.min_points = 30U;
  options.min_inliers = 30U;
  options.require_planar_normal_agreement = true;
  options.max_correspondence_distance = 0.45;
  options.min_inlier_ratio = 0.35;
  options.min_reciprocal_inlier_ratio = 0.60;
  itera_lio::loop::LoopVerifier verifier(options);
  const auto result = verifier.verify(
    {40U, 0U, 0.1, 0.0, 0.0}, current, matched, matched.points_I);
  EXPECT_FALSE(result.accepted);
  EXPECT_STREQ(result.rejection_reason, "degenerate_geometry");
}

TEST(LoopVerifier, LocalRefinementRecoversDoorPositionAndRejectsFeaturelessCorridor)
{
  using itera_lio::common::Pose;
  auto matched = makeCloud(0U, 0U);
  auto current = makeCloud(40U, 0U);
  const Pose actual{
    Eigen::AngleAxisd(0.012, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
    Eigen::Vector3d(0.025, 0.18, 0.03)};
  const auto append = [&](const Eigen::Vector3d & point) {
      matched.points_I.push_back({point.cast<float>(), 1.0F});
      current.points_I.push_back(
        {(actual.R_WI.transpose() * (point - actual.p_WI)).cast<float>(), 1.0F});
    };
  for (int y = -60; y <= 60; ++y) {
    for (int z = -10; z <= 15; ++z) {
      for (const double x : {-1.2, 1.2}) {append({x, y * 0.08, z * 0.10});}
    }
    for (int x = -12; x <= 12; ++x) {append({x * 0.1, y * 0.08, -1.0});}
  }
  itera_lio::loop::LoopVerifierOptions options;
  options.local_refinement = true;
  options.require_planar_normal_agreement = true;
  options.source_voxel_size = options.target_voxel_size = options.gicp_voxel_size = 0.12;
  options.max_source_points = 6000;
  options.max_target_points = 20000;
  options.min_geometry_eigenvalue_ratio = 0.001;
  options.max_correspondence_distance = 0.45;
  options.inlier_distance = 0.25;
  options.max_translation_jump = 0.60;
  const itera_lio::loop::LoopVerifier verifier(options);
  const itera_lio::loop::LoopCandidate candidate{40U, 0U, 0.1};
  const auto degenerate = verifier.verify(candidate, current, matched, matched.points_I);
  EXPECT_FALSE(degenerate.accepted);
  EXPECT_STREQ(degenerate.rejection_reason, "degenerate_geometry");
  // Fixed transverse jambs provide the along-corridor information missing
  // from the parallel walls. Unequal spacings avoid an artificially periodic test.
  for (const double y : {-2.1, -1.2, 1.0, 2.2}) {
    for (int x = 12; x <= 20; ++x) {
      for (int z = -10; z <= 12; ++z) {append({x * 0.1, y, z * 0.1});}
    }
  }
  const auto refined = verifier.verify(candidate, current, matched, matched.points_I);
  ASSERT_TRUE(refined.accepted) << refined.rejection_reason;
  EXPECT_STREQ(refined.initialization, "odometry");
  const auto recovered = itera_lio::backend::inversePose(refined.T_Ia_Ib);
  EXPECT_LT((recovered.p_WI - actual.p_WI).norm(), 0.03);
  EXPECT_LT((recovered.R_WI - actual.R_WI).norm(), 0.01);
  EXPECT_GT(refined.information(4, 4), 1.0);
  EXPECT_LT((refined.information - refined.information.transpose()).norm(), 1e-9);
  // A correction larger than the local basin must not be rescued by the
  // global centroid hypotheses (which can choose another repetitive doorway).
  current.keyframe.pose_WI.p_WI.y() = 2.0;
  const auto displaced = verifier.verify(candidate, current, matched, matched.points_I);
  EXPECT_FALSE(displaced.accepted) << "initialization=" << displaced.initialization
    << " correction=" << displaced.translation_correction
    << " fitness=" << displaced.fitness
    << " measured translation=" << displaced.T_Ia_Ib.p_WI.transpose();
  EXPECT_STREQ(displaced.rejection_reason, "ambiguous_local_alignment");
}

TEST(LoopVerifier, EstimatesMatchedToCurrentGraphEdge)
{
  using itera_lio::common::Pose;
  auto matched = makeCloud(0U, 0U);
  auto current = makeCloud(40U, 0U);
  matched.keyframe.pose_WI = {Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
  const Eigen::AngleAxisd rotation(0.12, Eigen::Vector3d::UnitZ());
  const Pose actual{rotation.toRotationMatrix(), Eigen::Vector3d(1.3, -0.5, 0.2)};
  current.keyframe.pose_WI = {
    Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
    Eigen::Vector3d(10.1, -0.4, 0.2)};
  for (int x = -5; x <= 5; ++x) {
    for (int y = -5; y <= 5; ++y) {
      const Eigen::Vector3d p(0.37 * x, 0.43 * y,
        0.15 * x * y + 0.04 * x * x);
      matched.points_I.push_back({p.cast<float>(), 1.0F});
      current.points_I.push_back(
        {(actual.R_WI.transpose() *
          (p - actual.p_WI)).cast<float>(), 1.0F});
    }
  }
  itera_lio::loop::LoopVerifierOptions options;
  options.min_points = 50U;
  options.min_inliers = 80U;
  options.max_fitness = 0.1;
  options.max_correspondence_distance = 0.7;
  itera_lio::loop::LoopVerifier verifier(options);
  const auto result = verifier.verify(
    {40U, 0U, 0.2, 0.12, current.keyframe.pose_WI.p_WI.norm()},
    current, matched, matched.points_I);
  ASSERT_TRUE(result.accepted) << result.rejection_reason;
  EXPECT_GT(result.translation_correction, 8.0);
  EXPECT_GE(result.inliers, 80U);
  EXPECT_LT(result.fitness, 0.02);
  const auto expected = itera_lio::backend::inversePose(actual);
  EXPECT_NEAR((result.T_Ia_Ib.p_WI - expected.p_WI).norm(), 0.0, 0.10);
  EXPECT_NEAR((result.T_Ia_Ib.R_WI - expected.R_WI).norm(), 0.0, 0.1);

  const auto corrected_pose = itera_lio::backend::composePoses(
    matched.keyframe.pose_WI,
    itera_lio::backend::inversePose(result.T_Ia_Ib));
  const auto global_correction = itera_lio::backend::composePoses(
    corrected_pose, itera_lio::backend::inversePose(current.keyframe.pose_WI));
  EXPECT_NEAR(global_correction.R_WI(2, 0), 0.0, 1e-6);
  EXPECT_NEAR(global_correction.R_WI(2, 1), 0.0, 1e-6);
  EXPECT_NEAR(global_correction.R_WI(0, 2), 0.0, 1e-6);
  EXPECT_NEAR(global_correction.R_WI(1, 2), 0.0, 1e-6);

  current.keyframe.pose_WI.p_WI.z() = 3.0;
  const auto vertical_drift_corrected = verifier.verify(
    {40U, 0U, 0.2, 0.12, current.keyframe.pose_WI.p_WI.norm()},
    current, matched, matched.points_I);
  ASSERT_TRUE(vertical_drift_corrected.accepted) <<
    vertical_drift_corrected.rejection_reason;
  EXPECT_GT(vertical_drift_corrected.translation_correction, 2.5);
  EXPECT_LT(vertical_drift_corrected.vertical_separation, 0.3);

  const Pose cross_floor_pose{
    actual.R_WI, Eigen::Vector3d(actual.p_WI.x(), actual.p_WI.y(), 3.0)};
  auto cross_floor = current;
  cross_floor.points_I.clear();
  for (const auto & point : matched.points_I) {
    cross_floor.points_I.push_back(
      {(cross_floor_pose.R_WI.transpose() *
        (point.position.cast<double>() - cross_floor_pose.p_WI)).cast<float>(), 1.0F});
  }
  const auto vertical_rejected = verifier.verify(
    {40U, 0U, 0.2, 0.12, cross_floor.keyframe.pose_WI.p_WI.norm()},
    cross_floor, matched, matched.points_I);
  EXPECT_FALSE(vertical_rejected.accepted);
  EXPECT_STREQ(vertical_rejected.rejection_reason, "vertical_correction_too_large");
  current.keyframe.pose_WI.p_WI.z() = 0.2;

  for (auto & point : matched.points_I) {
    point.position.array() += 20.0F;
  }
  const auto rejected = verifier.verify({40U, 0U, 0.2}, current, matched, matched.points_I);
  EXPECT_FALSE(rejected.accepted);
}

TEST(LoopVerifier, ManualPreprocessingReuseDoesNotTrustCloudIdOrOldOptions)
{
  auto matched=makeCloud(0U,0U);auto current=makeCloud(40U,0U);
  for(int x=-6;x<=6;++x)for(int y=-6;y<=6;++y){
    const Eigen::Vector3f p(.37F*x,.43F*y,.15F*x*y+.04F*x*x);
    matched.points_I.push_back({p,1.F});current.points_I.push_back({p,1.F});
  }
  itera_lio::loop::LoopVerifierOptions options;options.registration_engine="manual_gicp";
  options.min_points=50;options.min_inliers=50;options.source_voxel_size=.1;
  options.target_voxel_size=.1;options.gicp_voxel_size=.15;options.max_correspondence_distance=.7;
  auto check=[&](const auto& opts,double expected){
    itera_lio::loop::LoopVerifier verifier(opts);
    const auto result=verifier.verify({40U,0U,.1,0.,0.},current,matched,matched.points_I);
    ASSERT_TRUE(result.accepted)<<result.rejection_reason;
    EXPECT_NEAR(result.T_Ia_Ib.p_WI.x(),expected,.03);
    EXPECT_LT(result.fitness,.01);
  };
  check(options,0.);check(options,0.);
  // Same ID and same point count, changed geometry and pose.
  for(auto& p:current.points_I)p.position.x()+=.35F;
  current.keyframe.pose_WI.p_WI.x()=-.35;
  check(options,.35);check(options,.35);
  options.gicp_voxel_size=.25;options.normal_neighbors=12;
  check(options,.35);
  options.num_threads=1;check(options,.35);
}

TEST(LoopVerifier, AllowsDifferentHandheldPitchAtSamePlace)
{
  using itera_lio::common::Pose;
  auto matched = makeCloud(0U, 0U);
  auto current = makeCloud(40U, 0U);
  const Pose actual{
    Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitX()).toRotationMatrix(),
    Eigen::Vector3d(0.5, 0.2, 0.1)};
  current.keyframe.pose_WI = actual;
  current.keyframe.pose_WI.p_WI.x() += 0.3;
  for (int x = -5; x <= 5; ++x) {
    for (int y = -5; y <= 5; ++y) {
      const Eigen::Vector3d point(0.37 * x, 0.43 * y,
        0.15 * x * y + 0.04 * x * x);
      matched.points_I.push_back({point.cast<float>(), 1.0F});
      current.points_I.push_back(
        {(actual.R_WI.transpose() * (point - actual.p_WI)).cast<float>(), 1.0F});
    }
  }
  itera_lio::loop::LoopVerifierOptions options;
  options.min_points = 50U;
  options.min_inliers = 80U;
  options.max_fitness = 0.1;
  options.max_correspondence_distance = 0.7;
  itera_lio::loop::LoopVerifier verifier(options);
  const auto result = verifier.verify(
    {40U, 0U, 0.2, 0.0, 0.3}, current, matched, matched.points_I);
  ASSERT_TRUE(result.accepted) << result.rejection_reason;
  EXPECT_LT(result.gravity_residual_rad, 0.01);
}

TEST(LoopVerifier, UsesOdometryWhenPartialViewBiasesCentroid)
{
  using itera_lio::common::Pose;
  auto matched = makeCloud(0U, 0U);
  auto current = makeCloud(40U, 0U);
  const Eigen::AngleAxisd rotation(0.08, Eigen::Vector3d::UnitZ());
  const Pose actual{rotation.toRotationMatrix(), Eigen::Vector3d(1.0, -0.3, 0.1)};
  current.keyframe.pose_WI = {
    rotation.toRotationMatrix(), Eigen::Vector3d(1.15, -0.25, 0.1)};
  for (int x = -5; x <= 5; ++x) {
    for (int y = -5; y <= 5; ++y) {
      const Eigen::Vector3d point(
        0.37 * x, 0.43 * y, 0.15 * x * y + 0.04 * x * x);
      matched.points_I.push_back({point.cast<float>(), 1.0F});
      current.points_I.push_back(
        {(actual.R_WI.transpose() * (point - actual.p_WI)).cast<float>(), 1.0F});
    }
  }
  auto submap = matched.points_I;
  for (int index = 0; index < 100; ++index) {
    submap.push_back(
      {Eigen::Vector3f(
          8.0F + 0.1F * static_cast<float>(index % 10),
          0.1F * static_cast<float>(index / 10),
          2.0F + 0.03F * static_cast<float>(index)),
        1.0F});
  }

  itera_lio::loop::LoopVerifierOptions options;
  options.min_points = 50U;
  options.min_inliers = 80U;
  options.max_fitness = 0.1;
  options.max_correspondence_distance = 0.7;
  itera_lio::loop::LoopVerifier verifier(options);
  const auto result = verifier.verify(
    {40U, 0U, 0.2, 0.08, 1.2}, current, matched, submap);

  ASSERT_TRUE(result.accepted) << result.rejection_reason;
  EXPECT_STREQ(result.initialization, "odometry_rotation_centroid_gravity");
  EXPECT_NEAR(result.translation_correction, 0.15, 0.10);
  EXPECT_NEAR(result.global_correction.p_WI.z(), 0.0, 1e-7);
  EXPECT_NEAR(
    Eigen::Vector3d::UnitZ().dot(
      result.global_correction.R_WI * Eigen::Vector3d::UnitZ()),
    1.0, 1e-7);
}

TEST(LoopConfirmationGate, RequiresDistinctCurrentKeyframes)
{
  itera_lio::loop::LoopConfirmationGate gate;
  itera_lio::loop::LoopVerificationResult first;
  first.current_id = 40U;
  first.matched_id = 0U;
  first.global_correction.p_WI = Eigen::Vector3d(55.0, 0.0, 0.0);
  auto same_current = first;
  same_current.matched_id = 1U;
  same_current.global_correction.p_WI = Eigen::Vector3d(55.2, 0.0, 0.0);
  auto next_current = first;
  next_current.current_id = 41U;
  next_current.global_correction.p_WI = Eigen::Vector3d(55.3, 0.0, 0.0);

  EXPECT_EQ(
    gate.observe(first, 2U, 2.0, 0.1),
    itera_lio::loop::LoopConfirmationStatus::kAwaiting);
  EXPECT_EQ(
    gate.observe(same_current, 2U, 2.0, 0.1),
    itera_lio::loop::LoopConfirmationStatus::kDuplicateCurrentKeyframe);
  EXPECT_EQ(gate.count(), 1U);
  EXPECT_EQ(
    gate.observe(next_current, 2U, 2.0, 0.1),
    itera_lio::loop::LoopConfirmationStatus::kConfirmed);
}

TEST(LoopConfirmationGate, StrongSingleFrameRequiresAllEvidenceMargins)
{
  itera_lio::loop::StrongLoopEvidenceOptions options;
  options.enabled = true;
  itera_lio::loop::LoopVerificationResult verification;
  verification.current_id = 503U;
  verification.matched_id = 0U;
  verification.accepted = true;
  verification.inliers = 174U;
  verification.descriptor_distance = 0.146;
  verification.fitness = 0.0234;
  verification.reciprocal_inlier_ratio = 0.965;
  verification.geometry_condition_number = 19.1;
  verification.gravity_residual_rad = 0.029;
  verification.vertical_separation = 0.037;

  EXPECT_TRUE(itera_lio::loop::isStrongLoopEvidence(verification, options));
  verification.fitness = 0.026;
  EXPECT_FALSE(itera_lio::loop::isStrongLoopEvidence(verification, options));
  verification.fitness = 0.0234;
  verification.reciprocal_inlier_ratio = 0.85;
  EXPECT_FALSE(itera_lio::loop::isStrongLoopEvidence(verification, options));
  verification.reciprocal_inlier_ratio = 0.965;
  options.enabled = false;
  EXPECT_FALSE(itera_lio::loop::isStrongLoopEvidence(verification, options));
}

TEST(LoopVerifier, ReportsGeometryBeforeRejectingExcessiveCorrection)
{
  using itera_lio::common::Pose;
  auto matched = makeCloud(0U, 0U);
  auto current = makeCloud(40U, 0U);
  const Eigen::Matrix3d actual_rotation =
    (Eigen::AngleAxisd(0.12, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitX())).toRotationMatrix();
  const Pose actual{actual_rotation, Eigen::Vector3d(1.0, -0.3, 0.1)};
  current.keyframe.pose_WI = {
    actual_rotation, Eigen::Vector3d(55.0, -0.3, 0.1)};
  for (int x = -5; x <= 5; ++x) {
    for (int y = -5; y <= 5; ++y) {
      const Eigen::Vector3d point(
        0.37 * x, 0.43 * y, 0.15 * x * y + 0.04 * x * x);
      matched.points_I.push_back({point.cast<float>(), 1.0F});
      current.points_I.push_back(
        {(actual.R_WI.transpose() * (point - actual.p_WI)).cast<float>(), 1.0F});
    }
  }
  itera_lio::loop::LoopVerifierOptions options;
  options.min_points = 50U;
  options.min_inliers = 80U;
  options.max_fitness = 0.1;
  options.max_correspondence_distance = 0.7;
  options.max_translation_jump = 80.0;
  itera_lio::loop::LoopVerifier permissive_verifier(options);
  const auto accepted = permissive_verifier.verify(
    {40U, 0U, 0.1, 0.12, 55.0}, current, matched, matched.points_I);
  ASSERT_TRUE(accepted.accepted) << accepted.rejection_reason;
  EXPECT_STRNE(accepted.initialization, "none");

  options.max_translation_jump = 20.0;
  itera_lio::loop::LoopVerifier verifier(options);

  const auto result = verifier.verify(
    {40U, 0U, 0.1, 0.12, 55.0}, current, matched, matched.points_I);

  EXPECT_FALSE(result.accepted);
  EXPECT_STREQ(result.rejection_reason, "excessive_pose_correction");
  EXPECT_STRNE(result.initialization, "none");
  EXPECT_GE(result.inliers, 80U);
  EXPECT_GT(result.reciprocal_inlier_ratio, 0.25);
  EXPECT_GT(result.geometry_condition_number, 1.0);
}

TEST(KeyframeCloudStore, ConcurrentReadDuringInsertionAndEviction)
{
  itera_lio::loop::KeyframeCloudStore store(2U);
  ASSERT_TRUE(store.add(makeCloud(0U, 100U)));
  std::atomic<bool> finished{false};
  std::atomic<bool> valid{true};
  std::atomic<std::size_t> reads{0U};
  std::thread reader([&]() {
    do {
      itera_lio::loop::KeyframeCloud cloud;
      if (!store.get(0U, cloud) || cloud.points_I.size() != 100U ||
          cloud.keyframe.id != 0U) {
        valid.store(false);
      }
      ++reads;
    } while (!finished.load());
  });
  bool inserted = true;
  for (std::size_t id = 1U; id < 100U; ++id) {
    inserted = store.add(makeCloud(id, 100U)) && inserted;
  }
  finished.store(true);
  reader.join();
  EXPECT_TRUE(inserted);
  EXPECT_TRUE(valid.load());
  EXPECT_GT(reads.load(), 0U);
  EXPECT_EQ(store.stats().resident_clouds, 2U);
}

TEST(LoopSubmap, ExcludesRecentKeyframesAndUsesMatchedFrame)
{
  itera_lio::loop::KeyframeCloudStore store;
  for (std::size_t id = 0; id < 4U; ++id) {
    auto cloud = makeCloud(id, 1U);
    cloud.keyframe.pose_WI.p_WI.x() = static_cast<double>(id);
    ASSERT_TRUE(store.add(std::move(cloud)));
  }
  const auto points = itera_lio::loop::buildLoopSubmap(store, 1U, 3U, 2U, 2U);
  ASSERT_EQ(points.size(), 2U);
  EXPECT_NEAR(points[0].position.x(), -1.0, 1e-6);
  EXPECT_NEAR(points[1].position.x(), 0.0, 1e-6);
}

TEST(LoopSubmap, BuildsRecentCloudsInCurrentFrame)
{
  itera_lio::loop::KeyframeCloudStore store;
  for (std::size_t id = 0; id < 4U; ++id) {
    auto cloud = makeCloud(id, 1U);
    cloud.keyframe.pose_WI.p_WI.x() = static_cast<double>(id);
    ASSERT_TRUE(store.add(std::move(cloud)));
  }
  const auto points = itera_lio::loop::buildCurrentLoopSubmap(store, 3U, 2U);
  ASSERT_EQ(points.size(), 3U);
  EXPECT_NEAR(points[0].position.x(), -2.0, 1e-6);
  EXPECT_NEAR(points[1].position.x(), -1.0, 1e-6);
  EXPECT_NEAR(points[2].position.x(), 0.0, 1e-6);
}

TEST(LoopVerifier, RejectsAxiallyAmbiguousStraightCorridor)
{
  auto matched = makeCloud(0U, 0U);
  auto current = makeCloud(40U, 0U);
  current.keyframe.timestamp = 40.0;
  for (int x = -20; x <= 20; ++x) {
    for (int z = 0; z <= 4; ++z) {
      for (const float y : {-2.0F, 2.0F}) {
        const itera_lio::common::MapPoint point{
          Eigen::Vector3f(0.25F * x, y, 0.5F * z), 1.0F};
        matched.points_I.push_back(point);
        current.points_I.push_back(point);
      }
    }
  }
  itera_lio::loop::LoopVerifierOptions options;
  options.min_points = 100U;
  options.min_inliers = 100U;
  options.max_source_points = 1000U;
  options.max_target_points = 1000U;
  itera_lio::loop::LoopVerifier verifier(options);
  const auto result = verifier.verify(
    {40U, 0U, 0.1, 0.0, 0.0}, current, matched, matched.points_I);
  EXPECT_FALSE(result.accepted);
  EXPECT_STREQ(result.rejection_reason, "degenerate_geometry");
}

TEST(OptimizedMap, ReprojectsOriginalCloudsWithOptimizedPoses)
{
  itera_lio::loop::KeyframeCloudStore store;
  auto first = makeCloud(0U, 0U);
  first.points_I.push_back({Eigen::Vector3f(1.0F, 0.0F, 0.0F), 1.0F});
  auto second = makeCloud(1U, 0U);
  second.points_I.push_back({Eigen::Vector3f(0.0F, 1.0F, 0.0F), 2.0F});
  ASSERT_TRUE(store.add(first));
  ASSERT_TRUE(store.add(second));
  std::vector<itera_lio::backend::Keyframe> optimized{first.keyframe, second.keyframe};
  optimized[0].pose_WI.p_WI = Eigen::Vector3d(10.0, 0.0, 0.0);
  optimized[1].pose_WI.p_WI = Eigen::Vector3d(0.0, 20.0, 0.0);
  const auto map = itera_lio::loop::rebuildOptimizedMap(store, optimized, 0.01F);
  ASSERT_EQ(map.size(), 2U);
  EXPECT_TRUE(map[0].position.isApprox(Eigen::Vector3f(11.0F, 0.0F, 0.0F)));
  EXPECT_TRUE(map[1].position.isApprox(Eigen::Vector3f(0.0F, 21.0F, 0.0F)));
}

TEST(KeyframeCloudStore, ArchivesAndReloadsCloudsBeyondResidentLimit)
{
  const auto archive_root = std::filesystem::temp_directory_path() /
    "itera_lio_keyframe_store_test";
  {
    itera_lio::loop::KeyframeCloudStore store(1U, archive_root.string());
    ASSERT_TRUE(store.add(makeCloud(0U, 20U)));
    ASSERT_TRUE(store.add(makeCloud(1U, 20U)));
    const auto stats_after_add = store.stats();
    EXPECT_EQ(stats_after_add.resident_clouds, 1U);
    EXPECT_EQ(stats_after_add.archived_clouds, 1U);
    EXPECT_EQ(stats_after_add.archive_writes, 1U);
    EXPECT_TRUE(store.clouds().front().points_I.empty());

    itera_lio::loop::KeyframeCloud restored;
    ASSERT_TRUE(store.get(0U, restored));
    EXPECT_EQ(restored.points_I.size(), 20U);
    EXPECT_EQ(store.stats().archive_reads, 1U);
  }
  std::error_code error;
  std::filesystem::remove_all(archive_root, error);
}

TEST(KeyframeCloudStore, BulkPayloadMatchesLegacyBytesAndLoadsLegacyFile)
{
  const auto root = std::filesystem::temp_directory_path() / "b017_legacy_compat";
  std::filesystem::create_directories(root);
  {
    itera_lio::loop::KeyframeCloudStore store(1U, root.string());
    auto input = makeCloud(0U, 2049U);
    input.points_I[0] = {Eigen::Vector3f(-0.0F, -123.25F, 1e-20F), 256.5F};
    ASSERT_TRUE(store.add(input));
    ASSERT_TRUE(store.add(makeCloud(1U, 2U)));
    const auto file = std::filesystem::directory_iterator(root)->path() / "0.bin";
    std::ostringstream legacy(std::ios::binary | std::ios::out);
    const auto write = [&](const auto & value) {
        legacy.write(reinterpret_cast<const char *>(&value), sizeof(value));
      };
    write(std::uint32_t{0x4B46434CU});
    write(input.keyframe.id);
    write(static_cast<std::uint64_t>(input.points_I.size()));
    for (const auto & p : input.points_I) {
      write(p.position.x()); write(p.position.y()); write(p.position.z()); write(p.intensity);
    }
    std::ifstream saved(file, std::ios::binary);
    const std::string bytes{std::istreambuf_iterator<char>(saved), std::istreambuf_iterator<char>()};
    ASSERT_EQ(bytes, legacy.str());
    saved.close();
    // Explicit legacy scalar serializer output, read by the new bulk loader.
    std::ofstream old_file(file, std::ios::binary | std::ios::trunc);
    old_file << legacy.str(); old_file.close();
    itera_lio::loop::KeyframeCloud restored;
    ASSERT_TRUE(store.get(0U, restored));
    ASSERT_EQ(restored.points_I.size(), input.points_I.size());
    for (std::size_t i = 0; i < input.points_I.size(); ++i) {
      EXPECT_EQ(std::memcmp(restored.points_I[i].position.data(), input.points_I[i].position.data(),
        3U * sizeof(float)), 0);
      EXPECT_EQ(std::memcmp(&restored.points_I[i].intensity, &input.points_I[i].intensity,
        sizeof(float)), 0);
    }
  }
  std::filesystem::remove(root);
}

TEST(KeyframeCloudStore, RejectsTruncatedAndOverflowedPayloadBeforeAllocation)
{
  const auto root = std::filesystem::temp_directory_path() / "b017_corrupt_archive";
  std::filesystem::create_directories(root);
  {
    itera_lio::loop::KeyframeCloudStore store(1U, root.string());
    ASSERT_TRUE(store.add(makeCloud(0U, 20U)));
    ASSERT_TRUE(store.add(makeCloud(1U, 2U)));
    const auto file = std::filesystem::directory_iterator(root)->path() / "0.bin";
    std::filesystem::resize_file(file, std::filesystem::file_size(file) - 1U);
    itera_lio::loop::KeyframeCloud restored;
    EXPECT_FALSE(store.get(0U, restored));
    const auto count = std::numeric_limits<std::uint64_t>::max();
    std::fstream corrupt(file, std::ios::binary | std::ios::in | std::ios::out);
    corrupt.seekp(sizeof(std::uint32_t) + sizeof(itera_lio::common::KeyframeId));
    corrupt.write(reinterpret_cast<const char *>(&count), sizeof(count)); corrupt.close();
    EXPECT_FALSE(store.get(0U, restored));
    EXPECT_EQ(store.stats().archive_errors, 2U);
  }
  std::filesystem::remove(root);
}

TEST(LoopClosureManager, ExpiresQueuedVerificationAtDeadline)
{
  itera_lio::loop::LoopClosureOptions options;
  options.enabled = true;
  options.detector.min_keyframe_separation = 1U;
  options.detector.min_time_separation_sec = 0.1;
  options.min_attempt_interval_sec = 0.0;
  options.task_timeout_ms = 1e-9;
  options.max_keyframe_points = 100U;
  options.verifier.min_points = 10U;
  options.verifier.min_inliers = 10U;
  options.verifier.max_source_points = 100U;
  options.verifier.max_target_points = 100U;
  itera_lio::loop::LoopClosureManager manager(options);
  ASSERT_TRUE(manager.addKeyframe(makeCloud(0U, 100U)).keyframe_stored);
  auto current = makeCloud(1U, 100U);
  current.keyframe.timestamp = 1.0;
  ASSERT_FALSE(manager.addKeyframe(std::move(current)).candidates.empty());
  for (int attempt = 0; attempt < 100 && manager.workerStats().timed_out == 0U; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto stats = manager.workerStats();
  EXPECT_EQ(stats.submitted, 1U);
  EXPECT_EQ(stats.timed_out, 1U);
  EXPECT_EQ(stats.queue_depth, 0U);
}

TEST(LoopClosureManager, RejectsInvalidCloudOptions)
{
  itera_lio::loop::LoopClosureOptions options;
  options.keyframe_voxel_size = 0.0F;
  EXPECT_THROW(itera_lio::loop::LoopClosureManager manager(options), std::invalid_argument);
}

TEST(LoopVerifier, ValidatesGicpAndEvaluationOptions)
{
  itera_lio::loop::LoopVerifierOptions options;
  options.inlier_distance = 0.0;
  EXPECT_THROW(itera_lio::loop::LoopVerifier{options}, std::invalid_argument);
  options.inlier_distance = 0.45;
  options.num_threads = 0;
  EXPECT_THROW(itera_lio::loop::LoopVerifier{options}, std::invalid_argument);
  options.num_threads = 1;
  options.robust_kernel_scale = -1.0;
  EXPECT_THROW(itera_lio::loop::LoopVerifier{options}, std::invalid_argument);
}
