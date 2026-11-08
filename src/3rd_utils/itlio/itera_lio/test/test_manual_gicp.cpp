// Copyright 2026 rainbroken
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <cstdlib>
#include <chrono>
#include "itera_lio/loop/loop_pair_archive.hpp"
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include "itera_lio/loop/manual_gicp.hpp"
#include "itera_lio/loop/loop_verifier.hpp"
#include "itera_lio/common/so3.hpp"
#include "itera_lio/backend/pose_graph.hpp"
namespace {
using namespace itera_lio;
std::vector<Eigen::Vector3d> scene() {
  std::vector<Eigen::Vector3d> points;
  for(int i=0;i<27;++i) for(int j=0;j<23;++j) {
    const double u=.09*i+.007*std::sin(j*1.31),v=.09*j+.011*std::cos(i*.77);
    points.emplace_back(u,v,.003*std::sin(i*.3+j*.4));
    points.emplace_back(-.45,u,.12+v);points.emplace_back(u,2.7,.23+v);
  }
  return points;
}
std::vector<Eigen::Vector3d> inverseTransform(const std::vector<Eigen::Vector3d>& target,const common::Pose& truth) {
  std::vector<Eigen::Vector3d> source;
  for(const auto& p:target) source.emplace_back(truth.R_WI.transpose()*(p-truth.p_WI));
  return source;
}
common::Pose truthPose() {return {common::ExpSO3({.035,-.026,.043}),{.12,-.08,.055}};}
loop::KeyframeCloud keyframe(std::size_t id,const std::vector<Eigen::Vector3d>& points,const common::Pose& pose) {
  loop::KeyframeCloud cloud;cloud.keyframe.id=id;cloud.keyframe.timestamp=id*40.;cloud.keyframe.pose_WI=pose;
  for(const auto&p:points) cloud.points_I.push_back({p.cast<float>(),1.F});
  return cloud;
}
}
TEST(ManualGicp, RightPerturbationJacobianMatchesCentralDifferences) {
  const auto pose=truthPose();
  for(const Eigen::Vector3d & p:std::vector<Eigen::Vector3d>{{.3,-1.7,.8},{4.1,.02,-.4},{-.2,.6,2.}}) {
    Eigen::Matrix<double,3,6> numeric;
    for(int j=0;j<6;++j) {
      Eigen::Matrix<double,6,1> d=Eigen::Matrix<double,6,1>::Zero();d[j]=1e-7;
      const auto a=loop::manualGicpRetract(pose,d),b=loop::manualGicpRetract(pose,-d);
      numeric.col(j)=(a.R_WI*p+a.p_WI-b.R_WI*p-b.p_WI)/(2e-7);
    }
    EXPECT_LT((numeric-loop::manualGicpResidualJacobian(pose,p)).cwiseAbs().maxCoeff(),1e-8);
  }
}
TEST(ManualGicp, CovariancesAreFinitePositiveAndPlanarRegularized) {
  const auto cloud=loop::prepareManualGicp(scene(),.035,10,2);ASSERT_GT(cloud.points.size(),100U);
  for(const auto& C:cloud.covariances) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(C);
    ASSERT_EQ(eig.info(),Eigen::Success);EXPECT_GT(eig.eigenvalues().minCoeff(),.0009);
    EXPECT_LE(eig.eigenvalues().maxCoeff(),1.000001);
  }
}
TEST(ManualGicp, RecoversKnownSixDofTransformWithoutRegistrationLibrary) {
  const auto points=scene();const auto truth=truthPose();
  const auto target=loop::prepareManualGicp(points,.035,10,2),source=loop::prepareManualGicp(inverseTransform(points,truth),.035,10,2);
  loop::ManualGicpOptions options;options.max_iterations=80;
  const auto r=loop::alignManualGicp(target,source,{},options);ASSERT_TRUE(r.converged)<<r.reason;
  EXPECT_LT((r.T_target_source.p_WI-truth.p_WI).norm(),.002);
  EXPECT_LT(common::LogSO3(r.T_target_source.R_WI.transpose()*truth.R_WI).norm(),.001);
  EXPECT_GT(r.correspondences,1000U);
}
TEST(ManualGicp, StableAtCorrectNonidentityInitialization) {
  const auto points=scene();const auto truth=truthPose();
  const auto target=loop::prepareManualGicp(points,.025,10,1),source=loop::prepareManualGicp(inverseTransform(points,truth),.025,10,1);
  const auto r=loop::alignManualGicp(target,source,truth);ASSERT_TRUE(r.converged)<<r.reason;
  EXPECT_LT((r.T_target_source.p_WI-truth.p_WI).norm(),.001);
}
TEST(ManualGicp, PreservesExactWorldXYYawSubgroup) {
  const auto points=scene();common::Pose truth{common::ExpSO3({0,0,.047}),{.11,-.07,0}};
  const auto target=loop::prepareManualGicp(points,.035,10,2),source=loop::prepareManualGicp(inverseTransform(points,truth),.035,10,2);
  loop::ManualGicpOptions options;options.xy_yaw_only=true;options.max_iterations=80;
  const auto r=loop::alignManualGicp(target,source,{},options);ASSERT_TRUE(r.converged)<<r.reason;
  EXPECT_LT((r.T_target_source.p_WI-truth.p_WI).norm(),.002);EXPECT_NEAR(r.T_target_source.p_WI.z(),0.,1e-12);
  EXPECT_LT((r.T_target_source.R_WI*Eigen::Vector3d::UnitZ()-Eigen::Vector3d::UnitZ()).norm(),1e-12);
  EXPECT_FALSE(loop::alignManualGicp(target,source,truthPose(),options).converged);
}
TEST(ManualGicp, RejectsDisjointCloudsAndEmptyInput) {
  auto points=scene(),far=points;for(auto& p:far)p+=Eigen::Vector3d(30,30,30);
  const auto target=loop::prepareManualGicp(points,.04,10,1),source=loop::prepareManualGicp(far,.04,10,1);
  const auto r=loop::alignManualGicp(target,source,{});EXPECT_FALSE(r.converged);EXPECT_EQ(r.reason,"insufficient_correspondences");
  EXPECT_FALSE(loop::alignManualGicp(target,loop::prepareManualGicp({},.04,10,1),{}).converged);
}
TEST(ManualGicp, IgnoresDistantOutliersWithoutClaimingTheyAreCorrespondences) {
  const auto points=scene();const auto truth=truthPose();auto source_points=inverseTransform(points,truth);
  for(int i=0;i<400;++i)source_points.emplace_back(30+i*.03,-20,40);
  const auto r=loop::alignManualGicp(loop::prepareManualGicp(points,.035,10,2),loop::prepareManualGicp(source_points,.035,10,2),{});
  ASSERT_TRUE(r.converged)<<r.reason;EXPECT_LT((r.T_target_source.p_WI-truth.p_WI).norm(),.002);EXPECT_LT(r.correspondences,source_points.size());
}
TEST(ManualGicp, ThreadCountDoesNotChangeAcceptedGeometryBeyondRoundoff) {
  const auto points=scene();const auto truth=truthPose();
  const auto target=loop::prepareManualGicp(points,.035,10,2),source=loop::prepareManualGicp(inverseTransform(points,truth),.035,10,2);
  loop::ManualGicpOptions one;one.num_threads=1;auto two=one;two.num_threads=2;
  const auto a=loop::alignManualGicp(target,source,{},one),b=loop::alignManualGicp(target,source,{},two);
  ASSERT_TRUE(a.converged);ASSERT_TRUE(b.converged);EXPECT_LT((a.T_target_source.p_WI-b.T_target_source.p_WI).norm(),1e-9);
}
TEST(ManualGicp, RejectsInvalidOptionsAndInitialRotation) {
  EXPECT_THROW(loop::prepareManualGicp({},0.,10,1),std::invalid_argument);
  const auto cloud=loop::prepareManualGicp(scene(),.04,10,1);loop::ManualGicpOptions options;options.cauchy_scale=0;
  EXPECT_THROW(loop::alignManualGicp(cloud,cloud,{},options),std::invalid_argument);
  common::Pose bad;bad.R_WI.setZero();EXPECT_FALSE(loop::alignManualGicp(cloud,cloud,bad).converged);
}
TEST(ManualLoopVerifier, UsesOwnCoreAndReturnsCorrectGraphEdgeDirection) {
  const auto points=scene();const auto truth=truthPose();
  auto matched=keyframe(0,points,{}),current=keyframe(40,inverseTransform(points,truth),truth);
  loop::LoopVerifierOptions options;options.registration_engine="manual_gicp";options.source_voxel_size=.04;options.target_voxel_size=.04;
  options.gicp_voxel_size=.035;options.max_source_points=4000;options.max_target_points=4000;
  loop::LoopCandidate candidate;candidate.current_id=40;candidate.matched_id=0;
  const auto r=loop::LoopVerifier(options).verify(candidate,current,matched,matched.points_I);ASSERT_TRUE(r.isValid())<<r.rejection_reason;
  const auto identity=backend::composePoses(truth,r.T_Ia_Ib);
  EXPECT_LT(identity.p_WI.norm(),.002);EXPECT_LT(common::LogSO3(identity.R_WI).norm(),.001);
}
TEST(ManualLoopVerifier, RetainsDegeneratePlaneRejection) {
  std::vector<Eigen::Vector3d> points;for(int i=0;i<35;++i)for(int j=0;j<35;++j)points.emplace_back(i*.08,j*.08,0.);
  const auto matched=keyframe(0,points,{}),current=keyframe(40,points,{});
  loop::LoopVerifierOptions options;options.registration_engine="manual_gicp";options.source_voxel_size=.04;options.target_voxel_size=.04;options.gicp_voxel_size=.035;
  loop::LoopCandidate candidate;candidate.current_id=40;candidate.matched_id=0;
  const auto r=loop::LoopVerifier(options).verify(candidate,current,matched,matched.points_I);
  EXPECT_FALSE(r.isValid());EXPECT_STREQ(r.rejection_reason,"degenerate_geometry");
}
TEST(ManualLoopVerifier, RejectsUnknownRegistrationEngine) {
  loop::LoopVerifierOptions options;options.registration_engine="typo";EXPECT_THROW(loop::LoopVerifier{options},std::invalid_argument);
}

TEST(ManualLoopArchive, PersistsExactInputAndRejectsOverwrite) {
  const char* artifacts=std::getenv("ITERA_LIO_TEST_ARTIFACTS");
  ASSERT_NE(artifacts,nullptr);
  const auto path=std::filesystem::path(artifacts)/
    ("pair_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto points=scene();const auto truth=truthPose();
  loop::LoopPairInput input;input.options.registration_engine="manual_gicp";
  input.options.require_planar_normal_agreement=true;input.options.max_iterations=17;
  input.current=keyframe(40,inverseTransform(points,truth),truth);input.matched=keyframe(0,points,{});
  input.submap=input.matched.points_I;input.candidate.current_id=40;input.candidate.matched_id=0;
  input.candidate.yaw_offset_rad=.05123456789012345;
  loop::saveLoopPair(path,input);const auto loaded=loop::loadLoopPair(path);
  EXPECT_EQ(loaded.options.registration_engine,input.options.registration_engine);
  EXPECT_EQ(loaded.options.max_iterations,17);EXPECT_TRUE(loaded.options.require_planar_normal_agreement);
  EXPECT_EQ(loaded.current.keyframe.pose_WI.R_WI,truth.R_WI);
  EXPECT_EQ(loaded.current.keyframe.pose_WI.p_WI,truth.p_WI);
  EXPECT_DOUBLE_EQ(loaded.candidate.yaw_offset_rad,input.candidate.yaw_offset_rad);
  ASSERT_EQ(loaded.current.points_I.size(),input.current.points_I.size());
  for(std::size_t i=0;i<loaded.current.points_I.size();++i)
    EXPECT_EQ(loaded.current.points_I[i].position,input.current.points_I[i].position);
  EXPECT_THROW(loop::saveLoopPair(path,input),std::runtime_error);
}

TEST(ManualGicp, CustomSpdCovariancesRetainCorrectTransform) {
  const auto points=scene();const auto truth=truthPose();
  auto target=loop::prepareManualGicp(points,.035,10,2);
  auto source=loop::prepareManualGicp(inverseTransform(points,truth),.035,10,2);
  for(auto& C:target.covariances)C=Eigen::Vector3d(.1,.3,.9).asDiagonal();
  for(auto& C:source.covariances)C=Eigen::Vector3d(.07,.2,.9).asDiagonal();
  loop::ManualGicpOptions options;options.max_iterations=80;
  auto r=loop::alignManualGicp(target,source,{},options);
  ASSERT_TRUE(r.converged)<<r.reason;EXPECT_LT((r.T_target_source.p_WI-truth.p_WI).norm(),.003);
  EXPECT_LT(common::LogSO3(r.T_target_source.R_WI.transpose()*truth.R_WI).norm(),.002);
}
TEST(ManualGicp, NearSingularPositiveMetricsAndInvalidMetricsAreHandled) {
  auto cloud=loop::prepareManualGicp(scene(),.035,10,1);
  for(auto& C:cloud.covariances)C=1e-12*Eigen::Matrix3d::Identity();
  auto r=loop::alignManualGicp(cloud,cloud,{});EXPECT_TRUE(r.converged)<<r.reason;
  for(auto& C:cloud.covariances)C=Eigen::Vector3d(-1,1,1).asDiagonal();
  r=loop::alignManualGicp(cloud,cloud,{});EXPECT_FALSE(r.converged);EXPECT_EQ(r.reason,"insufficient_correspondences");
}
