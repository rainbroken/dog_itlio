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

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <random>
#include <chrono>
#include <array>
#include <limits>
#include <stdexcept>
#include <thread>

#include "itera_lio/backend/backend_manager.hpp"
#include "itera_lio/common/so3.hpp"
#include "itera_lio/backend/sparse_assembly_cache.hpp"

namespace
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

itera_lio::common::Pose poseAt(const double x, const double yaw = 0.0)
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  rotation(0, 0) = std::cos(yaw);
  rotation(0, 1) = -std::sin(yaw);
  rotation(1, 0) = std::sin(yaw);
  rotation(1, 1) = std::cos(yaw);
  return {rotation, Eigen::Vector3d(x, 0.0, 0.0)};
}

itera_lio::common::Pose poseWithRotationVector(
  const Eigen::Vector3d & rotation_vector, const Eigen::Vector3d & position)
{
  return {itera_lio::common::ExpSO3(rotation_vector), position};
}

itera_lio::common::Pose applyLeftPerturbation(
  const itera_lio::common::Pose & pose, const Vector6d & delta)
{
  return {
    itera_lio::common::ExpSO3(delta.head<3>()) * pose.R_WI,
    pose.p_WI + delta.tail<3>()};
}

Vector6d constraintResidual(
  const itera_lio::backend::PoseGraphConstraint & constraint,
  const itera_lio::common::Pose & from, const itera_lio::common::Pose & to)
{
  const auto predicted = itera_lio::backend::relativePose(from, to);
  const auto error = itera_lio::backend::composePoses(
    itera_lio::backend::inversePose(constraint.T_Ia_Ib), predicted);
  Vector6d residual;
  residual.head<3>() = itera_lio::common::LogSO3(error.R_WI);
  residual.tail<3>() = error.p_WI;
  return residual;
}

Matrix6d centralDifferenceJacobian(
  const itera_lio::backend::PoseGraphConstraint & constraint,
  const itera_lio::common::Pose & from, const itera_lio::common::Pose & to,
  const bool perturb_from)
{
  constexpr double kStep = 1e-7;
  Matrix6d jacobian;
  for (int column = 0; column < 6; ++column) {
    Vector6d delta = Vector6d::Zero();
    delta[column] = kStep;
    const auto from_plus = perturb_from ? applyLeftPerturbation(from, delta) : from;
    const auto from_minus = perturb_from ? applyLeftPerturbation(from, -delta) : from;
    const auto to_plus = perturb_from ? to : applyLeftPerturbation(to, delta);
    const auto to_minus = perturb_from ? to : applyLeftPerturbation(to, -delta);
    jacobian.col(column) =
      (constraintResidual(constraint, from_plus, to_plus) -
      constraintResidual(constraint, from_minus, to_minus)) / (2.0 * kStep);
  }
  return jacobian;
}

}  // namespace

TEST(StreamingSparseAssembly, PreservesDuplicateOrderAndRebuildsOnAnyStructureChange)
{
  using Triplet=Eigen::Triplet<double>;
  Eigen::SparseMatrix<double> actual(4,4), reference(4,4);
  itera_lio::backend::detail::StreamingSparseAssembly stream;
  itera_lio::backend::detail::SparseAssemblyCache original;
  const std::vector<std::vector<Triplet>> batches{
    {{0,0,2},{1,0,3},{1,0,4},{3,3,5}},
    {{0,0,7},{1,0,9},{1,0,-2},{3,3,11}},
    {{0,0,3},{2,1,8},{2,1,1},{3,3,6}},
    {{3,3,2},{2,1,8},{2,1,-3},{0,0,1}},
    {{3,3,2},{2,1,8}},
    {},
    {{0,0,9},{1,2,7},{1,2,-2}}};
  for(const auto& batch:batches){
    stream.begin(actual);
    for(const auto& t:batch)stream.emplace_back(t.row(),t.col(),t.value());
    stream.finish();original.assemble(reference,batch);
    EXPECT_EQ(actual.nonZeros(),reference.nonZeros());
    EXPECT_TRUE(Eigen::MatrixXd(actual)==Eigen::MatrixXd(reference));
  }
  actual.resize(6,6);reference.resize(6,6);
  std::vector<Triplet> resized{{5,5,2},{0,5,3}};
  stream.begin(actual);for(const auto& t:resized)stream.emplace_back(t.row(),t.col(),t.value());
  stream.finish();reference.setFromTriplets(resized.begin(),resized.end());
  EXPECT_TRUE(Eigen::MatrixXd(actual)==Eigen::MatrixXd(reference));
}

TEST(StreamingSparseAssembly, FixedPatternRetainsOrderedSumsAndRejectsStructuralMisuse)
{
  Eigen::SparseMatrix<double> matrix(2,2);
  itera_lio::backend::detail::StreamingSparseAssembly stream;
  for(int i=1;i<=4;++i){
    stream.begin(matrix,true,true);
    stream.emplace_back(0,0,i);stream.emplace_back(0,0,2*i);stream.emplace_back(1,1,3*i);
    stream.finish();EXPECT_EQ(matrix.coeff(0,0),3*i);EXPECT_EQ(matrix.coeff(1,1),3*i);
  }
  stream.begin(matrix,true,true);
  EXPECT_THROW(stream.emplace_back(0,1,1.),std::logic_error);
  stream.begin(matrix,true,true);stream.emplace_back(0,0,1.);
  EXPECT_THROW(stream.finish(),std::logic_error);
}

TEST(StreamingSparseAssembly, LowerOnlyMatchesOriginalFactorizationAndOrderedUpdates)
{
  using Matrix=Eigen::SparseMatrix<double>;
  Matrix lower(3,3),full(3,3);
  itera_lio::backend::detail::StreamingSparseAssembly stream;
  for(int rep=0;rep<4;++rep){
    std::vector<Eigen::Triplet<double>> entries{
      {0,0,5.+rep},{1,1,6.+rep},{2,2,7.+rep},
      {0,1,1.},{1,0,1.},{1,0,.25},{0,1,.25},
      {1,2,-.5},{2,1,-.5},{0,2,.1},{2,0,.1}};
    stream.begin(lower,true,true,true);
    for(const auto& t:entries)stream.emplace_back(t.row(),t.col(),t.value());
    stream.finish();full.setFromTriplets(entries.begin(),entries.end());
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)
      EXPECT_EQ(full.coeff(i,j),lower.coeff(std::max(i,j),std::min(i,j)));
    Eigen::SimplicialLDLT<Matrix> a,b;a.compute(full);b.compute(lower);
    ASSERT_EQ(a.info(),Eigen::Success);ASSERT_EQ(b.info(),Eigen::Success);
    const Eigen::Vector3d rhs(1.,2.,3.);
    EXPECT_TRUE(Eigen::Vector3d(a.solve(rhs))==Eigen::Vector3d(b.solve(rhs)));
  }
}

TEST(PoseGraphOptimizer, AnalyticJacobiansMatchCentralDifferences)
{
  std::mt19937 generator(42U);
  std::uniform_real_distribution<double> rotation(-0.5, 0.5);
  std::uniform_real_distribution<double> translation(-3.0, 3.0);
  for (int sample = 0; sample < 20; ++sample) {
    const auto random_vector = [&](auto & distribution) {
        return Eigen::Vector3d(
          distribution(generator), distribution(generator), distribution(generator));
      };
    const auto from = poseWithRotationVector(random_vector(rotation), random_vector(translation));
    const auto to = poseWithRotationVector(random_vector(rotation), random_vector(translation));
    itera_lio::backend::PoseGraphConstraint constraint;
    constraint.from_id = 0U;
    constraint.to_id = 1U;
    constraint.T_Ia_Ib = poseWithRotationVector(
      random_vector(rotation), random_vector(translation));

    const auto analytic = itera_lio::backend::linearizeConstraint(constraint, from, to);
    const Matrix6d numerical_from = centralDifferenceJacobian(constraint, from, to, true);
    const Matrix6d numerical_to = centralDifferenceJacobian(constraint, from, to, false);

    EXPECT_TRUE(analytic.residual.isApprox(constraintResidual(constraint, from, to), 1e-12));
    EXPECT_TRUE(analytic.from_jacobian.isApprox(numerical_from, 1e-6)) <<
      "sample " << sample << " analytic from:\n" << analytic.from_jacobian <<
      "\nnumerical from:\n" << numerical_from;
    EXPECT_TRUE(analytic.to_jacobian.isApprox(numerical_to, 1e-6)) <<
      "sample " << sample << " analytic to:\n" << analytic.to_jacobian <<
      "\nnumerical to:\n" << numerical_to;
  }
}

TEST(PoseGraph, InverseInformationMatchesMixedChartNumericalJacobian)
{
  using namespace itera_lio::backend;
  const auto measured = poseWithRotationVector({0.2, -0.1, 0.3}, {2.0, -3.0, 1.0});
  Matrix6d forward_information = Matrix6d::Identity();
  forward_information.diagonal() << 2, 3, 4, 5, 6, 7;
  const Matrix6d inverse_information = inverseMeasurementInformation(measured, forward_information);
  Matrix6d numerical;
  const auto residual = [&](const Vector6d &delta) {
    const auto error = composePoses(applyLeftPerturbation(measured, delta), inversePose(measured));
    Vector6d result;
    result << itera_lio::common::LogSO3(error.R_WI), error.p_WI;
    return result;
  };
  for (int i = 0; i < 6; ++i) {
    Vector6d delta = Vector6d::Zero();
    delta[i] = 1e-7;
    numerical.col(i) = (residual(delta) - residual(-delta)) / 2e-7;
  }
  EXPECT_TRUE((numerical.transpose() * inverse_information * numerical).isApprox(
      forward_information, 1e-7));
  EXPECT_GT((inverse_information.topRightCorner<3, 3>().norm()), 1.0);
}

TEST(BackendManager, QualityWeightsRetainWeakDirectionsWithoutCountingScansAsIndependent)
{
  using namespace itera_lio::backend;
  BackendOptions options;
  options.enabled = true;
  options.keyframes.min_translation = 0.5;
  BackendManager backend(options);
  backend.addOdometryKeyframe(0.0, poseAt(0.0));
  Matrix6d observed = 1e8 * Matrix6d::Identity();
  observed(4, 4) = 0.0;
  for (int i = 0; i < 10; ++i) {backend.observeOdometryQuality(poseAt(0.0), observed);}
  backend.addOdometryKeyframe(1.0, poseAt(1.0));
  const auto first = backend.graph().constraints().back().information;
  EXPECT_NEAR(first(4, 4), 1.0, 1e-6);
  EXPECT_GT(first(3, 3), 99.0);
  EXPECT_LE(first(3, 3), 100.0);
  backend.observeOdometryQuality(poseAt(0.0), observed);
  backend.addOdometryKeyframe(2.0, poseAt(2.0));
  EXPECT_TRUE(first.isApprox(backend.graph().constraints().back().information, 1e-12));
  backend.observeOdometryQuality(poseAt(0.0), Matrix6d::Zero());
  backend.addOdometryKeyframe(3.0, poseAt(3.0));
  EXPECT_NEAR(backend.graph().constraints().back().information(3, 3), 1.0, 1e-6);
}

TEST(BackendManager, MapCorrectionComposesWithOdometryAndPropagatesToNewKeyframes)
{
  using namespace itera_lio::backend;
  BackendOptions options;
  options.enabled = true;
  BackendManager backend(options);
  EXPECT_TRUE(backend.mapToOdometry().p_WI.isZero());
  backend.addOdometryKeyframe(0.0, poseAt(0.0));
  backend.addOdometryKeyframe(1.0, poseAt(2.0));
  ASSERT_TRUE(backend.addLoopConstraint(0, 1,
      poseWithRotationVector({0.05, -0.04, 0.1}, {1.5, 0.2, 0.1}),
      1e4 * PoseInformationMatrix::Identity()));
  const auto correction = backend.mapToOdometry();
  const auto corrected = composePoses(correction, backend.graph().nodes().back().pose_WI);
  EXPECT_TRUE(corrected.R_WI.isApprox(backend.optimizedKeyframes().back().pose_WI.R_WI, 1e-10));
  EXPECT_TRUE(corrected.p_WI.isApprox(backend.optimizedKeyframes().back().pose_WI.p_WI, 1e-10));
  backend.addOdometryKeyframe(2.0, poseAt(3.0));
  EXPECT_TRUE(backend.mapToOdometry().R_WI.isApprox(correction.R_WI, 1e-10));
  EXPECT_TRUE(backend.mapToOdometry().p_WI.isApprox(correction.p_WI, 1e-10));
}

TEST(KeyframeManager, UsesTranslationRotationAndTimeThresholds)
{
  itera_lio::backend::KeyframeManagerOptions options;
  options.min_translation = 1.0;
  options.min_rotation_rad = 0.2;
  options.max_interval_sec = 5.0;
  itera_lio::backend::KeyframeManager manager(options);

  const auto first = manager.addIfNeeded(0.0, poseAt(0.0));
  ASSERT_TRUE(first);
  EXPECT_EQ(first->id, 0U);
  EXPECT_FALSE(manager.addIfNeeded(1.0, poseAt(0.5)));
  const auto rotated = manager.addIfNeeded(2.0, poseAt(0.5, 0.25));
  ASSERT_TRUE(rotated);
  EXPECT_EQ(rotated->id, 1U);
  const auto timed = manager.addIfNeeded(7.0, poseAt(0.5, 0.25));
  ASSERT_TRUE(timed);
  EXPECT_EQ(timed->id, 2U);
}

TEST(BackendManager, KeepsOdometryGraphUnoptimizedUntilLoopConstraintExists)
{
  itera_lio::backend::BackendOptions options;
  options.keyframes.min_translation = 1.0;
  options.keyframes.min_rotation_rad = 0.1;
  options.keyframes.max_interval_sec = 20.0;
  options.enabled = true;
  itera_lio::backend::BackendManager backend(options);

  const auto first = backend.addOdometryKeyframe(0.0, poseAt(0.0));
  ASSERT_TRUE(first.keyframe_added);
  ASSERT_TRUE(first.keyframe);
  EXPECT_EQ(first.keyframe->id, 0U);
  EXPECT_FALSE(backend.addOdometryKeyframe(1.0, poseAt(0.2)).keyframe_added);
  const auto second = backend.addOdometryKeyframe(2.0, poseAt(1.1));
  EXPECT_TRUE(second.keyframe_added);
  EXPECT_EQ(second.keyframe_count, 2U);
  EXPECT_EQ(second.odometry_constraint_count, 1U);
  EXPECT_EQ(second.loop_constraint_count, 0U);
  EXPECT_FALSE(second.optimization.optimized);
  ASSERT_EQ(backend.optimizedKeyframes().size(), 2U);
  EXPECT_NEAR(backend.optimizedKeyframes().back().pose_WI.p_WI.x(), 1.1, 1e-12);
}

TEST(BackendManager, IsDisabledByDefault)
{
  itera_lio::backend::BackendManager backend;
  const auto update = backend.addOdometryKeyframe(0.0, poseAt(0.0));
  EXPECT_FALSE(update.keyframe_added);
  EXPECT_EQ(update.keyframe_count, 0U);
  EXPECT_FALSE(update.keyframe);
}



TEST(PoseGraphOptimizer, FullPoseOptimizationCorrectsRollPitchAndYaw)
{
  using itera_lio::backend::ConstraintType;
  using itera_lio::backend::PoseGraphConstraint;
  using itera_lio::backend::PoseInformationMatrix;
  itera_lio::backend::PoseGraphOptimizerOptions options;
  options.max_iterations = 30;
  itera_lio::backend::PoseGraphOptimizer optimizer(options);
  std::vector<itera_lio::backend::Keyframe> raw{
    {0U, 0.0, poseWithRotationVector({0.08, -0.05, 0.0}, {0.0, 0.0, 0.0})},
    {1U, 1.0, poseWithRotationVector({0.06, -0.07, 0.0}, {1.0, 0.0, 0.1})},
    {2U, 2.0, poseWithRotationVector({0.04, -0.04, 0.0}, {2.0, 0.0, 0.2})}};
  const auto loop_measurement = poseWithRotationVector(
    {0.3, -0.2, 0.25}, {2.0, 0.0, 0.2});
  const std::vector<PoseGraphConstraint> constraints{
    {0U, 1U, itera_lio::backend::relativePose(raw[0].pose_WI, raw[1].pose_WI),
      100.0 * PoseInformationMatrix::Identity(), ConstraintType::kOdometry},
    {1U, 2U, itera_lio::backend::relativePose(raw[1].pose_WI, raw[2].pose_WI),
      100.0 * PoseInformationMatrix::Identity(), ConstraintType::kOdometry},
    {0U, 2U, loop_measurement,
      100.0 * PoseInformationMatrix::Identity(), ConstraintType::kLoop}};
  auto optimized = raw;
  itera_lio::backend::PoseGraphOptimizationReport report;
  ASSERT_TRUE(optimizer.optimize(optimized, constraints, report));
  EXPECT_TRUE(report.optimized);
  EXPECT_TRUE(optimized[0].pose_WI.R_WI.isApprox(raw[0].pose_WI.R_WI, 1e-12));
  EXPECT_TRUE(optimized[0].pose_WI.p_WI.isApprox(raw[0].pose_WI.p_WI, 1e-12));
  const Eigen::Vector3d correction = itera_lio::common::LogSO3(
    optimized[2].pose_WI.R_WI * raw[2].pose_WI.R_WI.transpose());
  EXPECT_GT(std::abs(correction.x()), 0.01);
  EXPECT_GT(std::abs(correction.y()), 0.01);
  EXPECT_GT(std::abs(correction.z()), 0.01);
  EXPECT_LT(report.final_cost, report.initial_cost);
}

TEST(PoseGraphOptimizer, LoopCorrectsTiltAndPreservesHeightWithoutExtraPrior)
{
  using namespace itera_lio::backend;
  const std::vector<Keyframe> raw{
    {0U, 0.0, poseAt(0.0)},
    {1U, 1.0, poseWithRotationVector({0.10, 0.0, 0.0}, {2.0, 0.0, 3.0})}};
  const auto target = poseWithRotationVector({0.0, 0.0, 0.0}, {2.0, 0.0, 3.0});
  const std::vector<PoseGraphConstraint> constraints{
    {0U, 1U, raw[1].pose_WI, 100.0 * PoseInformationMatrix::Identity(),
      ConstraintType::kOdometry},
    {0U, 1U, target, 10000.0 * PoseInformationMatrix::Identity(), ConstraintType::kLoop}};
  PoseGraphOptimizerOptions options;
  options.max_iterations = 30;
  auto nodes = raw;
  PoseGraphOptimizationReport report;
  ASSERT_TRUE(PoseGraphOptimizer(options).optimize(nodes, constraints, report));
  const double tilt = itera_lio::common::LogSO3(nodes[1].pose_WI.R_WI).norm();
  EXPECT_LT(tilt, 0.02);
  EXPECT_NEAR(nodes[1].pose_WI.p_WI.z(), 3.0, 1e-8);
  EXPECT_LT(report.final_cost, report.initial_cost);
}

TEST(PoseGraphOptimizer, PreservesAscentDescentAndDevicePitchWithConsistentEdges)
{
  using namespace itera_lio::backend;
  std::vector<Keyframe> raw;
  std::vector<PoseGraphConstraint> constraints;
  // Real ascent followed by a landing and descent. Device pitch is unrelated
  // to whether the terrain is horizontal and must remain unchanged here.
  const std::array<double, 6> heights{0.0, 0.2, 0.4, 3.0, 3.0, 0.0};
  for (std::size_t i = 0U; i < heights.size(); ++i) {
    raw.push_back({i, double(i), poseWithRotationVector(
        {0.0, 0.15 * double(i), 0.05 * double(i)}, {double(i), 0.0, heights[i]})});
    if (i > 0U) {
      constraints.push_back({i - 1U, i, relativePose(raw[i - 1U].pose_WI, raw[i].pose_WI),
          PoseInformationMatrix::Identity(), ConstraintType::kOdometry});
    }
  }
  PoseGraphOptimizerOptions options;
  auto nodes = raw;
  PoseGraphOptimizationReport report;
  ASSERT_TRUE(PoseGraphOptimizer(options).optimize(nodes, constraints, report));
  for (std::size_t i = 0U; i < raw.size(); ++i) {
    EXPECT_TRUE(nodes[i].pose_WI.p_WI.isApprox(raw[i].pose_WI.p_WI, 1e-9));
    EXPECT_TRUE(nodes[i].pose_WI.R_WI.isApprox(raw[i].pose_WI.R_WI, 1e-9));
  }
}

TEST(PoseGraphOptimizer, RejectsInvalidSolverSettings)
{
  itera_lio::backend::PoseGraphOptimizerOptions options;
  options.damping = -0.05;
  EXPECT_THROW(itera_lio::backend::PoseGraphOptimizer{options}, std::invalid_argument);
  options.damping = 0.0;
  options.huber_delta = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(itera_lio::backend::PoseGraphOptimizer{options}, std::invalid_argument);
}




TEST(BackendManager, RejectsDuplicateLoopEdgeWithoutChangingGraph)
{
  itera_lio::backend::BackendOptions options;
  options.enabled = true;
  options.keyframes.min_translation = 0.1;
  itera_lio::backend::BackendManager backend(options);
  ASSERT_TRUE(backend.addOdometryKeyframe(0.0, poseAt(0.0)).keyframe_added);
  ASSERT_TRUE(backend.addOdometryKeyframe(1.0, poseAt(1.0)).keyframe_added);
  const auto first = backend.addLoopConstraint(
    0U, 1U, poseAt(1.0), itera_lio::backend::PoseInformationMatrix::Identity());
  ASSERT_TRUE(first);
  EXPECT_TRUE(first->optimization.optimized);
  EXPECT_FALSE(
    backend.addLoopConstraint(
      1U, 0U, poseAt(-1.0), itera_lio::backend::PoseInformationMatrix::Identity()));
  EXPECT_EQ(backend.graph().loopConstraintCount(), 1U);
}

TEST(BackendManager, LoopConstraintReducesPoseGraphCost)
{
  itera_lio::backend::BackendOptions options;
  options.keyframes.min_translation = 0.1;
  options.keyframes.min_rotation_rad = 0.01;
  options.keyframes.max_interval_sec = 20.0;
  options.odometry_rotation_stddev = 10.0;
  options.odometry_translation_stddev = 10.0;
  options.optimizer.max_iterations = 20;
  options.enabled = true;
  itera_lio::backend::BackendManager backend(options);

  ASSERT_TRUE(backend.addOdometryKeyframe(0.0, poseAt(0.0)).keyframe_added);
  ASSERT_TRUE(backend.addOdometryKeyframe(1.0, poseAt(1.0)).keyframe_added);
  ASSERT_TRUE(backend.addOdometryKeyframe(2.0, poseAt(2.4)).keyframe_added);

  itera_lio::backend::PoseInformationMatrix information =
    itera_lio::backend::PoseInformationMatrix::Identity() * 10.0;
  const auto update = backend.addLoopConstraint(0U, 2U, poseAt(2.0), information);
  ASSERT_TRUE(update);
  EXPECT_TRUE(update->loop_constraint_added);
  EXPECT_TRUE(update->optimization.optimized);
  EXPECT_LT(update->optimization.final_cost, update->optimization.initial_cost);
  ASSERT_EQ(backend.optimizedKeyframes().size(), 3U);
  EXPECT_NEAR(backend.optimizedKeyframes().front().pose_WI.p_WI.x(), 0.0, 1e-12);
  EXPECT_NEAR(backend.optimizedKeyframes().back().pose_WI.p_WI.x(), 2.0, 0.03);
  const auto next = backend.addOdometryKeyframe(3.0, poseAt(3.4));
  ASSERT_TRUE(next.keyframe_added);
  ASSERT_EQ(backend.optimizedKeyframes().size(), 4U);
  EXPECT_NEAR(
    backend.optimizedKeyframes().back().pose_WI.p_WI.x() -
    backend.optimizedKeyframes()[2].pose_WI.p_WI.x(), 1.0, 1e-6);
}

TEST(BackendManager, LoopConstraintCorrectsThreeDimensionalPose)
{
  itera_lio::backend::BackendOptions options;
  options.keyframes.min_translation = 0.1;
  options.keyframes.min_rotation_rad = 0.01;
  options.odometry_rotation_stddev = 10.0;
  options.odometry_translation_stddev = 10.0;
  options.optimizer.max_iterations = 20;
  options.enabled = true;
  itera_lio::backend::BackendManager backend(options);

  const auto initial_pose = poseWithRotationVector(
    Eigen::Vector3d(0.15, -0.10, 0.30), Eigen::Vector3d(1.2, 0.2, -0.1));
  const auto measured_loop = poseWithRotationVector(
    Eigen::Vector3d(0.10, -0.05, 0.20), Eigen::Vector3d(1.0, 0.0, 0.0));
  ASSERT_TRUE(backend.addOdometryKeyframe(0.0, {}).keyframe_added);
  ASSERT_TRUE(backend.addOdometryKeyframe(1.0, initial_pose).keyframe_added);

  const auto update = backend.addLoopConstraint(
    0U, 1U, measured_loop,
    100.0 * itera_lio::backend::PoseInformationMatrix::Identity());
  ASSERT_TRUE(update);
  ASSERT_TRUE(update->optimization.optimized);
  const auto error = itera_lio::backend::composePoses(
    itera_lio::backend::inversePose(measured_loop), backend.optimizedKeyframes().back().pose_WI);
  EXPECT_LT(itera_lio::common::LogSO3(error.R_WI).norm(), 0.01);
  EXPECT_LT(error.p_WI.norm(), 0.01);
}

TEST(PoseGraphOptimizer, HandlesProductionScaleSparseChain)
{
  constexpr std::size_t kNodeCount = 1200U;
  constexpr double kOdometryStep = 1.001;
  itera_lio::backend::PoseGraph graph;
  const auto odometry_information =
    itera_lio::backend::PoseInformationMatrix::Identity();
  for (std::size_t id = 0U; id < kNodeCount; ++id) {
    ASSERT_TRUE(
      graph.addNode(
        {id, static_cast<double>(id), poseAt(static_cast<double>(id) * kOdometryStep)}));
    if (id > 0U) {
      ASSERT_TRUE(
        graph.addConstraint(
          {id - 1U, id, poseAt(kOdometryStep), odometry_information,
            itera_lio::backend::ConstraintType::kOdometry}));
    }
  }
  ASSERT_TRUE(
    graph.addConstraint(
      {0U, kNodeCount - 1U, poseAt(static_cast<double>(kNodeCount - 1U)),
        100.0 * itera_lio::backend::PoseInformationMatrix::Identity(),
        itera_lio::backend::ConstraintType::kLoop}));

  auto nodes = graph.nodes();
  itera_lio::backend::PoseGraphOptimizerOptions options;
  options.max_iterations = 5;
  itera_lio::backend::PoseGraphOptimizer optimizer(options);
  itera_lio::backend::PoseGraphOptimizationReport report;
  ASSERT_TRUE(optimizer.optimize(nodes, graph.constraints(), report));
  EXPECT_TRUE(report.optimized);
  EXPECT_LT(report.final_cost, report.initial_cost);
  EXPECT_NEAR(nodes.front().pose_WI.p_WI.x(), 0.0, 1e-12);
  EXPECT_TRUE(nodes.back().pose_WI.isValid());
  EXPECT_EQ(report.symbolic_factorizations, 1U);

  auto repeated_nodes = graph.nodes();
  itera_lio::backend::PoseGraphOptimizationReport repeated_report;
  ASSERT_TRUE(optimizer.optimize(repeated_nodes, graph.constraints(), repeated_report));
  EXPECT_EQ(repeated_report.symbolic_factorizations, 0U);
}

TEST(BackendManager, LimitsOptimizationToConfiguredActiveWindow)
{
  itera_lio::backend::BackendOptions options;
  options.enabled = true;
  options.keyframes.min_translation = 0.1;
  options.odometry_rotation_stddev = 10.0;
  options.odometry_translation_stddev = 10.0;
  options.optimizer.active_window_size = 4U;
  itera_lio::backend::BackendManager backend(options);
  for (std::size_t id = 0U; id < 10U; ++id) {
    ASSERT_TRUE(
      backend.addOdometryKeyframe(
        static_cast<double>(id), poseAt(static_cast<double>(id) * 1.01)).keyframe_added);
  }
  const auto update = backend.addLoopConstraint(
    7U, 9U, poseAt(2.0),
    10.0 * itera_lio::backend::PoseInformationMatrix::Identity());
  ASSERT_TRUE(update);
  EXPECT_TRUE(update->optimization.optimized);
  EXPECT_EQ(update->optimization.active_node_count, 5U);
  EXPECT_EQ(update->optimization.symbolic_factorizations, 1U);
  EXPECT_EQ(backend.optimizedKeyframes().size(), 10U);
}

TEST(BackendManager, ExpandsActiveWindowToKeepOldLoopChainConnected)
{
  itera_lio::backend::BackendOptions options;
  options.enabled = true;
  options.keyframes.min_translation = 0.1;
  options.odometry_rotation_stddev = 10.0;
  options.odometry_translation_stddev = 10.0;
  options.optimizer.active_window_size = 4U;
  options.optimizer.max_iterations = 20;
  itera_lio::backend::BackendManager backend(options);
  for (std::size_t id = 0U; id < 10U; ++id) {
    ASSERT_TRUE(
      backend.addOdometryKeyframe(
        static_cast<double>(id), poseAt(static_cast<double>(id) * 1.01)).keyframe_added);
  }

  const auto update = backend.addLoopConstraint(
    0U, 9U, poseAt(9.0),
    10.0 * itera_lio::backend::PoseInformationMatrix::Identity());
  ASSERT_TRUE(update);
  EXPECT_TRUE(update->optimization.optimized);
  EXPECT_EQ(update->optimization.active_node_count, 10U);
  const auto optimized = backend.optimizedKeyframes();
  ASSERT_EQ(optimized.size(), 10U);
  EXPECT_NEAR(optimized.front().pose_WI.p_WI.x(), 0.0, 1e-12);
  for (std::size_t id = 1U; id < optimized.size(); ++id) {
    const double step = optimized[id].pose_WI.p_WI.x() -
      optimized[id - 1U].pose_WI.p_WI.x();
    EXPECT_GT(step, 0.95) << "id=" << id;
    EXPECT_LT(step, 1.05) << "id=" << id;
  }
}

TEST(BackendManager, ConvergesLargeDriftLoopWithoutBreakingOdometryChain)
{
  constexpr std::size_t kNodeCount = 513U;
  constexpr double kDriftedStep = 1.07;
  itera_lio::backend::BackendOptions options;
  options.enabled = true;
  options.keyframes.min_translation = 0.1;
  options.odometry_rotation_stddev = 0.05;
  options.odometry_translation_stddev = 0.10;
  options.optimizer.active_window_size = 500U;
  options.optimizer.max_iterations = 20;
  itera_lio::backend::BackendManager backend(options);
  for (std::size_t id = 0U; id < kNodeCount; ++id) {
    ASSERT_TRUE(
      backend.addOdometryKeyframe(
        static_cast<double>(id), poseAt(static_cast<double>(id) * kDriftedStep))
      .keyframe_added);
  }

  itera_lio::backend::PoseInformationMatrix information =
    itera_lio::backend::PoseInformationMatrix::Zero();
  information.diagonal().head<3>().setConstant(20.0);
  information.diagonal().tail<3>().setConstant(5.0);
  const auto update = backend.addLoopConstraint(
    kNodeCount - 1U, 0U, poseAt(-static_cast<double>(kNodeCount - 1U)),
    information);
  ASSERT_TRUE(update);
  EXPECT_LT(update->optimization.final_cost, update->optimization.initial_cost);
  EXPECT_EQ(update->optimization.active_node_count, kNodeCount);
  const auto optimized = backend.optimizedKeyframes();
  ASSERT_EQ(optimized.size(), kNodeCount);
  for (std::size_t id = 1U; id < optimized.size(); ++id) {
    const double step = (
      optimized[id].pose_WI.p_WI - optimized[id - 1U].pose_WI.p_WI).norm();
    EXPECT_LT(step, 1.20) << "id=" << id;
  }
}

TEST(BackendManager, RunsPoseGraphOptimizationOnWorkerAndReportsQueueStats)
{
  itera_lio::backend::BackendOptions options;
  options.enabled = true;
  options.asynchronous_optimization = true;
  options.keyframes.min_translation = 0.1;
  options.odometry_rotation_stddev = 10.0;
  options.odometry_translation_stddev = 10.0;
  options.worker_queue_capacity = 2U;
  options.task_timeout_ms = 1000.0;
  itera_lio::backend::BackendManager backend(options);
  ASSERT_TRUE(backend.addOdometryKeyframe(0.0, poseAt(0.0)).keyframe_added);
  ASSERT_TRUE(backend.addOdometryKeyframe(1.0, poseAt(1.1)).keyframe_added);
  ASSERT_TRUE(
    backend.enqueueLoopConstraint(
      0U, 1U, poseAt(1.0),
      10.0 * itera_lio::backend::PoseInformationMatrix::Identity()));

  std::vector<itera_lio::backend::BackendUpdate> completed;
  for (int attempt = 0; attempt < 100 && completed.empty(); ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    completed = backend.pollCompleted();
  }
  ASSERT_EQ(completed.size(), 1U);
  EXPECT_TRUE(completed.front().optimization.optimized);
  const auto stats = backend.workerStats();
  EXPECT_EQ(stats.submitted, 1U);
  EXPECT_EQ(stats.completed, 1U);
  EXPECT_EQ(stats.queue_depth, 0U);
}

TEST(BackendManager, ExpiresWorkerTaskAtDeadline)
{
  itera_lio::backend::BackendOptions options;
  options.enabled = true;
  options.asynchronous_optimization = true;
  options.keyframes.min_translation = 0.1;
  options.task_timeout_ms = 1e-9;
  itera_lio::backend::BackendManager backend(options);
  ASSERT_TRUE(backend.addOdometryKeyframe(0.0, poseAt(0.0)).keyframe_added);
  ASSERT_TRUE(backend.addOdometryKeyframe(1.0, poseAt(1.0)).keyframe_added);
  ASSERT_TRUE(
    backend.enqueueLoopConstraint(
      0U, 1U, poseAt(1.0), itera_lio::backend::PoseInformationMatrix::Identity()));
  for (int attempt = 0; attempt < 100 && backend.workerStats().timed_out == 0U; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(backend.workerStats().timed_out, 1U);
  EXPECT_TRUE(backend.pollCompleted().empty());
}
