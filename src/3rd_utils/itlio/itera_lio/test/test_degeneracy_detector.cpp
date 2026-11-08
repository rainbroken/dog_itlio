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

#include <cmath>

#include "itera_lio/estimator/degeneracy_detector.hpp"

TEST(DegeneracyDetector, DetectsSinglePlaneAsDegenerate)
{
  std::vector<itera_lio::common::PlaneObservation> observations;
  for (int index = 0; index < 10; ++index) {
    itera_lio::common::PlaneObservation observation;
    observation.point_W = Eigen::Vector3d(index * 0.1, 0.0, 0.1);
    observation.centroid_W = Eigen::Vector3d(index * 0.1, 0.0, 0.0);
    observation.normal_W = Eigen::Vector3d(0.0, 0.0, 1.0);
    observation.residual = 0.1;
    observations.push_back(observation);
  }
  const auto report = itera_lio::estimator::detectDegeneracy(
    observations, itera_lio::estimator::NominalState{},
    itera_lio::estimator::FixedExtrinsics{});
  EXPECT_TRUE(report.degenerate);
  EXPECT_TRUE(std::isinf(report.condition_number));
}

TEST(DegeneracyDetector, UsesTheInformationReportedByEskf)
{
  itera_lio::common::ObservationUpdateReport update;
  update.accepted_observations.resize(1U);
  update.pose_information_eigenvalues.setOnes();
  update.pose_information_condition_number = 1.0;
  // The detector must consume the ESKF report rather than rebuild this zero
  // matrix from the accepted observation.
  const auto report = itera_lio::estimator::detectDegeneracy(update);
  EXPECT_FALSE(report.degenerate);
  EXPECT_TRUE(report.eigenvalues.isApprox(Eigen::Matrix<double, 6, 1>::Ones()));
  EXPECT_DOUBLE_EQ(report.condition_number, 1.0);
}
