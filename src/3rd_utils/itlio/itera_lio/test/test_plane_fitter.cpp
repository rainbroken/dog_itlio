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

#include "itera_lio/frontend/plane_fitter.hpp"

TEST(PlaneFitter, FitsHorizontalPlane)
{
  const std::vector<Eigen::Vector3d> points{
    {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {0.0, 1.0, 1.0}, {1.0, 1.0, 1.0}};
  itera_lio::frontend::Plane plane;
  itera_lio::frontend::PlaneFitterOptions options;
  ASSERT_TRUE(itera_lio::frontend::fitPlane(points, options, plane));
  EXPECT_NEAR(std::abs(plane.normal.z()), 1.0, 1e-12);
  EXPECT_NEAR(plane.centroid.z(), 1.0, 1e-12);
  EXPECT_NEAR(plane.max_distance, 0.0, 1e-12);
}

TEST(PlaneFitter, RejectsNonPlanarNeighborhood)
{
  const std::vector<Eigen::Vector3d> points{
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 1.0, 1.0}};
  itera_lio::frontend::Plane plane;
  itera_lio::frontend::PlaneFitterOptions options;
  options.max_point_to_plane_distance = 0.05;
  EXPECT_FALSE(itera_lio::frontend::fitPlane(points, options, plane));
}

TEST(PlaneFitter, RejectsLineWithUndefinedNormal)
{
  const std::vector<Eigen::Vector3d> points{
    {0, 0, 0}, {0.1, 0, 0}, {0.2, 0, 0}, {0.3, 0, 0}, {0.4, 0, 0}};
  itera_lio::frontend::Plane plane;
  itera_lio::frontend::PlaneFitterOptions options;
  EXPECT_FALSE(itera_lio::frontend::fitPlane(points, options, plane));
  options.enable_quality_checks = false;
  EXPECT_TRUE(itera_lio::frontend::fitPlane(points, options, plane));
}

TEST(PlaneFitter, RetainsIndependentEigenvalueBoundForNonDefaultOptions)
{
  const std::vector<Eigen::Vector3d> points{
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, .1}, {0, 0, -.1}};
  itera_lio::frontend::Plane plane;
  itera_lio::frontend::PlaneFitterOptions options;
  options.max_point_to_plane_distance = .2;
  ASSERT_TRUE(itera_lio::frontend::fitPlane(points, options, plane));
  EXPECT_NEAR(plane.eigenvalues.x() / plane.eigenvalues.z(), .01, 1e-12);
  options.max_eigenvalue_ratio = .005;
  EXPECT_FALSE(itera_lio::frontend::fitPlane(points, options, plane));
  options.enable_quality_checks = false;
  EXPECT_FALSE(itera_lio::frontend::fitPlane(points, options, plane));
  options.max_eigenvalue_ratio = .1;
  EXPECT_TRUE(itera_lio::frontend::fitPlane(points, options, plane));
}
