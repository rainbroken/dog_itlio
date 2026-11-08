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

#include <algorithm>
#include <limits>
#include <vector>

#include "itera_lio/frontend/global_map.hpp"

TEST(GlobalMap, RejectsInvalidVoxelSize)
{
  EXPECT_THROW(itera_lio::frontend::GlobalMap(0.0F), std::invalid_argument);
}

TEST(GlobalMap, RetainsOneRepresentativePerVoxel)
{
  itera_lio::frontend::GlobalMap map(1.0F);
  map.build({{{0.1F, 0.1F, 0.1F}, 1.0F}, {{0.4F, 0.4F, 0.4F}, 2.0F}});
  EXPECT_EQ(map.size(), 1U);
  EXPECT_EQ(map.addPoints({{{1.1F, 0.1F, 0.1F}, 3.0F}}), 1);

  std::vector<itera_lio::frontend::MapPoint> points;
  ASSERT_TRUE(map.getAllPoints(points));
  ASSERT_EQ(points.size(), 2U);
  const auto representative = std::find_if(
    points.begin(), points.end(), [](const auto & point) {return point.position.x() < 1.0F;});
  ASSERT_NE(representative, points.end());
  EXPECT_FLOAT_EQ(representative->intensity, 2.0F);
}

TEST(GlobalMap, ReportsArchiveRepresentativesInsertedAndReplaced)
{
  itera_lio::frontend::GlobalMap map(1.0F);
  itera_lio::frontend::GlobalMapUpdate update;
  EXPECT_EQ(map.addPoints({{{0.1F, 0.1F, 0.1F}, 1.0F}}, &update), 1);
  EXPECT_EQ(update.inserted_count, 1U);
  EXPECT_EQ(update.replaced_count, 0U);
  ASSERT_EQ(update.upserted_points.size(), 1U);
  EXPECT_FLOAT_EQ(update.upserted_points.front().intensity, 1.0F);

  update = {};
  EXPECT_EQ(map.addPoints({{{0.49F, 0.49F, 0.49F}, 2.0F}}, &update), 0);
  EXPECT_EQ(update.inserted_count, 0U);
  EXPECT_EQ(update.replaced_count, 1U);
  ASSERT_EQ(update.upserted_points.size(), 1U);
  EXPECT_FLOAT_EQ(update.upserted_points.front().intensity, 2.0F);
}

TEST(GlobalMap, IgnoresNonFinitePoints)
{
  itera_lio::frontend::GlobalMap map(1.0F);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(map.addPoints({{{nan, 0.0F, 0.0F}, 1.0F}}), 0);
  EXPECT_EQ(map.size(), 0U);
}

TEST(GlobalMap, IgnoresVoxelCoordinatesOutsideCompactKeyRange)
{
  itera_lio::frontend::GlobalMap map(0.1F);
  const float outside_key_range = 3.0e8F;
  EXPECT_EQ(map.addPoints({{{outside_key_range, 0.0F, 0.0F}, 1.0F}}), 0);
  EXPECT_EQ(map.size(), 0U);
}

TEST(GlobalMap, VisitsRepresentativesWithoutChangingSize)
{
  itera_lio::frontend::GlobalMap map(1.0F);
  map.addPoints({{{0.1F, 0.1F, 0.1F}, 1.0F}, {{1.1F, 0.1F, 0.1F}, 2.0F}});
  std::size_t visited = 0U;
  float intensity_sum = 0.0F;
  EXPECT_TRUE(
    map.forEachPoint(
      [&](const auto & point) {
        ++visited;
        intensity_sum += point.intensity;
      }));
  EXPECT_EQ(visited, map.size());
  EXPECT_FLOAT_EQ(intensity_sum, 3.0F);
  EXPECT_FALSE(map.forEachPoint({}));
}

TEST(GlobalMap, PartitionsPositiveAndNegativeVoxelBoundariesIntoChunks)
{
  itera_lio::frontend::GlobalMap map(1.0F);
  EXPECT_EQ(
    map.addPoints(
  {
    {{0.1F, 0.1F, 0.1F}, 1.0F},
    {{63.1F, 0.1F, 0.1F}, 2.0F},
    {{64.1F, 0.1F, 0.1F}, 3.0F},
    {{-0.1F, 0.1F, 0.1F}, 4.0F},
    {{-64.1F, 0.1F, 0.1F}, 5.0F}}),
    5);
  EXPECT_EQ(map.size(), 5U);
  const auto stats = map.stats();
  EXPECT_EQ(stats.chunk_count, 4U);
  EXPECT_EQ(stats.max_chunk_points, 2U);
  EXPECT_GT(stats.rehash_count, 0U);
}

TEST(GlobalMap, MarginalizesOldestChunksAtResidentLimit)
{
  itera_lio::frontend::GlobalMap map(1.0F, 2U);
  EXPECT_EQ(
    map.addPoints(
  {
    {{0.1F, 0.1F, 0.1F}, 1.0F},
    {{64.1F, 0.1F, 0.1F}, 2.0F},
    {{128.1F, 0.1F, 0.1F}, 3.0F}}), 3);
  EXPECT_EQ(map.size(), 2U);
  const auto stats = map.stats();
  EXPECT_EQ(stats.chunk_count, 2U);
  EXPECT_EQ(stats.marginalized_chunks, 1U);
  EXPECT_EQ(stats.marginalized_points, 1U);
  std::vector<itera_lio::frontend::MapPoint> points;
  ASSERT_TRUE(map.getAllPoints(points));
  EXPECT_TRUE(
    std::none_of(
      points.begin(), points.end(), [](const auto & point) {
        return point.intensity == 1.0F;
      }));
}
