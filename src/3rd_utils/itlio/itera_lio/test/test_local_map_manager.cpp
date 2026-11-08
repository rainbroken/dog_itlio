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

#include "itera_lio/frontend/ikd_map.hpp"
#include "itera_lio/frontend/local_map_manager.hpp"

TEST(LocalMapManager, ShiftsAndDeletesLeavingSlab)
{
  itera_lio::frontend::IkdMap map(0.1F);
  map.build({{{0.0F, 0.0F, 0.0F}}, {{100.0F, 0.0F, 0.0F}}});
  itera_lio::frontend::LocalMapOptions options;
  options.cube_size = 10.0F;
  options.move_threshold = 2.0F;
  options.delete_chunk_size = 2.0F;
  options.max_delete_boxes_per_update = 1U;
  itera_lio::frontend::LocalMapManager manager(options);
  EXPECT_TRUE(manager.update(Eigen::Vector3f::Zero(), map).empty());
  EXPECT_TRUE(manager.update(Eigen::Vector3f(1.0F, 0.0F, 0.0F), map).empty());
  EXPECT_FALSE(manager.update(Eigen::Vector3f(6.0F, 0.0F, 0.0F), map).empty());
  EXPECT_FLOAT_EQ(manager.center().x(), 6.0F);
  EXPECT_GT(manager.pendingDeleteBoxes(), 0U);
}

TEST(LocalMapManager, BoundsDeleteWorkAndDrainsPendingChunks)
{
  itera_lio::frontend::IkdMap map(0.1F);
  std::vector<itera_lio::frontend::MapPoint> points;
  for (int x = -5; x < 5; ++x) {
    for (int y = -5; y < 5; ++y) {
      for (int z = -5; z < 5; ++z) {
        points.push_back({Eigen::Vector3f(x + 0.1F, y + 0.1F, z + 0.1F), 1.0F});
      }
    }
  }
  map.build(points);
  itera_lio::frontend::LocalMapOptions options;
  options.cube_size = 10.0F;
  options.move_threshold = 2.0F;
  options.delete_chunk_size = 2.0F;
  options.max_delete_boxes_per_update = 1U;
  itera_lio::frontend::LocalMapManager manager(options);
  ASSERT_TRUE(manager.update(Eigen::Vector3f::Zero(), map).empty());
  EXPECT_EQ(manager.update(Eigen::Vector3f(3.0F, 0.0F, 0.0F), map).size(), 1U);
  EXPECT_GT(manager.pendingDeleteBoxes(), 0U);

  std::size_t updates = 1U;
  while (manager.pendingDeleteBoxes() > 0U && updates < 100U) {
    EXPECT_LE(manager.update(Eigen::Vector3f(3.0F, 0.0F, 0.0F), map).size(), 1U);
    ++updates;
  }
  EXPECT_EQ(manager.pendingDeleteBoxes(), 0U);
  EXPECT_GT(map.size(), 0U);
}

TEST(LocalMapManager, SplitsDenseBoxesToRespectPointBudget)
{
  itera_lio::frontend::IkdMap map(
    0.1F, itera_lio::frontend::IkdRebuildPolicy::kSynchronous);
  std::vector<itera_lio::frontend::MapPoint> points;
  for (int x = -4; x <= -3; ++x) {
    for (int y = -4; y <= 4; ++y) {
      for (int z = -4; z <= 4; ++z) {
        points.push_back(
          {Eigen::Vector3f(x + 0.1F, y + 0.1F, z + 0.1F), 1.0F});
      }
    }
  }
  map.build(points);

  itera_lio::frontend::LocalMapOptions options;
  options.cube_size = 10.0F;
  options.move_threshold = 2.0F;
  options.delete_chunk_size = 10.0F;
  options.max_delete_boxes_per_update = 1U;
  options.max_delete_points_per_update = 25U;
  options.max_delete_split_depth = 12U;
  itera_lio::frontend::LocalMapManager manager(options);

  ASSERT_TRUE(manager.update(Eigen::Vector3f::Zero(), map).empty());
  EXPECT_EQ(manager.update(Eigen::Vector3f(3.0F, 0.0F, 0.0F), map).size(), 1U);
  EXPECT_GT(manager.lastDeletedPointCount(), 0U);
  EXPECT_LE(manager.lastDeletedPointCount(), 25U);
  EXPECT_GT(manager.pendingDeleteBoxes(), 0U);
}
