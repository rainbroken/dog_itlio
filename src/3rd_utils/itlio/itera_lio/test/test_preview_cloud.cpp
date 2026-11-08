// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "itera_lio/output/preview_cloud.hpp"

TEST(PreviewMap, KeepsClosestRepresentativeWithinVoxel) {
  itera_lio::output::preview::PreviewMap map(1.0F, 0U);
  map.addPoints({{{0.1F, 0.1F, 0.1F}, 1.0F}, {{0.49F, 0.49F, 0.49F}, 2.0F}});
  ASSERT_EQ(map.size(), 1U);

  std::vector<itera_lio::common::MapPoint> points;
  ASSERT_TRUE(map.getAllPoints(points));
  ASSERT_EQ(points.size(), 1U);
  EXPECT_FLOAT_EQ(points.front().intensity, 2.0F);
}

TEST(PreviewMap, HonorsConfiguredPointBudget) {
  itera_lio::output::preview::PreviewMap map(1.0F, 2U);
  map.addPoints({{{0.1F, 0.1F, 0.1F}, 1.0F},
                 {{1.1F, 0.1F, 0.1F}, 2.0F},
                 {{2.1F, 0.1F, 0.1F}, 3.0F}});
  EXPECT_LE(map.size(), 2U);
  std::vector<itera_lio::common::MapPoint> points;
  ASSERT_TRUE(map.getAllPoints(points));
  EXPECT_LE(points.size(), 2U);
}

TEST(PreviewMap, ReadsSnapshotInBatches) {
  itera_lio::output::preview::PreviewMap map(0.1F, 0U);
  map.addPoints({{{0.01F, 0.01F, 0.01F}, 1.0F},
                 {{0.21F, 0.01F, 0.01F}, 2.0F},
                 {{0.41F, 0.01F, 0.01F}, 3.0F}});

  std::vector<itera_lio::common::MapPoint> points;
  ASSERT_TRUE(map.getAllPointsBatched(points, 1U));
  ASSERT_EQ(points.size(), 3U);
  std::vector<float> intensities;
  for (const auto &point : points) {
    intensities.push_back(point.intensity);
  }
  std::sort(intensities.begin(), intensities.end());
  EXPECT_EQ(intensities, (std::vector<float>{1.0F, 2.0F, 3.0F}));
  EXPECT_FALSE(map.getAllPointsBatched(points, 0U));
}
