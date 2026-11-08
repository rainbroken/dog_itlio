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
#include <chrono>
#include <stdexcept>
#include <thread>
#include <vector>

#include "itera_lio/frontend/ikd_map.hpp"

namespace
{

using itera_lio::frontend::IkdMap;
using itera_lio::frontend::MapBox;
using itera_lio::frontend::MapPoint;

MapPoint point(float x, float y, float z)
{
  MapPoint value;
  value.position = Eigen::Vector3f(x, y, z);
  return value;
}

MapPoint pointWithIntensity(float x, float y, float z, float intensity)
{
  MapPoint value = point(x, y, z);
  value.intensity = intensity;
  return value;
}

TEST(IkdMap, RejectsInvalidVoxelSize)
{
  EXPECT_THROW(IkdMap(0.0F), std::invalid_argument);
}

TEST(IkdMap, BuildsAndFindsNearestPoints)
{
  IkdMap map(0.1F);
  map.build({point(0.0F, 0.0F, 0.0F), point(1.0F, 0.0F, 0.0F), point(2.0F, 0.0F, 0.0F)});
  EXPECT_EQ(map.size(), 3U);
  std::vector<MapPoint> neighbors;
  std::vector<float> distances;
  ASSERT_TRUE(map.nearestSearch(point(0.2F, 0.0F, 0.0F), 2, 10.0F, neighbors, distances));
  ASSERT_EQ(neighbors.size(), 2U);
  ASSERT_EQ(distances.size(), 2U);
  EXPECT_NEAR(distances.front(), 0.04F, 1e-5F);
}

TEST(IkdMap, FindsNearestPositionsWithoutMetadataLookup)
{
  IkdMap map(0.1F);
  map.build({point(0.0F, 0.0F, 0.0F), point(1.0F, 0.0F, 0.0F)});
  std::vector<Eigen::Vector3d> neighbors;
  std::vector<float> distances;
  ASSERT_TRUE(
    map.nearestSearchPositions(
      Eigen::Vector3f(0.2F, 0.0F, 0.0F), 2, 10.0F, neighbors, distances));
  ASSERT_EQ(neighbors.size(), 2U);
  EXPECT_NEAR(neighbors.front().x(), 0.0, 1e-6);
  EXPECT_NEAR(distances.front(), 0.04F, 1e-5F);
}

TEST(IkdMap, ReusesSearchStorageAcrossChangingCountsAndEmptyResults)
{
  const auto exercise = []() {
      IkdMap map(0.01F);
      std::vector<MapPoint> points;
      for (int i = 1; i <= 20; ++i) {
        points.push_back(point(static_cast<float>(i), 0.0F, 0.0F));
      }
      map.build(points);
      std::vector<Eigen::Vector3d> neighbors;
      std::vector<float> distances;
      neighbors.reserve(64U);
      distances.reserve(64U);
      const auto distance_capacity = distances.capacity();
      for (const int k : {5, 1, 20, 3, 30, 2}) {
        ASSERT_TRUE(map.nearestSearchPositions(
            Eigen::Vector3f::Zero(), k, 100.0F, neighbors, distances));
        ASSERT_EQ(neighbors.size(), static_cast<std::size_t>(std::min(k, 20)));
        ASSERT_EQ(distances.size(), neighbors.size());
        EXPECT_EQ(distances.capacity(), distance_capacity);
        for (std::size_t i = 0; i < neighbors.size(); ++i) {
          EXPECT_DOUBLE_EQ(neighbors[i].x(), static_cast<double>(i + 1U));
          EXPECT_FLOAT_EQ(distances[i], static_cast<float>((i + 1U) * (i + 1U)));
        }
        EXPECT_FALSE(map.nearestSearchPositions(
            Eigen::Vector3f::Zero(), k, 0.1F, neighbors, distances));
        EXPECT_TRUE(neighbors.empty());
        EXPECT_TRUE(distances.empty());
        EXPECT_EQ(distances.capacity(), distance_capacity);
      }
    };
  // Independent trees must not share a mutable search workspace across threads.
  std::thread first(exercise);
  std::thread second(exercise);
  exercise();
  first.join();
  second.join();
}

TEST(IkdMap, KeepsNearestDistanceAndEqualDistanceOrdering)
{
  IkdMap map(0.01F);
  map.build({point(2.0F, 0.0F, 0.0F), point(-1.0F, 0.0F, 0.0F),
      point(-2.0F, 0.0F, 0.0F), point(1.0F, 0.0F, 0.0F)});
  std::vector<Eigen::Vector3d> neighbors;
  std::vector<float> distances;
  ASSERT_TRUE(map.nearestSearchPositions(
      Eigen::Vector3f::Zero(), 4, 10.0F, neighbors, distances));
  ASSERT_EQ(neighbors.size(), 4U);
  const double expected[] = {-1.0, 1.0, -2.0, 2.0};
  for (std::size_t i = 0; i < neighbors.size(); ++i) {
    EXPECT_DOUBLE_EQ(neighbors[i].x(), expected[i]);
    EXPECT_FLOAT_EQ(distances[i], static_cast<float>(expected[i] * expected[i]));
  }
}

TEST(IkdMap, KeepsIntensityInTreePoints)
{
  IkdMap map(0.1F);
  map.build({pointWithIntensity(0.0F, 0.0F, 0.0F, 7.0F)});

  std::vector<MapPoint> neighbors;
  std::vector<float> distances;
  ASSERT_TRUE(map.nearestSearch(point(0.01F, 0.0F, 0.0F), 1, 1.0F, neighbors, distances));
  ASSERT_EQ(neighbors.size(), 1U);
  EXPECT_FLOAT_EQ(neighbors.front().intensity, 7.0F);

  std::vector<MapPoint> points;
  ASSERT_TRUE(map.getAllPoints(points));
  ASSERT_EQ(points.size(), 1U);
  EXPECT_FLOAT_EQ(points.front().intensity, 7.0F);
}

TEST(IkdMap, ExportsAccumulatedPoints)
{
  IkdMap map(0.1F);
  map.build({point(0.0F, 0.0F, 0.0F), point(1.0F, 0.0F, 0.0F)});
  ASSERT_EQ(map.addPoints({point(2.0F, 0.0F, 0.0F)}, false).inserted_count, 1U);

  std::vector<MapPoint> points;
  ASSERT_TRUE(map.getAllPoints(points));
  EXPECT_EQ(points.size(), 3U);
}

TEST(IkdMap, AddsAndDeletesByBox)
{
  IkdMap map(0.1F);
  map.build({point(0.0F, 0.0F, 0.0F)});
  EXPECT_EQ(map.addPoints({point(1.0F, 0.0F, 0.0F)}, false).inserted_count, 1U);
  EXPECT_EQ(map.size(), 2U);
  MapBox box;
  box.min = Eigen::Vector3f(-0.1F, -0.1F, -0.1F);
  box.max = Eigen::Vector3f(0.1F, 0.1F, 0.1F);
  EXPECT_EQ(map.deleteBoxes({box}), 1);
  EXPECT_EQ(map.size(), 1U);
}

TEST(IkdMap, CountsPointsExactlyAndStopsAtLimit)
{
  IkdMap map(0.1F);
  map.build(
    {point(0.0F, 0.0F, 0.0F), point(1.0F, 0.0F, 0.0F),
      point(2.0F, 0.0F, 0.0F), point(3.0F, 0.0F, 0.0F)});
  MapBox box;
  box.min = Eigen::Vector3f(-0.1F, -0.1F, -0.1F);
  box.max = Eigen::Vector3f(3.1F, 0.1F, 0.1F);

  EXPECT_EQ(map.countPoints(box), 4U);
  EXPECT_EQ(map.countPoints(box, 2U), 2U);
  EXPECT_EQ(map.countPoints(box, 0U), 0U);
}

TEST(IkdMap, CountExcludesDeletedPoints)
{
  IkdMap map(0.1F);
  map.build(
    {point(0.0F, 0.0F, 0.0F), point(1.0F, 0.0F, 0.0F),
      point(2.0F, 0.0F, 0.0F)});
  MapBox deleted_box;
  deleted_box.min = Eigen::Vector3f(0.9F, -0.1F, -0.1F);
  deleted_box.max = Eigen::Vector3f(1.1F, 0.1F, 0.1F);
  ASSERT_EQ(map.deleteBoxes({deleted_box}), 1);

  MapBox all_points;
  all_points.min = Eigen::Vector3f(-0.1F, -0.1F, -0.1F);
  all_points.max = Eigen::Vector3f(2.1F, 0.1F, 0.1F);
  EXPECT_EQ(map.countPoints(all_points), 2U);
}

TEST(IkdMap, ReportsDownsampleInsertReplaceAndReject)
{
  IkdMap map(1.0F);
  map.build({pointWithIntensity(0.40F, 0.40F, 0.40F, 1.0F)});

  const auto rejected = map.addPoints(
    {pointWithIntensity(0.10F, 0.10F, 0.10F, 2.0F)}, true);
  EXPECT_EQ(rejected.input_count, 1U);
  EXPECT_EQ(rejected.inserted_count, 0U);
  EXPECT_EQ(rejected.replaced_count, 0U);
  EXPECT_EQ(rejected.rejected_count, 1U);
  EXPECT_EQ(rejected.net_size_delta, 0);
  EXPECT_TRUE(rejected.upserted_points.empty());

  const auto replaced = map.addPoints(
    {pointWithIntensity(0.49F, 0.49F, 0.49F, 3.0F)}, true);
  EXPECT_EQ(replaced.inserted_count, 0U);
  EXPECT_EQ(replaced.replaced_count, 1U);
  EXPECT_EQ(replaced.rejected_count, 0U);
  EXPECT_EQ(replaced.net_size_delta, 0);
  ASSERT_EQ(replaced.upserted_points.size(), 1U);
  EXPECT_FLOAT_EQ(replaced.upserted_points.front().intensity, 3.0F);
  EXPECT_EQ(replaced.removed_points.size(), 1U);

  const auto inserted = map.addPoints(
    {pointWithIntensity(1.50F, 0.50F, 0.50F, 4.0F)}, true);
  EXPECT_EQ(inserted.inserted_count, 1U);
  EXPECT_EQ(inserted.replaced_count, 0U);
  EXPECT_EQ(inserted.rejected_count, 0U);
  EXPECT_EQ(inserted.net_size_delta, 1);
  ASSERT_EQ(inserted.upserted_points.size(), 1U);
  EXPECT_FLOAT_EQ(inserted.upserted_points.front().intensity, 4.0F);
  EXPECT_EQ(map.size(), 2U);
}

TEST(IkdMap, ReportsCompactionSeparatelyFromRejectedCandidate)
{
  IkdMap map(1.0F);
  map.build(
    {pointWithIntensity(0.49F, 0.49F, 0.49F, 1.0F),
      pointWithIntensity(0.20F, 0.20F, 0.20F, 2.0F)});

  const auto report = map.addPoints(
    {pointWithIntensity(0.10F, 0.10F, 0.10F, 3.0F)}, true);
  EXPECT_EQ(report.input_count, 1U);
  EXPECT_EQ(report.inserted_count, 0U);
  EXPECT_EQ(report.replaced_count, 0U);
  EXPECT_EQ(report.rejected_count, 1U);
  EXPECT_EQ(report.compacted_count, 1U);
  EXPECT_EQ(report.net_size_delta, -1);
  EXPECT_TRUE(report.upserted_points.empty());
  EXPECT_EQ(report.removed_points.size(), 2U);
  EXPECT_EQ(map.size(), 1U);
}

TEST(IkdMap, ReportsSynchronousRebuilds)
{
  IkdMap map(0.1F, itera_lio::frontend::IkdRebuildPolicy::kSynchronous);
  std::vector<itera_lio::frontend::MapPoint> points;
  map.build(
    {
      {Eigen::Vector3f(0.0F, 0.0F, 0.0F), 1.0F},
      {Eigen::Vector3f(0.2F, 0.0F, 0.0F), 1.0F},
      {Eigen::Vector3f(0.4F, 0.0F, 0.0F), 1.0F}});
  points.reserve(197U);
  for (int index = 3; index < 200; ++index) {
    points.push_back({Eigen::Vector3f(index * 0.2F, 0.0F, 0.0F), 1.0F});
  }
  EXPECT_EQ(map.addPoints(points, false).inserted_count, points.size());
  const auto stats = map.rebuildStats();
  EXPECT_GT(stats.total_count, 0U);
  EXPECT_EQ(stats.total_count, stats.synchronous_count);
  EXPECT_EQ(stats.asynchronous_count, 0U);
  EXPECT_GT(stats.max_subtree_size, 10U);
  EXPECT_GE(stats.max_duration_ms, 0.0);
  EXPECT_GE(stats.search_lock_wait_total_ms, 0.0);
  EXPECT_GE(stats.search_lock_wait_max_ms, 0.0);
}

}  // namespace

TEST(IkdMap, ConcurrentReadMatchesSerialAfterLazyDeletion)
{
  using namespace itera_lio;
  frontend::IkdMap map(0.1F, frontend::IkdRebuildPolicy::kSynchronous);
  std::vector<common::MapPoint> points;
  for (int x = 0; x < 25; ++x) {
    for (int y = 0; y < 25; ++y) {
      points.push_back({Eigen::Vector3f(x * .1F, y * .1F, .01F * (x % 3)), float(x + y)});
    }
  }
  map.build(points);
  common::MapBox box;
  box.min = Eigen::Vector3f(.3F, .3F, -1.F);
  box.max = Eigen::Vector3f(.9F, .9F, 1.F);
  map.deleteBoxes({box});
  std::vector<std::vector<Eigen::Vector3d>> actual(120), expected(120);
  std::vector<std::vector<float>> distances(120), expected_distances(120);
  {
    auto guard = map.prepareConcurrentRead();
    ASSERT_TRUE(map.concurrentReadPrepared());
    std::vector<std::thread> workers;
    for (int worker = 0; worker < 3; ++worker) {
      workers.emplace_back([&, worker] {
        for (int i = worker; i < 120; i += 3) {
          map.nearestSearchPositions(Eigen::Vector3f(i * .019F, .7F, .2F),
            5, 3.F, actual[i], distances[i]);
        }
      });
    }
    for (auto & worker : workers) {worker.join();}
  }
  EXPECT_FALSE(map.concurrentReadPrepared());
  for (int i = 0; i < 120; ++i) {
    map.nearestSearchPositions(Eigen::Vector3f(i * .019F, .7F, .2F),
      5, 3.F, expected[i], expected_distances[i]);
    EXPECT_EQ(actual[i], expected[i]);
    EXPECT_EQ(distances[i], expected_distances[i]);
  }
}


TEST(IkdMap, ExactSizeDuringAsynchronousRootRebuildAndReadGuard)
{
  using itera_lio::frontend::IkdRebuildPolicy;
  IkdMap map(0.01F, IkdRebuildPolicy::kAsynchronous);
  EXPECT_EQ(map.size(), 0U);
  std::vector<MapPoint> points;
  constexpr int count = 150000;
  points.reserve(count);
  for (int i = 0; i < count; ++i) {
    points.push_back(point(float(i), float(i % 11), float(i % 7)));
  }
  map.build(points);
  EXPECT_EQ(map.size(), static_cast<std::size_t>(count));
  MapBox removed;
  removed.min = Eigen::Vector3f(-1.F, -1.F, -1.F);
  removed.max = Eigen::Vector3f(90000.F, 20.F, 20.F);
  ASSERT_EQ(map.deleteBoxes({removed}), 90000);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  bool saw_rebuild = false;
  do {
    EXPECT_EQ(map.size(), 60000U);
    const auto stats = map.rebuildStats();
    saw_rebuild = saw_rebuild || stats.asynchronous_in_progress;
    if (stats.asynchronous_count > 0U && !stats.asynchronous_in_progress) {break;}
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < deadline);
  EXPECT_TRUE(saw_rebuild);
  EXPECT_GT(map.rebuildStats().asynchronous_count, 0U);
  EXPECT_EQ(map.size(), 60000U);
  {
    auto guard = map.prepareConcurrentRead();
    EXPECT_EQ(map.size(), 60000U);
  }
  EXPECT_EQ(map.addPoints({point(200000.F, 0.F, 0.F)}, false).inserted_count, 1U);
  EXPECT_EQ(map.size(), 60001U);
}

TEST(IkdMap, ReadOnlyQueriesRemainExactAcrossPendingAsynchronousRebuild)
{
  using itera_lio::frontend::IkdRebuildPolicy;
  IkdMap map(.01F, IkdRebuildPolicy::kAsynchronous);
  std::vector<MapPoint> points;
  for (int i = 0; i < 150000; ++i)
    points.push_back(point(float(i), float(i % 11), float(i % 7)));
  map.build(points);
  MapBox removed;
  removed.min = Eigen::Vector3f(-1.F, -1.F, -1.F);
  removed.max = Eigen::Vector3f(90000.F, 20.F, 20.F);
  ASSERT_EQ(map.deleteBoxes({removed}), 90000);
  std::vector<std::vector<Eigen::Vector3d>> actual(240), expected(240);
  std::vector<std::vector<float>> ad(240), ed(240);
  {
    auto guard = map.prepareConcurrentRead();
    std::vector<std::thread> workers;
    EXPECT_EQ(map.size(), 60000U);  // Must not invert working/search lock order.
    for (int w = 0; w < 3; ++w) workers.emplace_back([&, w] {
      for (int i = w; i < 240; i += 3)
        map.nearestSearchPositions(Eigen::Vector3f(90000.25F + i * 7.125F, 3.1F, 2.2F),
          5, 100.F, actual[i], ad[i]);
    });
    for (auto & worker : workers) worker.join();
  }
  for (int i = 0; i < 240; ++i) {
    map.nearestSearchPositions(Eigen::Vector3f(90000.25F + i * 7.125F, 3.1F, 2.2F),
      5, 100.F, expected[i], ed[i]);
    EXPECT_EQ(actual[i], expected[i]);
    EXPECT_EQ(ad[i], ed[i]);
    for (const auto & p : actual[i]) EXPECT_GE(p.x(), 90000.);
  }
}
