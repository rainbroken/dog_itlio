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

#include <limits>
#include <cmath>
#include <random>
#include <set>
#include <tuple>

#include "itera_lio/frontend/lidar_preprocessor.hpp"
#include "itera_lio/common/voxel_visit_set.hpp"

TEST(LidarPreprocessor, FiltersNearFieldPointsAtConfiguredRadialRange)
{
  itera_lio::sensor::LidarFrame input;
  input.scan_begin_time = 0.0;
  input.scan_end_time = 0.0;
  input.points = {
    {{0.3, 0.0, 0.0}, 1.0F, 0.0},
    {{0.5, 0.0, 0.0}, 2.0F, 0.0},
    {{0.0, 0.0, 0.6}, 3.0F, 0.0},
    {{1.0, 0.0, 0.0}, 4.0F, 0.0}};

  const auto filtered = itera_lio::frontend::filterLidarFrameByRange(input, 0.5);
  ASSERT_EQ(filtered.points.size(), 3U);
  EXPECT_FLOAT_EQ(filtered.points[0].intensity, 2.0F);
  EXPECT_FLOAT_EQ(filtered.points[1].intensity, 3.0F);
  EXPECT_FLOAT_EQ(filtered.points[2].intensity, 4.0F);
}

TEST(LidarPreprocessor, SamplesOnePointPerVoxel)
{
  itera_lio::sensor::LidarFrame input;
  input.scan_begin_time = 0.0;
  input.scan_end_time = 0.0;
  input.points = {
    {{0.01, 0.01, 0.01}, 1.0F, 0.0},
    {{0.05, 0.05, 0.05}, 2.0F, 0.0},
    {{0.25, 0.01, 0.01}, 3.0F, 0.0}};
  EXPECT_EQ(itera_lio::frontend::voxelSampleLidarFrame(input, 0.2F).points.size(), 2U);
}

TEST(LidarPreprocessor, StrideSamplingMatchesFastLioPointFilterSemantics)
{
  using namespace itera_lio;
  sensor::LidarFrame input;
  input.scan_begin_time = 1.0;
  input.scan_end_time = 2.0;
  for (std::size_t i = 0U; i < 8U; ++i) {
    input.points.push_back({{static_cast<double>(i + 1U), 0.0, 0.0},
      static_cast<float>(i), static_cast<double>(i) * 0.01});
  }

  const auto all = frontend::sampleLidarFrameByStride(input, 1U);
  ASSERT_EQ(all.points.size(), input.points.size());
  EXPECT_EQ(all.points.front().intensity, 0.0F);
  EXPECT_EQ(all.points.back().intensity, 7.0F);
  EXPECT_DOUBLE_EQ(all.scan_begin_time, input.scan_begin_time);
  EXPECT_DOUBLE_EQ(all.scan_end_time, input.scan_end_time);

  const auto every_third = frontend::sampleLidarFrameByStride(input, 3U);
  ASSERT_EQ(every_third.points.size(), 3U);
  EXPECT_FLOAT_EQ(every_third.points[0].intensity, 0.0F);
  EXPECT_FLOAT_EQ(every_third.points[1].intensity, 3.0F);
  EXPECT_FLOAT_EQ(every_third.points[2].intensity, 6.0F);
  EXPECT_DOUBLE_EQ(every_third.points[1].relative_time, input.points[3].relative_time);

  sensor::LidarFrame reused;
  frontend::sampleLidarFrameByStride(input, 3U, reused);
  ASSERT_EQ(reused.points.size(), every_third.points.size());
  for (std::size_t i = 0U; i < reused.points.size(); ++i) {
    EXPECT_EQ(reused.points[i].intensity, every_third.points[i].intensity);
  }
}

TEST(LidarPreprocessor, PreservesFirstPointOrderAcrossVoxelSampling)
{
  itera_lio::sensor::LidarFrame input;
  input.scan_begin_time = 0.0;
  input.scan_end_time = 0.0;
  input.points = {
    {{0.01, 0.01, 0.01}, 1.0F, 0.0},
    {{0.25, 0.01, 0.01}, 3.0F, 0.0},
    {{0.05, 0.05, 0.05}, 2.0F, 0.0}};

  const auto sampled = itera_lio::frontend::voxelSampleLidarFrame(input, 0.2F);
  ASSERT_EQ(sampled.points.size(), 2U);
  EXPECT_FLOAT_EQ(sampled.points[0].intensity, 1.0F);
  EXPECT_FLOAT_EQ(sampled.points[1].intensity, 3.0F);
}

TEST(LidarPreprocessor, ReportsSelectedIndicesInInputOrder)
{
  itera_lio::sensor::LidarFrame input;
  input.scan_begin_time = 0.0;
  input.scan_end_time = 0.0;
  input.points = {
    {{0.01, 0.01, 0.01}, 1.0F, 0.0},
    {{0.25, 0.01, 0.01}, 2.0F, 0.0},
    {{0.05, 0.05, 0.05}, 3.0F, 0.0}};

  const auto indices = itera_lio::frontend::voxelSelectLidarPointIndices(input, 0.2F);
  EXPECT_EQ(indices, (std::vector<std::size_t>{0U, 1U}));
}

TEST(LidarPreprocessor, ReusedVoxelStorageMatchesReferenceAcrossScanSizes)
{
  for (const int size : {20000, 17, 30000, 0, 20000}) {
    itera_lio::sensor::LidarFrame input;
    input.scan_begin_time = input.scan_end_time = 0.0;
    std::set<std::tuple<int, int, int>> visited;
    std::vector<std::size_t> expected;
    constexpr float voxel = 0.05F;
    for (int i = 0; i < size; ++i) {
      const Eigen::Vector3d position{
        ((i % 201) - 100) * 0.021,
        (((i / 201) % 101) - 50) * 0.021,
        (i % 7) * 0.003};
      input.points.push_back({position, 1.0F, 0.0});
      if (visited.emplace(
          static_cast<int>(std::floor(position.x() / voxel)),
          static_cast<int>(std::floor(position.y() / voxel)),
          static_cast<int>(std::floor(position.z() / voxel))).second) {
        expected.push_back(static_cast<std::size_t>(i));
      }
    }
    EXPECT_EQ(itera_lio::frontend::voxelSelectLidarPointIndices(input, voxel), expected);
  }
}

TEST(LidarPreprocessor, RejectsInvalidRangeCutoff)
{
  itera_lio::sensor::LidarFrame input;
  input.scan_begin_time = 0.0;
  input.scan_end_time = 0.0;
  input.points = {{{1.0, 0.0, 0.0}, 1.0F, 0.0}};
  EXPECT_TRUE(itera_lio::frontend::filterLidarFrameByRange(input, -0.1).points.empty());
  EXPECT_TRUE(
    itera_lio::frontend::filterLidarFrameByRange(
      input, std::numeric_limits<double>::quiet_NaN()).points.empty());
}

TEST(LidarPreprocessor, RangeFastPathMatchesHypotAtBoundariesAndExtremeMagnitudes)
{
  using namespace itera_lio;
  std::mt19937_64 random(92031);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  sensor::LidarFrame reused;
  for (const double cutoff : {0.0, 0.8, 1.0, 1e-200, 1e-150, 1e150, 1e200,
      std::numeric_limits<double>::max()})
  {
    sensor::LidarFrame input;
    for (int i = 0; i < 20000; ++i) {
      Eigen::Vector3d point{uniform(random), uniform(random), uniform(random)};
      point.normalize();
      double radius = cutoff;
      if (i % 3 == 0) {radius = std::nextafter(cutoff, 0.0);}
      if (i % 3 == 1) {radius = std::nextafter(cutoff, INFINITY);}
      if (std::isfinite(radius)) {
        input.points.push_back({point * radius, static_cast<float>(i), 0.0});
      }
    }
    input.points.push_back({{1e308, 1e308, 1e308}, -1.0F, 0.0});
    std::vector<float> expected;
    ASSERT_TRUE(input.isValid());
    for (const auto & point : input.points) {
      const double range = std::hypot(std::hypot(
          point.position.x(), point.position.y()), point.position.z());
      if (cutoff == 0.0 || (std::isfinite(range) && range >= cutoff)) {
        expected.push_back(point.intensity);
      }
    }
    frontend::filterLidarFrameByRange(input, cutoff, reused);
    ASSERT_EQ(reused.points.size(), expected.size()) << cutoff;
    for (std::size_t i = 0; i < expected.size(); ++i) {
      EXPECT_EQ(reused.points[i].intensity, expected[i]) << cutoff << " index " << i;
    }
    auto alias = input;
    frontend::filterLidarFrameByRange(alias, cutoff, alias);
    ASSERT_EQ(alias.points.size(), reused.points.size());
    for (std::size_t i = 0; i < reused.points.size(); ++i) {
      EXPECT_EQ(alias.points[i].position, reused.points[i].position);
    }
  }
  sensor::LidarFrame invalid;
  invalid.points.push_back({{NAN, 0, 0}, 0, 0});
  frontend::filterLidarFrameByRange(invalid, .8, reused);
  EXPECT_TRUE(reused.points.empty());
}

TEST(LidarPreprocessor, RangeFilterDiscardsPartialOutputOnLateInvalidPoint)
{
  using namespace itera_lio;
  sensor::LidarFrame input;
  input.scan_begin_time = 1.0; input.scan_end_time = 2.0;
  input.points = {{{1.0, 0.0, 0.0}, 1.0F, 0.0},
                  {{2.0, 0.0, 0.0}, 1.0F, 0.5},
                  {{3.0, 0.0, 0.0}, 1.0F, 1.1}};
  sensor::LidarFrame output;
  frontend::filterLidarFrameByRange(input, 0.8, output);
  EXPECT_TRUE(output.points.empty());
  EXPECT_EQ(output.scan_begin_time, input.scan_begin_time);
  EXPECT_EQ(output.scan_end_time, input.scan_end_time);
  input.points.back().relative_time = 1.0;
  input.points.back().intensity = std::numeric_limits<float>::quiet_NaN();
  frontend::filterLidarFrameByRange(input, 0.8, output);
  EXPECT_TRUE(output.points.empty());
  input.points.back().intensity = 1.0F;
  frontend::filterLidarFrameByRange(input, 0.8, output);
  EXPECT_EQ(output.points.size(), 3U);
}

TEST(LidarPreprocessor, CallerOwnedVoxelIndicesClearStaleResultsAndReuseCapacity)
{
  itera_lio::sensor::LidarFrame input;
  input.points = {{{-0.5, 0.0, 0.0}, 1.0F, 0.0},
    {{-0.49, 0.0, 0.0}, 2.0F, 0.0}, {{0.5, 0.0, 0.0}, 3.0F, 0.0}};
  std::vector<std::size_t> indices;
  itera_lio::frontend::voxelSelectLidarPointIndices(input, 0.5F, indices);
  EXPECT_EQ(indices, (std::vector<std::size_t>{0U, 2U}));
  const auto capacity = indices.capacity();
  const auto * storage = indices.data();
  input.points.back().relative_time = 1.0;
  itera_lio::frontend::voxelSelectLidarPointIndices(input, 0.5F, indices);
  EXPECT_TRUE(indices.empty());
  input.points.back().relative_time = 0.0;
  input.points.resize(1U);
  itera_lio::frontend::voxelSelectLidarPointIndices(input, 0.5F, indices);
  EXPECT_EQ(indices, (std::vector<std::size_t>{0U}));
  EXPECT_EQ(indices.capacity(), capacity);
  EXPECT_EQ(indices.data(), storage);
  itera_lio::frontend::voxelSelectLidarPointIndices(input, 0.0F, indices);
  EXPECT_TRUE(indices.empty());
}

TEST(LidarPreprocessor, ValidatedVoxelSelectorMatchesCheckedSelectorExactly)
{
  std::mt19937 generator(20261023U);
  std::uniform_real_distribution<double> coordinate(-40.0, 40.0);
  itera_lio::sensor::LidarFrame input;
  input.scan_begin_time = 10.0;
  input.scan_end_time = 10.1;
  for (std::size_t index = 0U; index < 50000U; ++index) {
    const double time = static_cast<double>(index % 1000U) * 0.0001;
    input.points.push_back({{coordinate(generator), coordinate(generator),
      coordinate(generator)}, static_cast<float>(index % 256U), time});
  }
  std::vector<std::size_t> checked;
  std::vector<std::size_t> validated;
  for (const float voxel : {0.5F, 0.3F, 0.05F}) {
    itera_lio::frontend::voxelSelectLidarPointIndices(input, voxel, checked);
    itera_lio::frontend::voxelSelectLidarPointIndicesFromValidatedFrame(input, voxel, validated);
    EXPECT_EQ(validated, checked);
  }
}

TEST(LidarPreprocessor, HalfMeterVoxelSelectionPreservesFloorBoundaries)
{
  itera_lio::sensor::LidarFrame input;
  input.scan_begin_time = 0.0;
  input.scan_end_time = 0.1;
  std::vector<double> coordinates;
  for (int cell = -1000; cell <= 1000; ++cell) {
    const double boundary = static_cast<double>(cell) * 0.5;
    coordinates.push_back(std::nextafter(boundary, -std::numeric_limits<double>::infinity()));
    coordinates.push_back(boundary);
    coordinates.push_back(std::nextafter(boundary, std::numeric_limits<double>::infinity()));
  }
  for (std::size_t index = 0U; index < coordinates.size(); ++index) {
    const double y = static_cast<double>(index % 31U) * 0.5 - 7.5;
    const double z = static_cast<double>(index % 19U) * 0.5 - 4.5;
    input.points.push_back({{coordinates[index], y, z}, 1.0F, 0.0});
  }
  std::set<std::tuple<int, int, int>> visited;
  std::vector<std::size_t> expected;
  for (std::size_t index = 0U; index < input.points.size(); ++index) {
    const auto & point = input.points[index].position;
    const auto key = std::make_tuple(
      static_cast<int>(std::floor(point.x() / 0.5F)),
      static_cast<int>(std::floor(point.y() / 0.5F)),
      static_cast<int>(std::floor(point.z() / 0.5F)));
    if (visited.insert(key).second) {expected.push_back(index);}
  }
  const auto actual = itera_lio::frontend::voxelSelectLidarPointIndices(input, 0.5F);
  EXPECT_EQ(actual, expected);
}

TEST(LidarPreprocessor, ReciprocalVoxelSelectionPreservesNonBinaryBoundaries)
{
  for (const float voxel : {0.3F, 0.05F}) {
    itera_lio::sensor::LidarFrame input;
    input.scan_begin_time = 0.0;
    input.scan_end_time = 0.1;
    for (int cell = -1000; cell <= 1000; ++cell) {
      const double boundary = static_cast<double>(cell) * static_cast<double>(voxel);
      for (const double coordinate : {
          std::nextafter(boundary, -std::numeric_limits<double>::infinity()), boundary,
          std::nextafter(boundary, std::numeric_limits<double>::infinity())})
      {
        input.points.push_back({{coordinate, coordinate * 0.25, -coordinate * 0.125},
          1.0F, 0.0});
      }
    }
    std::set<std::tuple<int, int, int>> visited;
    std::vector<std::size_t> expected;
    for (std::size_t index = 0U; index < input.points.size(); ++index) {
      const auto & point = input.points[index].position;
      const auto key = std::make_tuple(
        static_cast<int>(std::floor(point.x() / voxel)),
        static_cast<int>(std::floor(point.y() / voxel)),
        static_cast<int>(std::floor(point.z() / voxel)));
      if (visited.insert(key).second) {expected.push_back(index);}
    }
    EXPECT_EQ(itera_lio::frontend::voxelSelectLidarPointIndices(input, voxel), expected);
  }
}

TEST(VoxelVisitSet, FullWidthKeysAndResetGrowthMatchOrderedReference)
{
  itera_lio::common::VoxelVisitSet visited;
  std::mt19937_64 random(2701);
  for (const std::size_t count : {60000U, 7U, 90000U, 60000U}) {
    visited.reset(count);
    std::set<std::tuple<std::int64_t, std::int64_t, std::int64_t>> reference;
    for (std::size_t i = 0U; i < count; ++i) {
      const std::int64_t x = static_cast<std::int64_t>(random() % 200001U) - 100000;
      const itera_lio::common::VoxelKey key{
        i % 3U == 0U ? std::numeric_limits<std::int64_t>::min() : x,
        i % 5U == 0U ? std::numeric_limits<std::int64_t>::max() : x / 3,
        -x};
      const auto expected = reference.emplace(key.x, key.y, key.z).second;
      ASSERT_EQ(visited.insert(key), expected);
      ASSERT_FALSE(visited.insert(key));
    }
  }
}
