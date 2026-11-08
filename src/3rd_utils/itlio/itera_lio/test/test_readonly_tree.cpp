#include <gtest/gtest.h>
#include <algorithm>
#include <cstdint>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <thread>
#include "ikd_tree/ikd_Tree.h"

using Tree = KD_TREE<ikdTree_PointType>;

static std::uint64_t nodeFlags(Tree & tree) {
  auto visit = [](auto self, auto *node, std::uint64_t h) -> std::uint64_t {
    if (!node) return h;
    const std::uint64_t flags = node->tree_deleted | (node->point_deleted << 1) |
      (node->tree_downsample_deleted << 2) | (node->point_downsample_deleted << 3) |
      (node->need_push_down_to_left << 4) | (node->need_push_down_to_right << 5);
    h = (h ^ flags) * 1099511628211ULL;
    return self(self, node->right_son_ptr, self(self, node->left_son_ptr, h));
  };
  return visit(visit, tree.Root_Node, 1469598103934665603ULL);
}

static void compareQueries(Tree & tree, const Tree::PointVector & queries) {
  std::vector<Tree::PointVector> actual(queries.size()), expected(queries.size());
  std::vector<std::vector<float>> ad(queries.size()), ed(queries.size());
  const auto before = nodeFlags(tree);
  tree.Begin_Concurrent_Read();
  std::vector<std::thread> threads;
  for (std::size_t worker = 0; worker < 3; ++worker) {
    threads.emplace_back([&, worker] {
      for (std::size_t i = worker; i < queries.size(); i += 3)
        tree.Nearest_Search(queries[i], 1 + int(i % 9), actual[i], ad[i], i % 4 ? 100. : .25);
    });
  }
  for (auto & thread : threads) thread.join();
  tree.End_Concurrent_Read();
  EXPECT_EQ(nodeFlags(tree), before) << "read-only queries mutated lazy flags";
  tree.Begin_Concurrent_Read(true);  // Original materialization + original search.
  for (std::size_t i = 0; i < queries.size(); ++i)
    tree.Nearest_Search(queries[i], 1 + int(i % 9), expected[i], ed[i], i % 4 ? 100. : .25);
  tree.End_Concurrent_Read();
  for (std::size_t i = 0; i < queries.size(); ++i) {
    ASSERT_EQ(actual[i].size(), expected[i].size()) << i;
    EXPECT_EQ(ad[i], ed[i]) << i;
    for (std::size_t k = 0; k < actual[i].size(); ++k) {
      EXPECT_FLOAT_EQ(actual[i][k].x, expected[i][k].x);
      EXPECT_FLOAT_EQ(actual[i][k].y, expected[i][k].y);
      EXPECT_FLOAT_EQ(actual[i][k].z, expected[i][k].z);
      EXPECT_FLOAT_EQ(actual[i][k].intensity, expected[i][k].intensity);
    }
  }
}

TEST(ReadOnlyTree, ExactOrderedNeighborsThroughDeleteRestoreAndDownsample) {
  auto storage = std::make_unique<Tree>(.99F, 1.F, .25F, false);
  auto & tree = *storage;
  Tree::PointVector points, queries;
  for (int i = 0; i < 4096; ++i) {
    ikdTree_PointType p(float(i % 16) * .2F, float(i / 16 % 16) * .2F, float(i / 256) * .2F);
    p.intensity = float(i); points.push_back(p);
  }
  tree.Build(points);
  std::mt19937 rng(9817);
  std::uniform_real_distribution<float> dist(-.5F, 3.5F);
  for (int i = 0; i < 160; ++i) queries.emplace_back(dist(rng), dist(rng), dist(rng));
  queries.emplace_back(1.F, 1.F, 1.F);  // Exact ties on the regular grid.
  compareQueries(tree, queries);
  for (int step = 0; step < 40; ++step) {
    BoxPointType box;
    for (int axis = 0; axis < 3; ++axis) {
      box.vertex_min[axis] = dist(rng);
      box.vertex_max[axis] = box.vertex_min[axis] + .5F + float(step % 5) * .3F;
    }
    std::vector<BoxPointType> boxes{box};
    tree.Delete_Point_Boxes(boxes);
    if (step % 2 == 0) tree.Add_Point_Boxes(boxes);
    Tree::PointVector added;
    for (int i = 0; i < 8; ++i) {
      ikdTree_PointType p(dist(rng), dist(rng), dist(rng)); p.intensity = float(5000 + 8 * step + i);
      added.push_back(p);
    }
    tree.Add_Points(added, true);
    compareQueries(tree, queries);
  }
  BoxPointType all; for (int axis = 0; axis < 3; ++axis) {all.vertex_min[axis] = -100.F; all.vertex_max[axis] = 100.F;}
  std::vector<BoxPointType> boxes{all};
  tree.Delete_Point_Boxes(boxes); compareQueries(tree, queries);
  tree.Add_Point_Boxes(boxes); compareQueries(tree, queries);
}

TEST(ReadOnlyTree, EmptyAndSingleNode) {
  auto storage = std::make_unique<Tree>(.99F, 1.F, .25F, false);
  auto & tree = *storage;
  Tree::PointVector q{ikdTree_PointType(0.F, 0.F, 0.F)};
  compareQueries(tree, q);
  tree.Build(q); compareQueries(tree, q);
}

TEST(ReadOnlyTree, DISABLED_PrepareAndQueryBenchmark) {
  for (const int count : {100000, 500000, 1500000}) {
    auto storage = std::make_unique<Tree>(.99F, 1.F, .1F, false);
    auto & tree = *storage;
    Tree::PointVector points, queries;
    points.reserve(count);
    for (int i = 0; i < count; ++i)
      points.emplace_back(float(i % 200) * .1F, float(i / 200 % 200) * .1F, float(i / 40000) * .1F);
    tree.Build(points);
    for (int i = 0; i < 1600; ++i)
      queries.emplace_back(float(i % 193) * .103F, float(i % 197) * .101F, float(i % 31) * .102F);
    for (int repeat = 0; repeat < 10; ++repeat) {
      for (int order = 0; order < 2; ++order) {
        const bool legacy = ((repeat + order) % 2) == 0;
        const auto begin = std::chrono::steady_clock::now();
        tree.Begin_Concurrent_Read(legacy);
        const auto ready = std::chrono::steady_clock::now();
        std::vector<std::thread> workers;
        for (int worker = 0; worker < 3; ++worker) workers.emplace_back([&, worker] {
          Tree::PointVector neighbors; std::vector<float> distances;
          for (std::size_t i = worker; i < queries.size(); i += 3)
            tree.Nearest_Search(queries[i], 5, neighbors, distances, 3.);
        });
        for (auto & thread : workers) thread.join();
        tree.End_Concurrent_Read();
        const auto end = std::chrono::steady_clock::now();
        if (repeat >= 2) std::cout << "READ_BENCH " << count << ' ' << legacy << ' '
          << std::chrono::duration<double, std::milli>(ready - begin).count() << ' '
          << std::chrono::duration<double, std::milli>(end - ready).count() << '\n';
      }
    }
  }
}
