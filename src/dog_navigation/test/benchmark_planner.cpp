#include "benchmark_support.hpp"
#include "dog_navigation/local_planner.hpp"
#include "dog_terrain/terrain_graph.hpp"

namespace dog_navigation::benchmark {
Json successfulPlanningKernels() {
  auto cloud = std::make_shared<dog_terrain::Cloud>();
  for (double x = -2; x < 2; x += .035)
    for (double y = -1.3; y < 1.3; y += .035)
      cloud->push_back(pcl::PointXYZ(x, y, 0));
  dog_terrain::Config config;
  config.body_length = .66;
  config.body_width = .58;
  config.body_top = .5;
  auto graph = std::make_shared<dog_terrain::TerrainGraph>(cloud, config);
  const Point start(-1.5, 0, 0), goal(1.5, 0, 0);
  Points wall;
  for (double y = -1.4; y < 1.4; y += .08)
    for (double z = .2; z < .9; z += .12)
      wall.emplace_back(0, y, z);
  std::vector<double> cold, warm, blocked, recovered;
  for (int i = 0; i < 5; ++i) {
    LocalPlanner planner(graph);
    auto time = Clock::now();
    const auto route = planner.plan(start, goal, {}, Pose::Identity());
    cold.push_back(milliseconds(time));
    if (!route.valid)
      throw std::runtime_error("Positive planner fixture was blocked");
    for (int j = 0; j < 6; ++j) {
      time = Clock::now();
      auto again = planner.plan(start, goal, {}, Pose::Identity());
      warm.push_back(milliseconds(time));
      if (!again.valid || again.ids != route.ids)
        throw std::runtime_error("Warm route changed unexpectedly");
    }
    time = Clock::now();
    auto stop = planner.plan(start, goal, wall, Pose::Identity());
    blocked.push_back(milliseconds(time));
    if (stop.valid || !stop.points.empty())
      throw std::runtime_error("Live wall did not block motion");
    time = Clock::now();
    auto clear = planner.plan(start, goal, {}, Pose::Identity());
    recovered.push_back(milliseconds(time));
    if (!clear.valid)
      throw std::runtime_error(
          "Dynamic blockage was incorrectly retained as static failure");
  }
  return {{"model", "synthetic floor, 0.66 x 0.58 m robot; fresh dynamic wall "
                    "on every trial"},
          {"cold", distribution(cold, "milliseconds")},
          {"warm", distribution(warm, "milliseconds")},
          {"live_wall", distribution(blocked, "milliseconds")},
          {"wall_removed", distribution(recovered, "milliseconds")}};
}
} // namespace dog_navigation::benchmark
