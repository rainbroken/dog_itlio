#include "../src/local_planner/collision_overlay.hpp"
#include "benchmark_support.hpp"
#include "dog_navigation/obstacle_memory.hpp"

namespace dog_navigation::benchmark {
Json perceptionKernels() {
  Points returns;
  for (int x = -40; x <= 40; ++x)
    for (int y = -40; y <= 40; ++y)
      if (x * x + y * y < 1600)
        returns.emplace_back(x * .16, y * .16, -.65);
  ObstacleMemory memory;
  double stamp = 1;
  Json result;
  result["scan_returns"] = returns.size();
  result["obstacle_memory_update"] = kernel(
      [&] {
        memory.update(Point::Zero(), returns, stamp++, Point::Zero());
        return memory.size();
      },
      15, 1);
  Points obstacles;
  for (int i = 0; i < 500; ++i)
    obstacles.emplace_back((i % 25) * .18 - 2, (i / 25) * .18 - 2, .4);
  CollisionOverlay overlay(obstacles, {}, .44, .5);
  result["dynamic_edge_check"] = kernel([&] {
    return overlay.edgeRisk({-3, -3, 0}, {-2.8, -2.8, 0});
  });
  return result;
}
} // namespace dog_navigation::benchmark
