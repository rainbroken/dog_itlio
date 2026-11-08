#pragma once
#include "dog_navigation/obstacle_memory.hpp"
#include "dog_navigation/planning_performance.hpp"
#include <functional>

namespace dog_terrain { class TerrainGraph; }
namespace dog_navigation {
struct Route {
  bool valid = false;
  std::string reason;
  std::vector<int> ids;
  Points points;
  std::vector<bool> stairs;
  Points local;
  double cost = 0;
  size_t obstacle_count = 0;
  Points obstacles_map;
};

// Searches prevalidated 3D terrain connections, then permits ordinary-terrain
// chords only after fresh support, body sweep, turn and obstacle validation.
// A live obstacle overlay never invents support or connects unknown floors.
// Historical API name retained: global_planner/terrain_planner.cpp searches the
// full route, using local_planner obstacle overlay and certified path prefix.
class LocalPlanner {
public:
  explicit LocalPlanner(std::shared_ptr<const dog_terrain::TerrainGraph> graph, Options options = {});
  ~LocalPlanner();
  Route plan(const Point &ground_start, const Point &ground_goal,
             const Points &obstacles_odom, const Pose &map_from_odom,
             const std::function<void()> &checkpoint = {}) const;
  PlanningPerformance performance() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace dog_navigation
