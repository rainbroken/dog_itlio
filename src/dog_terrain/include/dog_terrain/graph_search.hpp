#pragma once
#include "dog_terrain/terrain_graph.hpp"
#include <functional>
#include <cstdint>

namespace dog_terrain {
struct GraphRoute {
  std::vector<int> ids;
  std::vector<bool> stairs;
  double cost = 0;
};
// The graph owns support/geometry. Callers may reject an edge with infinity or
// add nonnegative cost, but cannot create connections or reduce stored costs.
using EdgeCost = std::function<double(int source, const Edge &edge)>;
struct GraphSearchCounters {
  uint64_t lower_bound_builds = 0, turn_evaluations = 0, turn_cache_hits = 0;
  size_t turn_cache_entries = 0;
};
// Per-planner workspace bound to an immutable graph, which must outlive it.
// No live obstacle weights are retained. Use one workspace per calling thread.
class GraphSearchWorkspace {
public:
  explicit GraphSearchWorkspace(const TerrainGraph &graph);
  ~GraphSearchWorkspace();
  bool matches(const TerrainGraph &graph) const;
  const std::vector<double> &lowerBounds(int target);
  bool turnClear(int previous, int current, int next);
  GraphSearchCounters counters() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
GraphRoute searchGraph(const TerrainGraph &graph, int source, int target,
                       const EdgeCost &cost = {},
                       GraphSearchWorkspace *workspace = nullptr);
} // namespace dog_terrain
