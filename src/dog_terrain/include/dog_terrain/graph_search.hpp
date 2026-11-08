#pragma once
#include "dog_terrain/terrain_graph.hpp"
#include <functional>
#include <cstdint>
#include <stdexcept>

namespace dog_terrain {
struct GraphRoute {
  std::vector<int> ids;
  std::vector<bool> stairs;
  double cost = 0;
};
// The graph owns support/geometry. Callers may reject an edge with infinity or
// add nonnegative cost, but cannot create connections or reduce stored costs.
using EdgeCost = std::function<double(int source, const Edge &edge)>;
using SearchCheckpoint = std::function<void()>;
using TransitionClear = std::function<bool(int previous,int current,int next)>;
// Budget exhaustion is not proof that a graph has no route.
struct SearchInterrupted : std::runtime_error {
  SearchInterrupted() : std::runtime_error("planning_budget_exceeded") {}
};
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
  const std::vector<double> &lowerBounds(int target, const SearchCheckpoint &checkpoint = {});
  bool turnClear(int previous, int current, int next);
  GraphSearchCounters counters() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
GraphRoute searchGraph(const TerrainGraph &graph, int source, int target,
                       const EdgeCost &cost = {},
                       GraphSearchWorkspace *workspace = nullptr,
                       const SearchCheckpoint &checkpoint = {},
                       const TransitionClear &transition_clear = {},
                       double heuristic_weight = 1.);
// Fast candidate search with edge costs only. The caller MUST certify every
// heading transition before execution; this function alone does not do so.
GraphRoute searchNodeGraph(const TerrainGraph &graph,int source,int target,
                           const EdgeCost &cost = {},GraphSearchWorkspace *workspace = nullptr,
                           const SearchCheckpoint &checkpoint = {},double heuristic_weight = 1.);
} // namespace dog_terrain
