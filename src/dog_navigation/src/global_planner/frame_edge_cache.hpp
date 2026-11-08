#pragma once
#include "../local_planner/collision_overlay.hpp"
#include "dog_navigation/planning_performance.hpp"
#include "dog_terrain/terrain_graph.hpp"
#include <unordered_map>

namespace dog_navigation {
// Lifetime is exactly one plan call. Incoming-direction search states can ask
// about the same directed edge repeatedly, but observations must never be
// reused across scans. Saturation falls back to the full collision query.
class FrameEdgeCache {
public:
  FrameEdgeCache(const dog_terrain::TerrainGraph &graph,
                 const CollisionOverlay &overlay, bool empty,
                 PlanningPerformance &performance)
      : graph_(graph), overlay_(overlay), empty_(empty),
        performance_(performance) {}
  double risk(int source, int target) {
    if (empty_)
      return 0;
    const uint64_t key =
        uint64_t(source) * graph_.nodes().size() + uint64_t(target);
    const auto found = values_.find(key);
    if (found != values_.end()) {
      ++performance_.dynamic_edge_cache_hits;
      return found->second;
    }
    ++performance_.dynamic_edge_evaluations;
    const double value = overlay_.edgeRisk(graph_.nodes()[source].point,
                                           graph_.nodes()[target].point);
    if (values_.size() < 250000)
      values_.emplace(key, value);
    return value;
  }

private:
  const dog_terrain::TerrainGraph &graph_;
  const CollisionOverlay &overlay_;
  bool empty_;
  PlanningPerformance &performance_;
  std::unordered_map<uint64_t, double> values_;
};
} // namespace dog_navigation
