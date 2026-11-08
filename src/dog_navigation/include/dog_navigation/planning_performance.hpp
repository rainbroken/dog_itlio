#pragma once
#include <cstdint>

namespace dog_navigation {
struct PlanningPerformance {
  double total_ms = 0, classification_ms = 0, search_ms = 0,
         certification_ms = 0;
  // Cumulative immutable-map workspace counters, not live-obstacle cache hits.
  uint64_t turn_evaluations = 0, turn_cache_hits = 0, lower_bound_builds = 0;
  uint64_t turn_cache_entries = 0;
  uint64_t dynamic_edge_evaluations = 0, dynamic_edge_cache_hits = 0;
};
} // namespace dog_navigation
