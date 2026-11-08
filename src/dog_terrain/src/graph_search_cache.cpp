#include "dog_terrain/graph_search.hpp"
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace dog_terrain {
struct GraphSearchWorkspace::Impl {
  const TerrainGraph &graph;
  std::vector<std::vector<std::pair<int, double>>> incoming;
  std::vector<double> lower;
  int target = -1;
  std::unordered_map<uint64_t, bool> turns;
  GraphSearchCounters counts;
  explicit Impl(const TerrainGraph &g) : graph(g) {
    if (g.config().body_length > 0) {
      incoming.resize(g.nodes().size());
      for (size_t i = 0; i < g.nodes().size(); ++i)
        for (const auto &edge : g.edges()[i])
          incoming[edge.target].emplace_back(int(i), edge.cost);
    }
  }
};
GraphSearchWorkspace::GraphSearchWorkspace(const TerrainGraph &g)
    : impl_(std::make_unique<Impl>(g)) {}
GraphSearchWorkspace::~GraphSearchWorkspace() = default;
bool GraphSearchWorkspace::matches(const TerrainGraph &g) const {
  return &impl_->graph == &g;
}
const std::vector<double> &GraphSearchWorkspace::lowerBounds(int target, const SearchCheckpoint &checkpoint) {
  auto &s = *impl_;
  if (target < 0 || size_t(target) >= s.graph.nodes().size())
    throw std::invalid_argument("Invalid heuristic target");
  if (s.target == target)
    return s.lower;
  // An interrupted rebuild must never expose a partial heuristic as the
  // cached result for either the previous target or the new one.
  s.target = -1;
  s.lower.assign(s.graph.nodes().size(),
                 std::numeric_limits<double>::infinity());
  using Bound = std::pair<double, int>;
  std::priority_queue<Bound, std::vector<Bound>, std::greater<Bound>> queue;
  s.lower[target] = 0;
  queue.emplace(0, target);
  while (!queue.empty()) {
    if(checkpoint)checkpoint();
    const auto [cost, current] = queue.top();
    queue.pop();
    if (cost > s.lower[current])
      continue;
    for (const auto &[previous, edge_cost] : s.incoming.at(current)) {
      const double candidate = cost + edge_cost;
      if (candidate < s.lower[previous]) {
        s.lower[previous] = candidate;
        queue.emplace(candidate, previous);
      }
    }
  }
  s.target = target;
  ++s.counts.lower_bound_builds;
  return s.lower;
}
bool GraphSearchWorkspace::turnClear(int previous, int current, int next) {
  if (previous < 0)
    return true;
  auto &s = *impl_;
  const uint64_t n = s.graph.nodes().size();
  const uint64_t key =
      (uint64_t(previous + 1) * n + uint64_t(current)) * n + uint64_t(next);
  const auto it = s.turns.find(key);
  if (it != s.turns.end()) {
    ++s.counts.turn_cache_hits;
    return it->second;
  }
  const bool clear = s.graph.turnClear(previous, current, next);
  ++s.counts.turn_evaluations;
  // Bound auxiliary memory even for pathological searches. On saturation the
  // exact geometric check still runs; there is no permissive fallback.
  if (s.turns.size() < 250000)
    s.turns.emplace(key, clear);
  return clear;
}
GraphSearchCounters GraphSearchWorkspace::counters() const {
  auto result = impl_->counts;
  result.turn_cache_entries = impl_->turns.size();
  return result;
}
} // namespace dog_terrain
