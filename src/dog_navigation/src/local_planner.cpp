#include "dog_navigation/local_planner.hpp"
#include "collision_overlay.hpp"
#include "dog_terrain/graph_search.hpp"
#include "frame_edge_cache.hpp"
#include "stage_timer.hpp"
#include "terrain_route_validation.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <pcl/kdtree/kdtree_flann.h>
#include <set>
#include <stdexcept>

namespace dog_navigation {
struct LocalPlanner::Impl {
  std::shared_ptr<const dog_terrain::TerrainGraph> graph;
  Options options;
  pcl::KdTreeFLANN<pcl::PointXYZ> static_tree;
  mutable std::vector<int> cached_ids;
  mutable std::vector<bool> cached_stairs;
  mutable int cached_target = -1;
  mutable std::set<std::pair<int, int>> uncertified_edges;
  mutable std::set<std::pair<int, int>> static_no_routes;
  std::unique_ptr<dog_terrain::GraphSearchWorkspace> search;
  mutable PlanningPerformance performance;
  explicit Impl(std::shared_ptr<const dog_terrain::TerrainGraph> g, Options o)
      : graph(std::move(g)), options(o) {
    o.validate();
    if (!graph)
      throw std::invalid_argument("Missing terrain graph");
    if (o.obstacle_bottom >= graph->config().body_top)
      throw std::invalid_argument("Invalid vertical collision envelope");
    static_tree.setInputCloud(graph->cloud().makeShared());
    search = std::make_unique<dog_terrain::GraphSearchWorkspace>(*graph);
  }
};
LocalPlanner::LocalPlanner(std::shared_ptr<const dog_terrain::TerrainGraph> g,
                           Options o)
    : impl_(std::make_unique<Impl>(std::move(g), o)) {}
LocalPlanner::~LocalPlanner() = default;
PlanningPerformance LocalPlanner::performance() const {
  auto result = impl_->performance;
  const auto counters = impl_->search->counters();
  result.turn_evaluations = counters.turn_evaluations;
  result.turn_cache_hits = counters.turn_cache_hits;
  result.lower_bound_builds = counters.lower_bound_builds;
  result.turn_cache_entries = counters.turn_cache_entries;
  return result;
}
Route LocalPlanner::plan(const Point &start, const Point &goal,
                         const Points &observations,
                         const Pose &map_from_odom) const {
  Route out;
  impl_->performance = {};
  StageTimer total(impl_->performance.total_ms);
  if (!start.allFinite() || !goal.allFinite() || !validPose(map_from_odom)) {
    out.reason = "invalid_pose";
    return out;
  }
  const auto &g = *impl_->graph;
  const auto &o = impl_->options;
  const auto &c = g.config();
  std::vector<int> nearest(1);
  std::vector<float> distance(1);
  const auto classify_start = std::chrono::steady_clock::now();
  for (const auto &raw : observations) {
    if (!raw.allFinite())
      continue;
    const Point p = map_from_odom * raw;
    pcl::PointXYZ query(p.x(), p.y(), p.z());
    // Static surfaces (including stair risers/treads) were checked by the
    // terrain builder. Do not reinterpret them as newly appeared obstacles.
    if (impl_->static_tree.nearestKSearch(query, 1, nearest, distance) == 1 &&
        distance[0] <= o.static_tolerance * o.static_tolerance)
      continue;
    ++out.obstacle_count;
    out.obstacles_map.push_back(p);
  }
  // Live observations use a conservative circumscribed circle, including turns.
  // Never silently fall back to the old 0.22 m radius for rectangular projects.
  const double collision_radius =
      c.body_length > 0 ? std::hypot(c.body_length, c.body_width) * .5
                        : c.body_radius;
  CollisionOverlay overlay(out.obstacles_map, o,
                           collision_radius + c.footprint_padding, c.body_top);
  impl_->performance.classification_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - classify_start)
          .count();
  int source, target;
  try {
    source = g.snap(start);
    target = g.snap(goal);
  } catch (const std::exception &e) {
    out.reason = e.what();
    return out;
  }
  if (g.nodes()[source].component != g.nodes()[target].component) {
    out.reason = "disconnected_terrain";
    return out;
  }
  // Only an exhausted search without novel obstacles proves this persistent
  // failure. Added live obstacles cannot create a connection. Budget-exhausted
  // and observation-dependent failures are never retained here.
  if (impl_->static_no_routes.count({source, target})) {
    out.reason = "blocked_no_route";
    return out;
  }
  if (overlay.risk(g.nodes()[source].point) < 0) {
    out.reason = "start_blocked";
    return out;
  }
  if (overlay.risk(g.nodes()[target].point) < 0) {
    out.reason = "goal_blocked";
    return out;
  }
  bool reused = false;
  FrameEdgeCache frame_edges(g, overlay, out.obstacles_map.empty(),
                             impl_->performance);
  if (impl_->cached_target == target) {
    const auto begin =
        std::find(impl_->cached_ids.begin(), impl_->cached_ids.end(), source);
    if (begin != impl_->cached_ids.end()) {
      const size_t offset = std::distance(impl_->cached_ids.begin(), begin);
      // Keep the incoming edge until the robot has actually passed its next
      // waypoint. Nearest-node snapping can advance half a cell too early.
      const size_t prefix = offset > 0 ? offset - 1 : 0;
      out.ids.assign(impl_->cached_ids.begin() + prefix,
                     impl_->cached_ids.end());
      out.stairs.assign(impl_->cached_stairs.begin() +
                            std::min(prefix, impl_->cached_stairs.size()),
                        impl_->cached_stairs.end());
      reused = true;
      for (size_t i = 1; i < out.ids.size(); ++i) {
        const auto &edges = g.edges()[out.ids[i - 1]];
        const auto edge =
            std::find_if(edges.begin(), edges.end(),
                         [&](const auto &e) { return e.target == out.ids[i]; });
        const double danger = frame_edges.risk(out.ids[i - 1], out.ids[i]);
        if (edge == edges.end() || danger < 0) {
          reused = false;
          break;
        }
        out.cost += edge->cost * (1 + o.avoidance_weight * danger);
      }
    }
  }
  try {
    if (!reused) {
      out.cost = 0;
      bool certified = false;
      for (int attempt = 0; attempt < 8; ++attempt) {
        dog_terrain::GraphRoute route;
        {
          StageTimer search_time(impl_->performance.search_ms);
          route = dog_terrain::searchGraph(
              g, source, target,
              [&](int current, const dog_terrain::Edge &edge) {
                if (impl_->uncertified_edges.count({current, edge.target}))
                  return std::numeric_limits<double>::infinity();
                const double danger = frame_edges.risk(current, edge.target);
                return danger < 0
                           ? std::numeric_limits<double>::infinity()
                           : edge.cost * (1 + o.avoidance_weight * danger);
              },
              impl_->search.get());
        }
        out.ids = route.ids;
        out.stairs = route.stairs;
        out.cost = route.cost;
        std::vector<std::pair<int, int>> invalid;
        {
          StageTimer certificate_time(impl_->performance.certification_ms);
          invalid = uncertifiedForwardEdges(g, out.ids);
        }
        if (invalid.empty()) {
          certified = true;
          break;
        }
        impl_->uncertified_edges.insert(invalid.begin(), invalid.end());
      }
      if (!certified) {
        out.ids.clear();
        out.stairs.clear();
        out.reason = "forward_execution_route_not_certified";
        return out;
      }
      impl_->cached_ids = out.ids;
      impl_->cached_stairs = out.stairs;
      impl_->cached_target = target;
    }
  } catch (const std::invalid_argument &) {
    if (out.obstacles_map.empty() && impl_->static_no_routes.size() < 4096)
      impl_->static_no_routes.insert({source, target});
    out.ids.clear();
    out.stairs.clear();
    out.reason = "blocked_no_route";
    return out;
  }
  for (int id : out.ids)
    out.points.push_back(g.nodes()[id].point);
  // Keep intervening treads in the same flight, like TerrainGraph::plan.
  std::vector<size_t> steps;
  for (size_t i = 0; i < out.stairs.size(); ++i)
    if (out.stairs[i])
      steps.push_back(i);
  for (size_t s = 1; s < steps.size(); ++s) {
    const auto left = steps[s - 1], right = steps[s];
    double gap = 0;
    for (size_t i = left + 1; i < right; ++i)
      gap += (out.points[i + 1] - out.points[i]).norm();
    if ((out.points[left + 1].z() - out.points[left].z()) *
                (out.points[right + 1].z() - out.points[right].z()) >
            0 &&
        gap <= c.max_tread_gap)
      for (size_t i = left; i <= right; ++i)
        out.stairs[i] = true;
  }
  if (!reused)
    impl_->cached_stairs = out.stairs;
  // Include the checked next edge at the horizon; never shortcut or smooth
  // edges.
  double length = 0;
  out.local.push_back(out.points.front());
  for (size_t i = 1; i < out.points.size() && length < o.local_length; ++i) {
    length += (out.points[i] - out.points[i - 1]).norm();
    out.local.push_back(out.points[i]);
  }
  out.valid = true;
  out.reason = "planned";
  return out;
}
} // namespace dog_navigation
