#include "dog_navigation/local_planner.hpp"
#include "../local_planner/collision_overlay.hpp"
#include "../local_planner/local_path.hpp"
#include "dog_terrain/graph_search.hpp"
#include "frame_edge_cache.hpp"
#include "../common/stage_timer.hpp"
#include "terrain_route_validation.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <pcl/kdtree/kdtree_flann.h>
#include <set>
#include <stdexcept>

namespace dog_navigation {
namespace {
// Ordinary terrain may use a checked chord instead of following every graph
// zigzag. Stair edges, endpoint turns, support and live obstacles remain hard
// constraints. Keep graph-node endpoints for route identity validation.
void shortenWalkRoute(Route &route,const dog_terrain::TerrainGraph &graph,
                      const CollisionOverlay &overlay,const std::function<void()> &checkpoint,
                      int allowance_ms=20) {
  if(route.points.size()<3)return;
  const auto begin=std::chrono::steady_clock::now();
  Points points{route.points.front()};std::vector<int> ids{route.ids.front()};
  std::vector<bool> stairs;size_t current=0;double incoming=0;bool has_incoming=false;
  while(current+1<route.points.size()){
    size_t end=current+1;
    while(end+1<route.points.size()&&end<current+12&&!route.stairs[end-1]&&!route.stairs[end]&&
          (route.points[end+1]-route.points[current]).head<2>().norm()<=2.)++end;
    size_t chosen=current+1;
    if(!route.stairs[current])for(size_t candidate=end;candidate>current+1;--candidate){
      // This is optional refinement: retain the certified graph route once
      // its small work allowance is spent, rather than starving publication.
      if(std::chrono::steady_clock::now()-begin>std::chrono::milliseconds(allowance_ms))break;
      if(checkpoint)checkpoint();
      const Point a=route.points[current],b=route.points[candidate],d=b-a;
      if(d.squaredNorm()<1e-12)continue;
      const double yaw=std::atan2(d.y(),d.x());
      // Keep the clearance-aware search corridor. A collision-free chord can
      // still cut a broad turn into a narrow corner that needs extra stops.
      // One map cell permits graph zigzag removal without discarding the
      // larger detour selected by the search costs.
      bool keeps_corridor=true;
      for(size_t i=current+1;i<candidate;++i){
        const double t=std::clamp((route.points[i]-a).dot(d)/d.squaredNorm(),0.,1.);
        if((route.points[i]-a-t*d).norm()>graph.config().resolution){keeps_corridor=false;break;}
      }
      if(!keeps_corridor)continue;
      // A long chord must not turn several gentle bends into one sharp
      // stop-and-rotate corner. Keep intermediate certified nodes instead.
      if(has_incoming&&std::abs(std::remainder(yaw-incoming,2*M_PI))>1.)continue;
      if(overlay.edgeRisk(a,b)<0||!graph.motionClear(a,b,yaw,yaw))continue;
      if(has_incoming&&(!graph.motionClear(a,a,incoming,yaw)||!overlay.motionClear(a,a,incoming,yaw)))continue;
      if(candidate+1<route.points.size()){
        const Point next=route.points[candidate+1]-b;
        const double next_yaw=std::atan2(next.y(),next.x());
        if(std::abs(std::remainder(next_yaw-yaw,2*M_PI))>1.)continue;
        if(!graph.motionClear(b,b,yaw,next_yaw)||!overlay.motionClear(b,b,yaw,next_yaw))continue;
      }
      chosen=candidate;break;
    }
    const Point d=route.points[chosen]-route.points[current];
    incoming=std::atan2(d.y(),d.x());has_incoming=true;
    points.push_back(route.points[chosen]);ids.push_back(route.ids[chosen]);
    stairs.push_back(route.stairs[current]);current=chosen;
  }
  route.points=std::move(points);route.ids=std::move(ids);route.stairs=std::move(stairs);
}
}
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
                         const Pose &map_from_odom, const std::function<void()> &checkpoint) const {
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
    if(checkpoint)checkpoint();
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
  // Rectangular projects retain their dimensions in both translation and
  // turning live checks; cylinder geometry is only for legacy projects.
  const double collision_radius =
      c.body_length > 0 ? std::hypot(c.body_length, c.body_width) * .5
                        : c.body_radius;
  CollisionOverlay overlay(out.obstacles_map, o,
                           collision_radius + c.footprint_padding, c.body_top,
                           c.body_length>0?c.body_length+2*c.footprint_padding:0.,
                           c.body_width>0?c.body_width+2*c.footprint_padding:0.);
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
  bool reused = false,already_shortened=false;
  FrameEdgeCache frame_edges(g, overlay, out.obstacles_map.empty(),
                             impl_->performance);
  if (impl_->cached_target == target) {
    const auto begin =
        std::find(impl_->cached_ids.begin(), impl_->cached_ids.end(), source);
    size_t prefix=impl_->cached_ids.size();
    if (begin != impl_->cached_ids.end()) {
      const size_t offset=std::distance(impl_->cached_ids.begin(),begin);
      // Keep the incoming edge until the robot has actually passed its next
      // waypoint. Nearest-node snapping can advance half a cell too early.
      prefix=offset>0?offset-1:0;
    }else{
      // A certified chord contains many graph cells which are not endpoints.
      // Reuse it only when the actual 3D pose is close to one of its segments.
      double best=.10*.10;
      for(size_t i=1;i<impl_->cached_ids.size();++i){
        const Point a=g.nodes()[impl_->cached_ids[i-1]].point,d=g.nodes()[impl_->cached_ids[i]].point-a;
        const double n=d.squaredNorm();if(n<1e-12)continue;
        const double t=std::clamp((start-a).dot(d)/n,0.,1.);
        const double distance=(start-a-t*d).squaredNorm();
        if(distance<best){best=distance;prefix=i-1;}
      }
    }
    if(prefix<impl_->cached_ids.size()){
      out.ids.assign(impl_->cached_ids.begin() + prefix,
                     impl_->cached_ids.end());
      out.stairs.assign(impl_->cached_stairs.begin() +
                            std::min(prefix, impl_->cached_stairs.size()),
                        impl_->cached_stairs.end());
      reused = true;
      for (size_t i = 1; i < out.ids.size(); ++i) {
        if(checkpoint)checkpoint();
        const auto &edges = g.edges()[out.ids[i - 1]];
        const auto edge =
            std::find_if(edges.begin(), edges.end(),
                         [&](const auto &e) { return e.target == out.ids[i]; });
        const double danger = frame_edges.risk(out.ids[i - 1], out.ids[i]);
        if (danger < 0 ||
            (i>1&&!frame_edges.turnClear(out.ids[i-2],out.ids[i-1],out.ids[i]))) {
          reused = false;
          break;
        }
        const double base=edge==edges.end()?(g.nodes()[out.ids[i]].point-g.nodes()[out.ids[i-1]].point).norm():edge->cost;
        out.cost += base * (1 + o.avoidance_weight * danger);
      }
    }
  }
  try {
    if (!reused) {
      out.cost = 0;
      bool certified = false;
      std::set<std::pair<int,int>> seed_rejections;
      for (int attempt = 0; attempt < 8; ++attempt) {
        already_shortened=false;
        // Seed with a static candidate before optimizing soft live-clearance
        // costs over the whole graph. As with cached routes, a feasible route
        // is useful immediately; every live edge still must pass validation.
        const bool static_candidate=attempt==0;
        const bool feasibility_candidate=attempt>0&&attempt<7;
        dog_terrain::GraphRoute route;
        {
          StageTimer search_time(impl_->performance.search_ms);
          const auto edge_cost = [&](int current, const dog_terrain::Edge &edge) {
                if (impl_->uncertified_edges.count({current, edge.target}))
                  return std::numeric_limits<double>::infinity();
                if(feasibility_candidate&&seed_rejections.count({current,edge.target}))
                  return std::numeric_limits<double>::infinity();
                const double danger = static_candidate?0.:frame_edges.risk(current, edge.target);
                return danger < 0
                           ? std::numeric_limits<double>::infinity()
                           : edge.cost * (1 + (!static_candidate&&!feasibility_candidate?o.avoidance_weight:0.)*danger);
              };
          try {
            if(feasibility_candidate)
              route=dog_terrain::searchNodeGraph(g,source,target,edge_cost,impl_->search.get(),checkpoint,2.);
            else route=dog_terrain::searchGraph(g,source,target,edge_cost,impl_->search.get(),checkpoint,
                [&](int previous,int current,int next){return static_candidate||frame_edges.turnClear(previous,current,next);},static_candidate?1.:2.);
          } catch(const std::invalid_argument &) {
            // Seed rejections forbid an edge after one bad incoming turn.
            // They are only a shortcut heuristic: another incoming heading
            // can still use that edge. Exhausting this restricted search must
            // not skip the final heading-aware search or report no route.
            if(feasibility_candidate&&!seed_rejections.empty()){attempt=6;continue;}
            throw;
          }
        }
        out.ids = route.ids;
        out.stairs = route.stairs;
        out.cost = route.cost;
        const auto reject_seed=[&](int a){
          const auto at=std::find(route.ids.begin(),route.ids.end(),a);
          if(at!=route.ids.end()&&std::next(at)!=route.ids.end())seed_rejections.emplace(a,*std::next(at));
        };
        // The node-only candidate can contain needless grid turns. Replace
        // them with checked chords BEFORE deciding whether those turns block
        // the route. Never shortcut an ungrouped stair flight here.
        if(feasibility_candidate&&std::none_of(out.stairs.begin(),out.stairs.end(),[](bool stair){return stair;})){
          out.points.clear();for(int id:out.ids)out.points.push_back(g.nodes()[id].point);
          shortenWalkRoute(out,g,overlay,checkpoint,100);
          already_shortened=true;
        }
        {
          bool clear=true;double live_cost=0;
          for(size_t i=1;i<out.ids.size();++i){
            if(checkpoint)checkpoint();
            const int a=out.ids[i-1],b=out.ids[i];
            const double danger=frame_edges.risk(a,b);
            if(danger<0||(i>1&&!frame_edges.turnClear(out.ids[i-2],a,b))){
              if(feasibility_candidate)reject_seed(a);
              clear=false;break;
            }
            const auto &edges=g.edges()[a];
            const auto edge=std::find_if(edges.begin(),edges.end(),[&](const auto &e){return e.target==b;});
            const double base=edge==edges.end()?(g.nodes()[a].point-g.nodes()[b].point).norm():edge->cost;
            live_cost+=base*(1+o.avoidance_weight*danger);
          }
          if(!clear)continue;
          out.cost=live_cost;
        }
        std::vector<std::pair<int, int>> invalid;
        {
          StageTimer certificate_time(impl_->performance.certification_ms);
          invalid = uncertifiedForwardEdges(g, out.ids,checkpoint);
        }
        if (invalid.empty()) {
          certified = true;
          break;
        }
        // An edge-only seed has not validated its incoming headings. A bad
        // seed turn cannot globally ban an otherwise feasible directed edge.
        if(!feasibility_candidate)impl_->uncertified_edges.insert(invalid.begin(), invalid.end());
        else for(const auto &edge:invalid)reject_seed(edge.first);
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
  out.points.clear();
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
  {
    StageTimer certificate_time(impl_->performance.certification_ms);
    // Never compound corridor error by repeatedly simplifying cached chords
    // or applying a second pass to an already shortened seed in this call.
    if(!reused&&!already_shortened)shortenWalkRoute(out,g,overlay,checkpoint);
  }
  // The stored chords have passed the same static and live certificates as
  // this output. Every subsequent frame rechecks their live swept geometry.
  impl_->cached_ids=out.ids;impl_->cached_stairs=out.stairs;impl_->cached_target=target;
  // Include the complete checked next segment at the local horizon.
  out.local = localPath(out.points, o.local_length);
  out.valid = true;
  out.reason = "planned";
  return out;
}
} // namespace dog_navigation
