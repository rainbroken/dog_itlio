#include "dog_terrain/graph_search.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <optional>

namespace dog_terrain {
GraphRoute searchGraph(const TerrainGraph &graph, int source, int target, const EdgeCost &weight,
                       GraphSearchWorkspace *workspace) {
  const auto &nodes = graph.nodes();
  if (source < 0 || target < 0 || size_t(source) >= nodes.size() || size_t(target) >= nodes.size() ||
      !nodes[source].valid || !nodes[target].valid)
    throw std::invalid_argument("Invalid graph endpoint");
  if (nodes[source].component != nodes[target].component)
    throw std::invalid_argument("No observed traversable connection between these surfaces/floors");
  if(graph.config().body_length>0) {
    std::optional<GraphSearchWorkspace> temporary;
    if(!workspace){temporary.emplace(graph);workspace=&*temporary;}
    if(!workspace->matches(graph))throw std::invalid_argument("Search workspace belongs to a different graph");
    // Exact obstacle-free graph costs are a much tighter admissible heuristic
    // than Euclidean distance when clearance/stair penalties dominate. Compute
    // on reversed base edges; overlays can only increase these costs.
    const auto &lower=workspace->lowerBounds(target);
    // Incoming direction is part of the state. Endpoint headings are free.
    struct State {int previous,current,parent;double cost;bool stairs;};
    std::vector<State> states{{-1,source,-1,0,false}};
    std::unordered_map<uint64_t,size_t> best;
    auto key=[&](int previous,int current){return uint64_t(previous+1)*nodes.size()+current;};
    using Item=std::pair<double,size_t>;
    std::priority_queue<Item,std::vector<Item>,std::greater<Item>> pending;
    pending.emplace(lower[source],0);
    int finish=-1;
    while(!pending.empty()) {
      const size_t id=pending.top().second;pending.pop();
      const State state=states[id];
      const auto settled=best.find(key(state.previous,state.current));
      if(settled!=best.end()&&states[settled->second].cost<=state.cost)continue;
      // Validate a proposed transition only when its lower-bound priority is
      // reached. Most frontier proposals never need their expensive geometry.
      // A rejected proposal must NOT settle this directed edge: another arrival
      // at its previous node may reach it with a different, feasible turn.
      if(state.parent>=0) {
        const auto &parent=states[state.parent];
        if(!workspace->turnClear(parent.previous,parent.current,state.current))continue;
      }
      best[key(state.previous,state.current)]=id;
      if(state.current==target){finish=int(id);break;}
      for(const auto &edge:graph.edges()[state.current]) {
        const double cost=weight ? weight(state.current,edge) : edge.cost;
        if(std::isinf(cost)&&cost>0)continue;
        if(!std::isfinite(cost)||cost<edge.cost)
          throw std::invalid_argument("Overlay must reject edges or add finite nonnegative cost");
        const double candidate=state.cost+cost;
        const uint64_t k=key(state.current,edge.target);
        const auto existing=best.find(k);
        if(existing!=best.end()&&states[existing->second].cost<=candidate)continue;
        const size_t next=states.size();
        states.push_back({state.current,edge.target,int(id),candidate,edge.stairs});
        pending.emplace(candidate+lower[edge.target],next);
      }
    }
    if(finish<0)throw std::invalid_argument("No footprint-safe route including turns");
    GraphRoute route;route.cost=states[finish].cost;
    for(int id=finish;id>=0;id=states[id].parent) {
      route.ids.push_back(states[id].current);
      if(states[id].parent>=0)route.stairs.push_back(states[id].stairs);
    }
    std::reverse(route.ids.begin(),route.ids.end());
    std::reverse(route.stairs.begin(),route.stairs.end());
    return route;
  }
  std::vector<double> costs(nodes.size(), std::numeric_limits<double>::infinity());
  std::vector<int> parent(nodes.size(), -1);
  std::vector<bool> closed(nodes.size()), parent_step(nodes.size());
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
  costs[source] = 0; queue.emplace(0, source);
  while (!queue.empty()) {
    const int current = queue.top().second; queue.pop();
    if (closed[current]) continue;
    if (current == target) break;
    closed[current] = true;
    for (const auto &edge : graph.edges()[current]) {
      if (closed[edge.target]) continue;
      const double cost = weight ? weight(current, edge) : edge.cost;
      if (std::isinf(cost) && cost > 0) continue;
      if (!std::isfinite(cost) || cost < edge.cost)
        throw std::invalid_argument("Overlay must reject edges or add finite nonnegative cost");
      const double candidate = costs[current] + cost;
      if (candidate < costs[edge.target]) {
        costs[edge.target] = candidate; parent[edge.target] = current; parent_step[edge.target] = edge.stairs;
        queue.emplace(candidate + (nodes[edge.target].point - nodes[target].point).norm(), edge.target);
      }
    }
  }
  if (!std::isfinite(costs[target])) throw std::invalid_argument("No path found");
  GraphRoute route; route.cost = costs[target]; route.ids = {target};
  while (route.ids.back() != source) {
    const int previous = parent[route.ids.back()];
    if (previous < 0) throw std::runtime_error("Invalid graph predecessor");
    route.ids.push_back(previous);
  }
  std::reverse(route.ids.begin(), route.ids.end());
  for (size_t i = 1; i < route.ids.size(); ++i) route.stairs.push_back(parent_step[route.ids[i]]);
  return route;
}
}  // namespace dog_terrain
