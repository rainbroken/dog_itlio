#include "dog_terrain/terrain_graph.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace dog_terrain {
namespace {
Eigen::Vector3d xyz(const pcl::PointXYZ &p) { return {p.x, p.y, p.z}; }
Json position(const Eigen::Vector3d &p) { return Json::array({p.x(), p.y(), p.z()}); }
}
Json TerrainGraph::summary() const {
  size_t valid = 0, edges = 0, stairs = 0;
  int components = 0;
  Eigen::Vector3d low = xyz(cloud_->front()), high = low;
  for (const auto &p : *cloud_) { low = low.cwiseMin(xyz(p)); high = high.cwiseMax(xyz(p)); }
  for (size_t i = 0; i < nodes_.size(); ++i) {
    valid += nodes_[i].valid; components = std::max(components, nodes_[i].component + 1);
    edges += edges_[i].size();
    for (const auto &edge : edges_[i]) stairs += edge.stairs;
  }
  return {{"revision", revision_}, {"frame_id", "map"}, {"config", config_.json()}, {"points", cloud_->size()},
          {"surfaces", nodes_.size()}, {"valid_surfaces", valid}, {"edges", edges / 2},
          {"components", components}, {"step_edges", stairs / 2}, {"bounds", {position(low), position(high)}}};
}

Json TerrainGraph::cache() const {
  Json nodes = Json::array(), edges = Json::array();
  for (size_t i = 0; i < nodes_.size(); ++i) {
    const auto &node = nodes_[i];
    nodes.push_back({node.point.x(), node.point.y(), node.point.z(), node.slope, node.roughness,
                     node.valid, node.component, node.boundary_distance, node.support_valid, node.body_clear});
    for (const auto &edge : edges_[i]) edges.push_back({i, edge.target, edge.cost, edge.stairs});
  }
  Json support=Json::array();
  for(const auto &[key,ids]:support_cells_)for(int id:ids)support.push_back(id);
  return {{"algorithm_version", 5}, {"support_points",support}, {"config", config_.json()}, {"revision", revision_},
          {"nodes", nodes}, {"edges", edges}};
}

std::shared_ptr<TerrainGraph> TerrainGraph::fromCache(Cloud::Ptr cloud, const Json &cache) {
  const int version = cache.at("algorithm_version").get<int>();
  if (version < 1 || version > 5) throw std::invalid_argument("Unsupported navigation cache version");
  if (version < 3) {
    // Old caches certified sampled body poses only. Rebuild in memory from the
    // verified source cloud; preserve the user's original project on disk.
    return std::make_shared<TerrainGraph>(std::move(cloud),Config::parse(cache.at("config")));
  }
  auto graph = std::shared_ptr<TerrainGraph>(new TerrainGraph());
  graph->cloud_ = std::move(cloud); graph->config_ = Config::parse(cache.at("config"));
  graph->revision_ = cache.at("revision").get<std::string>();
  const auto &nodes = cache.at("nodes");
  if (nodes.size() > static_cast<size_t>(graph->config_.max_nodes)) throw std::invalid_argument("Cache node limit exceeded");
  for (const auto &n : nodes) {
    Node node{{n.at(0).get<double>(), n.at(1).get<double>(), n.at(2).get<double>()},
              n.at(3).get<double>(), n.at(4).get<double>(), n.at(5).get<bool>(), n.at(6).get<int>(),
              n.at(7).get<double>(),n.at(8).get<bool>(),n.at(9).get<bool>()};
    if (!node.point.allFinite() || !std::isfinite(node.slope) || !std::isfinite(node.roughness) ||
        !std::isfinite(node.boundary_distance) || node.boundary_distance < 0 ||
        (node.valid && !(node.support_valid && node.body_clear)) ||
        (graph->config_.body_length==0 && node.valid != (node.support_valid && node.body_clear)))
      throw std::invalid_argument("Invalid cached surface");
    graph->nodes_.push_back(node);
  }
  graph->edges_.resize(nodes.size());
  for (const auto &e : cache.at("edges")) {
    auto source = e.at(0).get<size_t>(), target = e.at(1).get<size_t>();
    double cost = e.at(2).get<double>();
    if (source >= nodes.size() || target >= nodes.size() || !std::isfinite(cost) || cost <= 0)
      throw std::invalid_argument("Invalid cached edge");
    graph->edges_[source].push_back({static_cast<int>(target), cost, e.at(3).get<bool>()});
  }
  graph->indexNodes();
  graph->indexRawPoints();
  if(graph->config_.body_length>0 && version<4)
    return std::make_shared<TerrainGraph>(graph->cloud_,graph->config_);
  if(version>=4)for(const auto &entry:cache.at("support_points")) {
    const int id=entry.get<int>();
    if(id<0||size_t(id)>=graph->cloud_->size())throw std::invalid_argument("Invalid support point index");
    graph->support_cells_[graph->cell(xyz((*graph->cloud_)[id]))].push_back(id);
  }
  return graph;
}
}  // namespace dog_terrain
