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
}
TerrainGraph::TerrainGraph(Cloud::Ptr cloud, Config config) : config_(config), cloud_(std::move(cloud)) {
  config_.validate();
  if (!cloud_ || cloud_->size() < 3 || cloud_->size() > 8000000)
    throw std::invalid_argument("Map must have 3..8000000 points");
  for (const auto &p : *cloud_)
    if (!xyz(p).allFinite() || xyz(p).cwiseAbs().maxCoeff() > 100000)
      throw std::invalid_argument("Map requires finite local coordinates in meters");
  build();
  revision_ = sha256(cache().dump()).substr(0, 20);
}

Cell TerrainGraph::cell(const Eigen::Vector3d &p) const {
  return {static_cast<int>(std::floor(p.x() / config_.resolution)),
          static_cast<int>(std::floor(p.y() / config_.resolution))};
}

void TerrainGraph::indexNodes() {
  cells_.clear();
  for (size_t i = 0; i < nodes_.size(); ++i) cells_[cell(nodes_[i].point)].push_back(static_cast<int>(i));
}

void TerrainGraph::indexRawPoints() {
  raw_cells_.clear();
  for (size_t i = 0; i < cloud_->size(); ++i)
    raw_cells_[cell(xyz((*cloud_)[i]))].push_back(static_cast<int>(i));
}

void TerrainGraph::applyClearanceCosts() {
  // A multi-source graph distance transform. Propagating on verified surface
  // edges avoids treating another floor, or a neighboring stair flight, as an
  // obstacle at the same XY. Missing transitions represent walls, drops or unknowns.
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
  for (size_t i = 0; i < nodes_.size(); ++i) {
    auto &node = nodes_[i];
    node.boundary_distance = 0;
    if (!node.valid) continue;
    std::array<bool, 9> directions{};
    const Cell current = cell(node.point);
    for (const auto &edge : edges_[i]) {
      const Cell neighbor = cell(nodes_[edge.target].point);
      int dx = neighbor.first - current.first, dy = neighbor.second - current.second;
      if (std::abs(dx) <= 1 && std::abs(dy) <= 1) directions[(dx + 1) * 3 + dy + 1] = true;
    }
    directions[4] = true;
    bool boundary = std::find(directions.begin(), directions.end(), false) != directions.end();
    node.boundary_distance = boundary ? config_.resolution * .5 : std::numeric_limits<double>::infinity();
    if (boundary) queue.emplace(node.boundary_distance, static_cast<int>(i));
  }
  while (!queue.empty()) {
    const auto [distance, current] = queue.top(); queue.pop();
    if (distance > nodes_[current].boundary_distance) continue;
    for (const auto &edge : edges_[current]) {
      double candidate = distance + (nodes_[edge.target].point - nodes_[current].point).norm();
      if (candidate < nodes_[edge.target].boundary_distance) {
        nodes_[edge.target].boundary_distance = candidate;
        queue.emplace(candidate, edge.target);
      }
    }
  }
  auto penalty = [&](const Node &node) {
    // Continuous decay keeps a preference for the middle even in narrow passages;
    // this is a soft cost, so feasible narrow stairs remain reachable.
    return std::pow(config_.clearance_distance /
                    (config_.clearance_distance + node.boundary_distance), 2);
  };
  for (size_t i = 0; i < nodes_.size(); ++i) {
    if (nodes_[i].valid && !std::isfinite(nodes_[i].boundary_distance))
      throw std::invalid_argument("Surface component has no observed boundary");
    // Like obstacle inflation, combine a soft clearance preference with hard
    // body rejection. Only geometry in THIS surface's height envelope counts.
    if (nodes_[i].valid) nodes_[i].boundary_distance = std::min(
        nodes_[i].boundary_distance, obstacleClearance(nodes_[i].point));
  }
  // Update every node's distance before computing symmetric edge costs.
  for (size_t i = 0; i < nodes_.size(); ++i) {
    for (auto &edge : edges_[i]) {
      const double base = (nodes_[edge.target].point - nodes_[i].point).norm();
      edge.cost = base * (1 + (edge.stairs ? config_.step_penalty : 0) +
          config_.clearance_weight * .5 * (penalty(nodes_[i]) + penalty(nodes_[edge.target])));
    }
  }
}

int TerrainGraph::snap(const Eigen::Vector3d &p) const {
  if (!p.allFinite() || p.cwiseAbs().maxCoeff() > 100000) throw std::invalid_argument("Endpoint must be finite local XYZ");
  int result = -1;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < nodes_.size(); ++i) {
    const auto &node = nodes_[i]; const Eigen::Vector3d delta = node.point - p;
    if (node.valid && delta.head<2>().norm() <= config_.snap_xy && std::abs(delta.z()) <= config_.snap_z &&
        delta.squaredNorm() < best) { best = delta.squaredNorm(); result = static_cast<int>(i); }
  }
  if (result < 0) throw std::invalid_argument("Endpoint has no supported surface on selected height; pick a walkable floor");
  return result;
}

}  // namespace dog_terrain
