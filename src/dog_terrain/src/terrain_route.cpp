#include "dog_terrain/terrain_graph.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include "dog_terrain/graph_search.hpp"

namespace dog_terrain {
namespace {
Json position(const Eigen::Vector3d &p) { return Json::array({p.x(), p.y(), p.z()}); }
}
Json TerrainGraph::plan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal) const {
  const auto route = searchGraph(*this, snap(start), snap(goal));
  const auto &ids = route.ids;
  for (size_t i=1;i<ids.size();++i) {
    if (!bodySweepClear(nodes_[ids[i-1]].point,nodes_[ids[i]].point))
      throw std::invalid_argument("Selected route failed continuous body collision validation");
    if(i+1<ids.size()&&!turnClear(ids[i-1],ids[i],ids[i+1]))
      throw std::invalid_argument("Selected route failed footprint turn validation");
  }
  std::vector<bool> steps;
  std::vector<size_t> step_indices;
  for (size_t i = 1; i < ids.size(); ++i) {
    steps.push_back(route.stairs[i - 1]);
    if (steps.back()) step_indices.push_back(i - 1);
  }
  auto point = [&](size_t i) -> const Eigen::Vector3d & { return nodes_[ids[i]].point; };
  for (size_t s = 1; s < step_indices.size(); ++s) {
    size_t left = step_indices[s - 1], right = step_indices[s];
    double gap = 0;
    for (size_t i = left + 1; i < right; ++i) gap += (point(i + 1) - point(i)).norm();
    if ((point(left + 1).z() - point(left).z()) * (point(right + 1).z() - point(right).z()) > 0 &&
        gap <= config_.max_tread_gap) for (size_t i = left; i <= right; ++i) steps[i] = true;
  }
  Json points = Json::array(), sections = Json::array();
  double length = 0, ascent = 0, descent = 0;
  double min_boundary_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < ids.size(); ++i) {
    points.push_back(position(point(i)));
    min_boundary_distance = std::min(min_boundary_distance, nodes_[ids[i]].boundary_distance);
    if (i) {
      length += (point(i) - point(i - 1)).norm();
      const double dz = point(i).z() - point(i - 1).z();
      ascent += std::max(0.0, dz); descent += std::max(0.0, -dz);
    }
  }
  for (size_t i = 0; i < steps.size(); ++i) {
    std::string mode = steps[i] ? "stairs" : "walk";
    if (!sections.empty() && sections.back()["mode"] == mode) sections.back()["end_index"] = i + 1;
    else sections.push_back({{"mode", mode}, {"start_index", i}, {"end_index", i + 1}});
  }
  for (auto &section : sections) {
    double dz = point(section["end_index"].get<size_t>()).z() - point(section["start_index"].get<size_t>()).z();
    section["direction"] = dz > .03 ? "up" : dz < -.03 ? "down" : "level";
  }
  return {{"revision", revision_}, {"frame_id", "map"}, {"points", points}, {"node_ids", ids},
          {"sections", sections}, {"cost", route.cost}, {"length", length}, {"ascent", ascent},
          {"descent", descent}, {"min_boundary_distance", min_boundary_distance},
          {"collision_check",{{"model",config_.body_length>0 ? "oriented_rectangle_height_filtered" : "continuous_vertical_cylinder"},
                              {"body_length",config_.body_length},{"body_width",config_.body_width},
                              {"turns_checked",config_.body_length>0},{"endpoint_yaw","unconstrained"},
                              {"checked_edges",ids.size()-1},
                              {"radius",(config_.body_length>0 ? std::hypot(config_.body_length,config_.body_width)*.5 :
                                          config_.body_radius)+config_.footprint_padding},
                              {"body_bottom",config_.body_bottom},{"body_top",config_.body_top}}},
          {"preview_only", true}};
}

}  // namespace dog_terrain
