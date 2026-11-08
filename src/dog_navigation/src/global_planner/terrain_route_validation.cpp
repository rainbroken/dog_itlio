#include "terrain_route_validation.hpp"
#include <cmath>

namespace dog_navigation {
std::vector<std::pair<int, int>>
uncertifiedForwardEdges(const dog_terrain::TerrainGraph &graph,
                        const std::vector<int> &ids) {
  double previous = 0;
  std::vector<std::pair<int, int>> invalid;
  for (size_t i = 1; i < ids.size(); ++i) {
    const auto &a = graph.nodes().at(ids[i - 1]).point,
               &b = graph.nodes().at(ids[i]).point;
    const auto delta = b - a;
    const double yaw = std::atan2(delta.y(), delta.x());
    if (!graph.motionClear(a, b, yaw, yaw) ||
        (i > 1 && !graph.motionClear(a, a, previous, yaw)))
      invalid.emplace_back(ids[i - 1], ids[i]);
    previous = yaw;
  }
  return invalid;
}
} // namespace dog_navigation
