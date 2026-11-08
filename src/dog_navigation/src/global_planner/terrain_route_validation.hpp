#pragma once
#include "dog_terrain/terrain_graph.hpp"
#include <utility>
#include <functional>

namespace dog_navigation {
// Checks the forward heading and finer support model used by execution, also
// when a graph was loaded from an older, coarser planning cache.
std::vector<std::pair<int, int>>
uncertifiedForwardEdges(const dog_terrain::TerrainGraph &graph,
                        const std::vector<int> &ids,
                        const std::function<void()> &checkpoint = {});
} // namespace dog_navigation
