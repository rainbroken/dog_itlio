#pragma once
#include "dog_navigation/obstacle_memory.hpp"

namespace dog_navigation {
// Prefix of certified edges; includes the edge crossing the requested horizon.
Points localPath(const Points &points, double horizon);
}  // namespace dog_navigation
