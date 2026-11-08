#include "local_path.hpp"

namespace dog_navigation {
Points localPath(const Points &points, double horizon) {
  if (points.empty()) return {};
  Points local{points.front()};
  double length = 0;
  for (size_t i = 1; i < points.size() && length < horizon; ++i) {
    length += (points[i] - points[i - 1]).norm();
    local.push_back(points[i]);
  }
  return local;
}
}  // namespace dog_navigation
