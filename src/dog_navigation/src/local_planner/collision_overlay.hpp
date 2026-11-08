#pragma once
#include "dog_navigation/obstacle_memory.hpp"

namespace dog_navigation {
// Geometry-only live collision field; no ROS, terrain search, or map persistence.
class CollisionOverlay {
public:
  CollisionOverlay(const Points &obstacles, Options options, double body_radius, double body_top);
  double risk(const Point &p) const;
  double edgeRisk(const Point &a, const Point &b) const;
private:
  Options options_;
  double radius_, top_;
  static constexpr double bin_ = .25;
  std::map<std::pair<int,int>,Points> obstacles_;
};
}  // namespace dog_navigation
