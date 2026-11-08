#pragma once
#include "dog_navigation/obstacle_memory.hpp"

namespace dog_navigation {
// Geometry-only live collision field; no ROS, terrain search, or map persistence.
class CollisionOverlay {
public:
  CollisionOverlay(const Points &obstacles, Options options, double body_radius, double body_top,
                   double body_length=0,double body_width=0);
  double risk(const Point &p) const;
  double edgeRisk(const Point &a, const Point &b) const;
  bool motionClear(const Point &a,const Point &b,double first_yaw,double last_yaw) const;
private:
  Options options_;
  double radius_, top_;
  double half_length_=0,half_width_=0;
  static constexpr double bin_ = .25;
  std::map<std::pair<int,int>,Points> obstacles_;
};
}  // namespace dog_navigation
