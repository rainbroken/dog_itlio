#pragma once
#include "dog_navigation/execution_types.hpp"
#include <functional>

namespace dog_navigation {
class PathTracker {
public:
  explicit PathTracker(ExecutionOptions options = {});
  void setRoute(Points path, std::vector<bool> stairs, const TrackingPose &pose,
                Point requested_goal, double goal_yaw, bool use_yaw);
  TrackResult update(const TrackingPose &pose, double dt);
  void stop();
  bool hasRoute() const { return !path_.empty(); }
  const Points &path() const { return path_; }

private:
  ExecutionOptions options_;
  Points path_;
  std::vector<bool> stairs_;
  size_t target_ = 0;
  Point goal_ = Point::Zero();
  double goal_yaw_ = 0, settled_ = 0;
  bool use_yaw_ = false;
  Velocity previous_;
  bool entered_ = false;
  double entry_settled_ = 0;
};
} // namespace dog_navigation
