#pragma once
#include "dog_navigation/execution_types.hpp"
#include <functional>

namespace dog_navigation {
class PathTracker {
public:
  explicit PathTracker(ExecutionOptions options = {});
  void setRoute(Points path, std::vector<bool> stairs, const TrackingPose &pose,
                Point requested_goal, double goal_yaw, bool use_yaw);
  // Optional brake-envelope certificate supplied by execution. It constrains
  // the target before smoothing; safety rejection after smoothing stops now.
  TrackResult update(const TrackingPose &pose, double dt,
                     const std::function<bool(const Velocity &)> &command_clear = {});
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
  bool aligning_ = true;
  bool rotation_started_ = false;
  bool continuousAfter(size_t endpoint) const;
  bool readyToRotate(const TrackingPose &pose);
  double entry_settled_ = 0;
};
} // namespace dog_navigation
