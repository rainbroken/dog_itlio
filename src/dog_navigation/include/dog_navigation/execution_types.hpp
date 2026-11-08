#pragma once
#include "dog_navigation/obstacle_memory.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace dog_navigation {
struct Velocity {
  double x = 0, y = 0, yaw = 0;
};
struct TrackingPose {
  Point ground = Point::Zero();
  double yaw = 0;
  Velocity measured;
};
struct ExecutionOptions {
  double walk_speed = .20, stair_speed = .08, acceleration = .15,
         yaw_rate = .30, yaw_acceleration = .5;
  double stair_yaw_rate = .15, heading_tolerance = .08,
         waypoint_tolerance = .008;
  double goal_xy = .15, goal_z = .12, goal_yaw = .10, max_tracking_error = .12;
  double stopped_speed = .025, settle_time = .4;
  void validate() const;
};
struct TimedSample {
  Point point;
  double yaw = 0, time = 0, speed = 0, acceleration = 0;
  std::string phase;
  double yaw_rate = 0, yaw_acceleration = 0;
};
struct TrackResult {
  Velocity command;
  Point predicted = Point::Zero();
  double predicted_yaw = 0;
  std::string phase = "waiting_route", reason;
  double remaining = 0, error = 0;
  bool done = false, fault = false;
};
inline double angleError(double target, double current) {
  return std::remainder(target - current, 2 * 3.14159265358979323846);
}
inline double speed(const Velocity &v) {
  if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.yaw))
    return std::numeric_limits<double>::infinity();
  return std::max(std::hypot(v.x, v.y), std::abs(v.yaw));
}
} // namespace dog_navigation
