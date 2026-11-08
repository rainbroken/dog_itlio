#include "dog_navigation/timed_trajectory.hpp"
#include <algorithm>
#include <stdexcept>

namespace dog_navigation {
namespace {
struct Profile {
  double length, acceleration, peak, ramp, cruise, duration;
  Profile(double l, double limit, double a) : length(l), acceleration(a) {
    peak = std::min(limit, std::sqrt(l * a));
    ramp = peak / a;
    cruise = std::max(0., (l - peak * ramp) / peak);
    duration = 2 * ramp + cruise;
  }
  std::array<double, 3> at(double t) const {
    if (t < ramp)
      return {.5 * acceleration * t * t, acceleration * t, acceleration};
    if (t < ramp + cruise)
      return {.5 * peak * ramp + peak * (t - ramp), peak, 0};
    const double left = duration - t;
    return {length - .5 * acceleration * left * left, acceleration * left,
            -acceleration};
  }
};
} // namespace
void ExecutionOptions::validate() const {
  for (double value :
       {walk_speed, stair_speed, acceleration, yaw_rate, yaw_acceleration,
        stair_yaw_rate, heading_tolerance, waypoint_tolerance, goal_xy, goal_z,
        goal_yaw, max_tracking_error, stopped_speed, settle_time, continuous_turn_angle})
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument(
          "Execution limits must be finite and positive");
  if(!std::isfinite(lookahead_distance)||lookahead_distance<0||lookahead_distance>1.||
     continuous_turn_angle>1.2||continuous_turn_angle<=heading_tolerance)
    throw std::invalid_argument("Invalid lookahead or continuous turn angle");
  if (stair_speed > walk_speed || stair_yaw_rate > yaw_rate || walk_speed > 1 ||
      yaw_rate > 1.5)
    throw std::invalid_argument("Invalid execution speed hierarchy");
  if(!std::isfinite(startup_linear_speed)||!std::isfinite(startup_yaw_rate)||
     startup_linear_speed<0||startup_yaw_rate<0||startup_linear_speed>stair_speed||startup_yaw_rate>stair_yaw_rate)
    throw std::invalid_argument("Startup thresholds must be nonnegative and no greater than normal/stair limits");
}
std::vector<TimedSample>
parameterizeTerrainPath(const Points &path, const std::vector<bool> &stairs,
                        const ExecutionOptions &o, double period) {
  o.validate();
  if (path.empty() || stairs.size() + 1 != path.size() ||
      !std::isfinite(period) || period <= 0)
    throw std::invalid_argument("Invalid timed terrain route");
  std::vector<TimedSample> result;
  double clock = 0, previous_yaw = 0;
  for (size_t i = 0; i < path.size(); ++i)
    if (!path[i].allFinite())
      throw std::invalid_argument("Nonfinite trajectory point");
  for (size_t i = 1; i < path.size(); ++i) {
    const Point delta = path[i] - path[i - 1];
    const double length = delta.head<2>().norm();
    if (length < 1e-6)
      throw std::invalid_argument("Terrain edge must have horizontal extent");
    const double heading = std::atan2(delta.y(), delta.x());
    if (i > 1) {
      const double angle = angleError(heading, previous_yaw),
                   sign = angle >= 0 ? 1. : -1.;
      if (std::abs(angle) > 1e-8) {
        Profile rotate(std::abs(angle),
                       (stairs[i - 1] || stairs[i - 2]) ? o.stair_yaw_rate
                                                        : o.yaw_rate,
                       o.yaw_acceleration);
        const int count = std::max(1, int(std::ceil(rotate.duration / period)));
        for (int k = 0; k <= count; ++k) {
          const double t = rotate.duration * k / count;
          const auto sample = rotate.at(t);
          TimedSample p{path[i - 1], previous_yaw + sign * sample[0],
                        clock + t,   0,
                        0,           "align"};
          p.yaw_rate = sign * sample[1];
          p.yaw_acceleration = sign * sample[2];
          result.push_back(p);
        }
        clock += rotate.duration + o.settle_time;
      }
    }
    previous_yaw = heading;
    Profile translation(length, stairs[i - 1] ? o.stair_speed : o.walk_speed,
                        o.acceleration);
    const double duration = translation.duration;
    const int count = std::max(1, int(std::ceil(duration / period)));
    for (int k = 0; k <= count; ++k) {
      const double t = duration * k / count;
      const auto sample = translation.at(t);
      result.push_back(
          {path[i - 1] + delta * std::clamp(sample[0] / length, 0., 1.),
           heading, clock + t, sample[1], sample[2],
           stairs[i - 1] ? (delta.z() > 0 ? "stairs_up" : "stairs_down")
                         : "walk"});
    }
    clock += duration + o.settle_time;
  }
  if (result.empty())
    result.push_back({path.front(), 0, 0, 0, 0, "goal"});
  return result;
}
} // namespace dog_navigation
