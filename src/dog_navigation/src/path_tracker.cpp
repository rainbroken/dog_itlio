#include "dog_navigation/path_tracker.hpp"
#include <algorithm>
#include <stdexcept>

namespace dog_navigation {
PathTracker::PathTracker(ExecutionOptions options) : options_(options) {
  options_.validate();
}
void PathTracker::stop() {
  previous_ = {};
  settled_ = 0;
  entered_ = false;
  entry_settled_ = 0;
}
void PathTracker::setRoute(Points path, std::vector<bool> stairs,
                           const TrackingPose &pose, Point goal, double yaw,
                           bool use_yaw) {
  if (path.empty() || stairs.size() + 1 != path.size() || !goal.allFinite() ||
      !std::isfinite(yaw))
    throw std::invalid_argument("Invalid tracker route");
  for (const auto &p : path)
    if (!p.allFinite())
      throw std::invalid_argument("Nonfinite tracker route");
  // Preserve the current target when a rolling replan removes only its prefix.
  size_t target = path.size() > 1 ? 1 : 0;
  if (!path_.empty() && target_ < path_.size()) {
    const auto old = path_[target_];
    for (size_t i = 0; i < path.size(); ++i)
      if ((path[i] - old).norm() < 1e-6) {
        target = i;
        break;
      }
  } else if (path.size() > 1) {
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < path.size(); ++i) {
      const Point d = path[i] - path[i - 1];
      const double t = std::clamp(
          (pose.ground - path[i - 1]).dot(d) / d.squaredNorm(), 0., 1.);
      const double distance = (pose.ground - path[i - 1] - t * d).norm();
      if (distance < best) {
        best = distance;
        target = i;
      }
    }
  }
  path_ = std::move(path);
  stairs_ = std::move(stairs);
  target_ = target;
  goal_ = goal;
  goal_yaw_ = yaw;
  use_yaw_ = use_yaw;
}
TrackResult PathTracker::update(const TrackingPose &pose, double dt) {
  TrackResult out;
  out.predicted = pose.ground;
  out.predicted_yaw = pose.yaw;
  if (!pose.ground.allFinite() || !std::isfinite(pose.yaw) ||
      !std::isfinite(dt) || dt <= 0 || dt > .25) {
    out.fault = true;
    out.reason = "invalid_control_state_or_period";
    stop();
    return out;
  }
  if (path_.empty())
    return out;
  if (!entered_) {
    out.phase =
        !stairs_.empty() && stairs_[0] ? "stair_entry_wait" : "entry_stop";
    entry_settled_ = speed(pose.measured) <= options_.stopped_speed
                         ? entry_settled_ + dt
                         : 0;
    if (entry_settled_ >= options_.settle_time)
      entered_ = true;
    return out;
  }
  const double goal_xy = (pose.ground - goal_).head<2>().norm();
  out.remaining = goal_xy;
  for (size_t i = target_ + 1; i < path_.size(); ++i)
    out.remaining += (path_[i] - path_[i - 1]).head<2>().norm();
  const bool at_goal = goal_xy <= options_.goal_xy &&
                       std::abs(pose.ground.z() - goal_.z()) <= options_.goal_z;
  if (at_goal) {
    if (use_yaw_ &&
        std::abs(angleError(goal_yaw_, pose.yaw)) > options_.goal_yaw) {
      out.phase = "goal_align";
      if (std::hypot(pose.measured.x, pose.measured.y) <=
          options_.stopped_speed)
        out.command.yaw = std::clamp(angleError(goal_yaw_, pose.yaw),
                                     -options_.yaw_rate, options_.yaw_rate);
    } else {
      out.phase = "settling";
      settled_ = speed(pose.measured) <= options_.stopped_speed &&
                         speed(previous_) <= options_.stopped_speed
                     ? settled_ + dt
                     : 0;
      if (settled_ >= options_.settle_time) {
        out.done = true;
        out.phase = "succeeded";
      }
      if (out.done)
        return out;
    }
  } else {
    if (path_.size() == 1) {
      out.fault = true;
      out.reason = "goal_snap_outside_arrival_tolerance";
      stop();
      return out;
    }
    if (target_ == 0)
      target_ = 1;
    const Point a = path_[target_ - 1], b = path_[target_], delta = b - a;
    const double length = delta.head<2>().norm();
    if (length < 1e-6) {
      out.fault = true;
      out.reason = "vertical_or_duplicate_path_edge";
      stop();
      return out;
    }
    const double along =
        (pose.ground - a).head<2>().dot(delta.head<2>()) / (length * length);
    const Point projected = a + delta * std::clamp(along, 0., 1.);
    out.error = (pose.ground - projected).head<2>().norm();
    if (out.error > options_.max_tracking_error ||
        std::abs(pose.ground.z() - projected.z()) > .25) {
      out.fault = true;
      out.reason = "terrain_tracking_error";
      stop();
      return out;
    }
    const double distance = (b - pose.ground).head<2>().norm();
    if (distance <= options_.waypoint_tolerance || along >= 1) {
      out.phase = "waypoint_stop";
      if (speed(pose.measured) <= options_.stopped_speed) {
        settled_ += dt;
        if (settled_ >= options_.settle_time && target_ + 1 < path_.size()) {
          ++target_;
          settled_ = 0;
        }
      } else
        settled_ = 0;
    } else {
      settled_ = 0;
      const bool stair = stairs_[target_ - 1];
      const double desired =
          std::atan2(b.y() - pose.ground.y(), b.x() - pose.ground.x());
      const double error = angleError(desired, pose.yaw);
      const double yaw_limit =
          stair ? options_.stair_yaw_rate : options_.yaw_rate;
      if (std::abs(error) > options_.heading_tolerance) {
        out.phase = stair ? "stair_align" : "align";
        if (std::hypot(pose.measured.x, pose.measured.y) <=
            options_.stopped_speed)
          out.command.yaw = std::clamp(error, -yaw_limit, yaw_limit);
      } else {
        out.phase = stair ? (delta.z() > 0 ? "stairs_up" : "stairs_down")
                    : target_ > 1 && stairs_[target_ - 2] ? "stairs_exit"
                                                          : "walk";
        const double limit = stair ? options_.stair_speed : options_.walk_speed;
        const double reserve = options_.acceleration * .20;
        out.command.x = std::min(
            limit, -reserve + std::sqrt(reserve * reserve +
                                        2 * options_.acceleration * distance));
        // Continue heading feedback during translation to avoid drift and
        // repeated stop/turn cycles at the alignment threshold.
        out.command.yaw = std::clamp(2 * error, -yaw_limit, yaw_limit);
      }
    }
  }
  auto limited = [dt](double desired, double old, double acceleration) {
    return std::clamp(desired, old - acceleration * dt,
                      old + acceleration * dt);
  };
  out.command.x = limited(out.command.x, previous_.x, options_.acceleration);
  out.command.yaw =
      limited(out.command.yaw, previous_.yaw, options_.yaw_acceleration);
  previous_ = out.command;
  out.predicted.x() += out.command.x * std::cos(pose.yaw) * dt;
  out.predicted.y() += out.command.x * std::sin(pose.yaw) * dt;
  if (target_ > 0) {
    const Point a = path_[target_ - 1], d = path_[target_] - a;
    const double horizontal = d.head<2>().squaredNorm();
    if (horizontal > 1e-12)
      out.predicted.z() =
          a.z() +
          d.z() * std::clamp((out.predicted - a).head<2>().dot(d.head<2>()) /
                                 horizontal,
                             0., 1.);
  }
  out.predicted_yaw += out.command.yaw * dt;
  return out;
}
} // namespace dog_navigation
