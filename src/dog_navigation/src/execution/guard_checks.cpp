#include "../common/navigation_ros.hpp"
#include "dog_terrain/collision_geometry.hpp"
#include "velocity_guard_node.hpp"

namespace dog_navigation {
bool VelocityGuard::collisionFree(const Velocity &v) {
  if (!graph_ || !route_)
    return false;
  const auto map = buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
  const auto body =
      buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
  if (!fresh(map.header.stamp, .3) || !fresh(body.header.stamp, .3))
    return false;
  const Pose map_body = transform(map.transform) * transform(body.transform);
  const Point a = map_body * Point(0, 0, -offset_);
  const double yaw =
      std::atan2(map_body.linear()(1, 0), map_body.linear()(0, 0));
  // Software braking envelope; physical deceleration must be calibrated before
  // enabling hardware. The controller also reduces speed ahead of every corner.
  const double velocity =
      std::max(std::hypot(v.x, v.y), std::hypot(measured_.x, measured_.y));
  const double distance = velocity * .15 + velocity * velocity / (2 * .15);
  Point b = a + Point((v.x * std::cos(yaw) - v.y * std::sin(yaw)) * .15,
                      (v.x * std::sin(yaw) + v.y * std::cos(yaw)) * .15, 0);
  if (std::hypot(v.x, v.y) > 1e-6) {
    b = a + (b - a).normalized() * distance;
    double best = std::numeric_limits<double>::infinity();
    Point end = a, delta = Point::Zero();
    for (size_t i = 1; i < route_->points.size(); ++i) {
      const auto &p = route_->points[i - 1], &q = route_->points[i];
      const Point first(p.x, p.y, p.z), d(q.x - p.x, q.y - p.y, q.z - p.z);
      const double n = d.head<2>().squaredNorm();
      if (n < 1e-12)
        continue;
      const double t =
          std::clamp((a - first).head<2>().dot(d.head<2>()) / n, 0., 1.);
      const double error = (a - first - t * d).norm();
      if (error < best) {
        best = error;
        end = first + d;
        delta = d;
      }
    }
    if (best > .25 || distance > (end - a).head<2>().norm() + .04)
      return false;
    const double length = delta.head<2>().squaredNorm();
    if (length < 1e-12)
      return false;
    b.z() = end.z() - delta.z() +
            delta.z() *
                std::clamp((b - end + delta).head<2>().dot(delta.head<2>()) /
                               length,
                           0., 1.);
  }
  const double rotation =
      v.yaw * .15 + std::copysign(v.yaw * v.yaw / (2 * .5), v.yaw);
  if (!graph_->motionClear(a, b, yaw, yaw + rotation))
    return false;
  const auto &c = graph_->config();
  const double radius =
      (c.body_length > 0 ? std::hypot(c.body_length, c.body_width) * .5
                         : c.body_radius) +
      c.footprint_padding + .12 + .10 * .866025404;
  for (const auto &p : route_->obstacles)
    if (dog_terrain::sweptCylinderContact(a, b, Point(p.x, p.y, p.z), radius,
                                          .12 - .05, c.body_top + .12 + .05))
      return false;
  return true;
}
void VelocityGuard::tick() {
  const double wall = guardWall();
  input_.robot_ready = wall - status_at_ < .5 && robot_.connected &&
                       robot_.state_fresh && robot_.current_motion_state == 6 &&
                       robot_.current_gait == 0 &&
                       robot_.safety_state != robot_.SAFETY_FAULT;
  input_.feedback_fresh = wall - feedback_at_ < .3;
  input_.localization_ready =
      localization_ && localization_->ready(std::chrono::steady_clock::now());
  input_.route_ready = route_ && wall < route_deadline_ &&
                       route_->generation == last_command_.route_generation;
  auto result = arbiter_.update(input_, wall);
  try {
    if (result.owner == "autonomous" && speed(result.command) > 1e-9 &&
        !collisionFree(result.command)) {
      result.command = {};
      result.owner = "none";
      result.reason = "braking_or_footprint_collision";
    }
  } catch (const std::exception &) {
    result.command = {};
    result.owner = "none";
    result.reason = "safety_tf_unavailable";
  }
  if (guardWall() - wall > .08 ||
      (guardWall() > input_.command_deadline && result.owner == "autonomous")) {
    result.command = {};
    result.owner = "none";
    result.reason = "safety_processing_or_command_expired";
  }
  geometry_msgs::msg::Twist command;
  command.linear.x = result.command.x;
  command.linear.y = result.command.y;
  command.angular.z = result.command.yaw;
  output_->publish(command);
  std_msgs::msg::String state;
  state.data =
      dog_terrain::Json({{"owner", result.owner},
                         {"reason", result.reason},
                         {"auto_enabled", auto_enabled_},
                         {"estop", input_.estop},
                         {"task_id", input_.task_id},
                         {"map_revision", revision_},
                         {"measured_speed", speed(measured_)},
                         {"processing_ms", (guardWall() - wall) * 1000}})
          .dump();
  status_->publish(state);
}
} // namespace dog_navigation
