#include "../navigation_ros.hpp"
#include "dog_navigation/timed_trajectory.hpp"
#include "executor_node.hpp"

namespace dog_navigation {
void NavigationExecutor::submitPlannerGoal() {
  goal_.header.frame_id = "map";
  goal_.header.stamp = now();
  goal_pub_->publish(goal_);
  route_.reset();
  route_deadline_ = 0;
  last_request_ = monotonicSeconds();
}
void NavigationExecutor::receiveRoute(
    dog_interfaces::msg::TerrainRoute::ConstSharedPtr m) {
  if (!handle_ || m->map_revision != revision_ || m->map_project != project_ ||
      !m->has_goal || m->goal_stamp != goal_.header.stamp)
    return;
  const auto &g = goal_.pose.position;
  if ((Point(m->goal.x, m->goal.y, m->goal.z) - Point(g.x, g.y, g.z)).norm() >
      1e-6)
    return;
  const double age = now().seconds() - rclcpp::Time(m->header.stamp).seconds();
  if (age < -.05 || age > .6 || m->header.frame_id != "map" ||
      !std::isfinite(m->valid_for_seconds))
    return;
  if (!m->valid) {
    route_.reset();
    route_deadline_ = 0;
    return;
  }
  if (m->points.empty() || m->points.size() > 5000 ||
      m->stairs.size() + 1 != m->points.size() ||
      m->node_ids.size() != m->points.size())
    return;
  for (size_t i = 0; i < m->points.size(); ++i) {
    const auto id = m->node_ids[i];
    const auto &p = m->points[i];
    if (id < 0 || size_t(id) >= graph_->nodes().size() ||
        (graph_->nodes()[id].point - Point(p.x, p.y, p.z)).norm() > 1e-5)
      return;
  }
  route_ = m;
  route_deadline_ = monotonicSeconds() +
                    std::max(0., std::min(.6, m->valid_for_seconds) - age);
}
TrackingPose NavigationExecutor::readPose(Pose &map_from_odom) {
  const auto map = buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
  const auto body =
      buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
  for (const auto &stamp : {map.header.stamp, body.header.stamp}) {
    const double age = now().seconds() - rclcpp::Time(stamp).seconds();
    if (age < -.05 || age > .3)
      throw std::runtime_error("execution_tf_expired");
  }
  map_from_odom = transform(map.transform);
  const auto odom_from_body = transform(body.transform);
  if (correction_) {
    const Pose old = (*correction_) * odom_from_body,
               current = map_from_odom * odom_from_body;
    if ((old.translation() - current.translation()).norm() > .12 ||
        Eigen::AngleAxisd(old.linear().transpose() * current.linear()).angle() >
            .10) {
      correction_ = map_from_odom;
      tracker_ = std::make_unique<PathTracker>(options_);
      submitPlannerGoal();
      throw std::runtime_error("localization_correction_requires_replan");
    }
  }
  correction_ = map_from_odom;
  TrackingPose pose;
  pose.ground = odom_from_body * Point(0, 0, -ground_offset_);
  pose.yaw =
      std::atan2(odom_from_body.linear()(1, 0), odom_from_body.linear()(0, 0));
  pose.measured = measured_;
  if (!pose.ground.allFinite() || !std::isfinite(speed(measured_)))
    throw std::runtime_error("invalid_execution_feedback");
  return pose;
}
void NavigationExecutor::finish(bool success, const std::string &reason,
                                bool cancelled) {
  TrackResult stopped;
  stopped.phase = success ? "succeeded" : cancelled ? "cancelled" : "aborted";
  stopped.reason = reason;
  publishCommand(stopped, false);
  tracker_->stop();
  if (planner_cancel_->service_is_ready())
    planner_cancel_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
  if (handle_) {
    auto result = std::make_shared<Action::Result>();
    result->success = success;
    result->reason = reason;
    if (success)
      handle_->succeed(result);
    else if (cancelled && handle_->is_canceling())
      handle_->canceled(result);
    else
      handle_->abort(result);
    handle_.reset();
  }
  route_.reset();
  publishState(stopped);
}
} // namespace dog_navigation
