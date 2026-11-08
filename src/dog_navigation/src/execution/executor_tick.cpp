#include "dog_navigation/timed_trajectory.hpp"
#include "executor_node.hpp"

namespace dog_navigation {
void NavigationExecutor::tick() {
  const double wall = monotonicSeconds(), dt = wall - last_tick_;
  last_tick_ = wall;
  TrackResult out;
  out.phase = supervisor_.state();
  if (!handle_) {
    publishCommand(out, false);
    publishState(out);
    return;
  }
  if (supervisor_.state() == "cancelled" || handle_->is_canceling()) {
    finish(false, "cancelled_by_user", true);
    return;
  }
  if (supervisor_.paused()) {
    tracker_->stop();
    out.phase = "paused";
    publishCommand(out, false);
    publishState(out);
    return;
  }
  if (wall - guard_at_ < .3 &&
      (guard_reason_ == "manual_priority" ||
       (guard_reason_ == "manual_takeover_requires_new_task" && guard_task_id_ == task_id_) ||
       guard_reason_ == "estop_latched")) {
    supervisor_.abort(guard_reason_);
    finish(false, guard_reason_);
    return;
  }
  if (wall - guard_at_ < .3 && !guard_auto_enabled_) {
    supervisor_.abort("guard_disabled_or_calibration_unconfirmed");
    finish(false, supervisor_.reason());
    return;
  }
  try {
    Pose map_from_odom;
    const auto pose = readPose(map_from_odom);
    const bool robot = wall - robot_at_ < .5 && robot_.connected &&
                       robot_.state_fresh && robot_.current_motion_state == 6 &&
                       robot_.current_gait == 0 && wall - feedback_at_ < .3 &&
                       wall - guard_at_ < .3;
    const bool located = localization_->ready(std::chrono::steady_clock::now());
    const bool progress = (pose.ground - progress_position_).norm() > .01 ||
                          std::abs(angleError(pose.yaw, progress_yaw_)) > .02;
    if (progress) {
      progress_position_ = pose.ground;
      progress_yaw_ = pose.yaw;
    }
    const bool route_ready = route_ && wall < route_deadline_;
    if (!supervisor_.update(wall, located, route_ready, robot, progress)) {
      tracker_->stop();
      out.phase = supervisor_.state();
      out.reason = supervisor_.reason();
      publishCommand(out, false);
      publishState(out);
      if (!supervisor_.active()) {
        finish(false, supervisor_.reason());
        return;
      }
      if (supervisor_.retryDue(wall)) {
        if (!robot && robot_.connected && robot_.state_fresh &&
            walk_->service_is_ready()) {
          auto request =
              std::make_shared<dog_interfaces::srv::SetMotion::Request>();
          request->motion = request->WALK;
          request->confirm_restricted_motion = true;
          walk_->async_send_request(request);
        } else if (!located && relocalize_->service_is_ready())
          relocalize_->async_send_request(
              std::make_shared<std_srvs::srv::Trigger::Request>());
        else if (!route_ready)
          submitPlannerGoal();
      }
      return;
    }
    if (accepted_generation_ != route_->generation || !tracker_->hasRoute()) {
      Points map_path, odom_path;
      for (const auto &p : route_->points) {
        map_path.emplace_back(p.x, p.y, p.z);
        odom_path.push_back(map_from_odom.inverse() * map_path.back());
      }
      const auto &goal = goal_.pose.position;
      Eigen::Quaterniond q(goal_.pose.orientation.w, goal_.pose.orientation.x,
                           goal_.pose.orientation.y, goal_.pose.orientation.z);
      Point heading = Point::UnitX();
      if (use_yaw_)
        heading = q * heading;
      const Point direction = map_from_odom.linear().transpose() * heading;
      tracker_->setRoute(odom_path, route_->stairs, pose,
                         map_from_odom.inverse() *
                             Point(goal.x, goal.y, goal.z),
                         std::atan2(direction.y(), direction.x()), use_yaw_);
      if (installed_ids_ != route_->node_ids) {
        dog_interfaces::msg::TimedTerrainTrajectory msg;
        msg.header.frame_id = "map";
        msg.header.stamp = now();
        msg.task_id = task_id_;
        msg.map_revision = revision_;
        for (const auto &s :
             parameterizeTerrainPath(map_path, route_->stairs, options_)) {
          dog_interfaces::msg::TimedTerrainPoint p;
          p.position.x = s.point.x();
          p.position.y = s.point.y();
          p.position.z = s.point.z();
          p.yaw = s.yaw;
          p.speed = s.speed;
          p.acceleration = s.acceleration;
          p.phase = s.phase;
          p.yaw_rate = s.yaw_rate;
          p.yaw_acceleration = s.yaw_acceleration;
          p.time_from_start.sec = int32_t(s.time);
          p.time_from_start.nanosec =
              uint32_t((s.time - std::floor(s.time)) * 1e9);
          msg.points.push_back(p);
        }
        trajectory_pub_->publish(msg);
        installed_ids_ = route_->node_ids;
      }
      accepted_generation_ = route_->generation;
    }
    out = tracker_->update(pose, dt);
    if (out.fault) {
      supervisor_.abort(out.reason);
      finish(false, out.reason);
      return;
    }
    if (out.done) {
      supervisor_.succeed();
      finish(true, supervisor_.reason());
      return;
    }
    const Point map_position = map_from_odom * pose.ground,
                map_next = map_from_odom * out.predicted;
    const double correction_yaw =
        std::atan2(map_from_odom.linear()(1, 0), map_from_odom.linear()(0, 0));
    if (!graph_->motionClear(map_position, map_next, pose.yaw + correction_yaw,
                             out.predicted_yaw + correction_yaw)) {
      tracker_->stop();
      out.phase = "blocked_footprint";
      out.reason = "actual_or_predicted_footprint_not_certified";
      publishCommand(out, false);
      publishState(out);
      if (wall - last_request_ > 4)
        submitPlannerGoal();
      return;
    }
    // Processing time consumes the command lease; never renew permission for a
    // stale route.
    const bool permit =
        monotonicSeconds() < route_deadline_ && monotonicSeconds() - wall < .1;
    publishCommand(out, permit);
    publishState(out);
  } catch (const std::exception &e) {
    tracker_->stop();
    out.phase = "waiting_safe_inputs";
    out.reason = e.what();
    publishCommand(out, false);
    publishState(out);
    if (!supervisor_.update(wall, false, false, false, false) &&
        !supervisor_.active())
      finish(false, supervisor_.reason());
  }
}
} // namespace dog_navigation
