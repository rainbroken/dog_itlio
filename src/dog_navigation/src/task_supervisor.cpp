#include "dog_navigation/task_supervisor.hpp"

namespace dog_navigation {
bool TaskSupervisor::active() const {
  return state_ != "idle" && state_ != "succeeded" && state_ != "aborted" &&
         state_ != "cancelled";
}
void TaskSupervisor::start(double now) {
  state_ = "waiting_inputs";
  reason_.clear();
  started_ = last_progress_ = now;
  blocked_since_ = -1;
  last_retry_ = -100;
  retries_ = 0;
}
void TaskSupervisor::pause() {
  if (active())
    state_ = "paused";
}
void TaskSupervisor::resume(double now) {
  if (paused())
    start(now);
}
void TaskSupervisor::cancel() {
  state_ = "cancelled";
  reason_ = "cancelled_by_user";
}
void TaskSupervisor::abort(const std::string &reason) {
  state_ = "aborted";
  reason_ = reason;
}
void TaskSupervisor::succeed() {
  state_ = "succeeded";
  reason_ = "goal_pose_and_stop_confirmed";
}
bool TaskSupervisor::update(double now, bool localization, bool route,
                            bool robot, bool progress) {
  if (!active() || paused())
    return false;
  if (progress)
    last_progress_ = now;
  if (!robot || !localization || !route) {
    state_ = !robot          ? "waiting_walk_state"
             : !localization ? "relocalizing"
                             : "waiting_route";
    reason_ = state_;
    if (blocked_since_ < 0)
      blocked_since_ = now;
    if (now - blocked_since_ > 20)
      abort("input_or_recovery_timeout");
    return false;
  }
  blocked_since_ = -1;
  retries_ = 0;
  state_ = "following";
  reason_.clear();
  if (now - last_progress_ > 30) {
    abort("no_measured_progress");
    return false;
  }
  return true;
}
bool TaskSupervisor::retryDue(double now) {
  if (!active() || paused() || retries_ >= 3 || now - last_retry_ < 4)
    return false;
  ++retries_;
  last_retry_ = now;
  return true;
}
} // namespace dog_navigation
