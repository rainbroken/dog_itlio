#include "executor_node.hpp"
#include "dog_navigation/navigation_motion_settings.hpp"

namespace dog_navigation {
void NavigationExecutor::publishCommand(const TrackResult &r, bool permit) {
  dog_interfaces::msg::NavigationCommand msg;
  msg.header.frame_id = "base_link";
  msg.header.stamp = now();
  msg.task_id = task_id_;
  msg.map_revision = revision_;
  msg.route_generation = route_ ? route_->generation : 0;
  msg.sequence = ++sequence_;
  msg.phase = r.phase;
  msg.permit = permit;
  msg.valid_for_seconds = permit ? .15 : 0;
  if (permit) {
    msg.velocity.linear.x = r.command.x;
    msg.velocity.linear.y = r.command.y;
    msg.velocity.angular.z = r.command.yaw;
  }
  command_pub_->publish(msg);
}
void NavigationExecutor::publishState(const TrackResult &r) {
  if(handle_&&!r.reason.empty()&&r.phase!="aborted"&&r.phase!="cancelled"&&r.phase!="succeeded")last_block_reason_=r.reason;
  const dog_terrain::Json value = {{"task_id", task_id_},
                                   {"phase", r.phase},
                                   {"reason", r.reason},
                                   {"task_state", supervisor_.state()},
                                   {"enabled", enabled_},
                                   {"distance_remaining", r.remaining},
                                   {"tracking_error", r.error},
                                   {"map_project", project_},
                                   {"map_revision", revision_},
                                   {"base_ground_offset",ground_offset_},
                                   {"guard_reason", guard_reason_},
                                   {"physical_traversal_validated", false}};
  auto state = value;
  state["planner_goal_stamp"]={{"sec",goal_.header.stamp.sec},{"nanosec",goal_.header.stamp.nanosec}};
  state["guard_auto_enabled"] = guard_auto_enabled_;
  state["last_block_reason"]=last_block_reason_;
  state["last_guard_block_reason"]=last_guard_block_reason_;
  state["planner_block_reason"]=planner_block_reason_;
  state["motion_limits"] = NavigationMotionSettings::values(options_);
  state["processing_ms"] = (monotonicSeconds()-last_tick_)*1000;
  state["localization_ready"] =
      localization_->ready(std::chrono::steady_clock::now());
  std_msgs::msg::String msg;
  msg.data = state.dump();
  status_pub_->publish(msg);
  if (handle_ && !handle_->is_canceling()) {
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->phase = r.phase;
    feedback->reason = r.reason;
    feedback->distance_remaining = r.remaining;
    feedback->tracking_error = r.error;
    handle_->publish_feedback(feedback);
  }
}
} // namespace dog_navigation
