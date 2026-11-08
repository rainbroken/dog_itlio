#include "dog_navigation/safety_arbiter.hpp"
#include <algorithm>

namespace dog_navigation {
void SafetyArbiter::manualTakeover(const std::string &task) {
  revoked_task_ = task;
}
SafetyOutput SafetyArbiter::update(const SafetyInput &i, double now) {
  SafetyOutput out;
  if (i.estop) {
    out.reason = "estop_latched";
    return out;
  }
  if (!i.robot_ready || !i.feedback_fresh) {
    out.reason = "robot_or_feedback_stale";
    return out;
  }
  if (i.manual_deadline > now) {
    out.command = i.manual;
    out.owner = "manual";
    out.reason = "manual_priority";
  } else {
    if (!i.auto_enabled) {
      out.reason = "autonomous_output_disabled";
      return out;
    }
    if (i.task_id.empty() || i.task_id == revoked_task_) {
      out.reason = "manual_takeover_requires_new_task";
      return out;
    }
    if (i.command_deadline <= now) {
      out.reason = "autonomous_command_expired";
      return out;
    }
    if (!i.localization_ready || !i.route_ready) {
      out.reason = "localization_or_route_invalid";
      return out;
    }
    out.command = i.autonomous;
    out.owner = "autonomous";
    out.reason = "permitted";
  }
  if (!std::isfinite(out.command.x) || !std::isfinite(out.command.y) ||
      !std::isfinite(out.command.yaw)) {
    out.command = {};
    out.owner = "none";
    out.reason = "nonfinite_command";
    return out;
  }
  // The guard supplies the validated startup profile and fresh task limits.
  // A hidden fixed clamp here would disagree with the executor and Web UI.
  if (out.owner == "autonomous") {
    const auto &v=i.autonomous_limits;
    if(!std::isfinite(speed(v))||v.x<=0||v.y<=0||v.yaw<=0||v.x>1||v.y>1||v.yaw>1.5){
      out.command={};out.owner="none";out.reason="invalid_autonomous_limits";return out;
    }
    out.command.x = std::clamp(out.command.x, -v.x, v.x);
    out.command.y = std::clamp(out.command.y, -v.y, v.y);
    const double planar=std::hypot(out.command.x,out.command.y);
    if(planar>v.x){out.command.x*=v.x/planar;out.command.y*=v.x/planar;}
    out.command.yaw = std::clamp(out.command.yaw, -v.yaw, v.yaw);
  }
  return out;
}
} // namespace dog_navigation
