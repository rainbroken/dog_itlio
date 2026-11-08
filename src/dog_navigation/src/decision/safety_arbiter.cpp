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
  // Keep existing manual speed settings and driver limits. Autonomous limits
  // remain conservative until braking and tracking are calibrated.
  if (out.owner == "autonomous") {
    out.command.x = std::clamp(out.command.x, -.3, .3);
    out.command.y = std::clamp(out.command.y, -.15, .15);
    out.command.yaw = std::clamp(out.command.yaw, -.5, .5);
  }
  return out;
}
} // namespace dog_navigation
