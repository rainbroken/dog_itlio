#pragma once
#include "dog_navigation/execution_types.hpp"

namespace dog_navigation {
struct SafetyInput {
  bool robot_ready = false, feedback_fresh = false, localization_ready = false,
       route_ready = false;
  bool auto_enabled = false, estop = false;
  double command_deadline = 0, manual_deadline = 0;
  Velocity autonomous, manual;
  Velocity autonomous_limits{.30, autonomous_lateral_speed, .50};
  std::string task_id;
};
struct SafetyOutput {
  Velocity command;
  std::string owner = "none", reason = "waiting_inputs";
};
// Independent arbiter. Manual control latches an autonomous-task revocation;
// expiration never silently resumes that task. Emergency inhibits are
// immediate.
class SafetyArbiter {
public:
  SafetyOutput update(const SafetyInput &input, double now);
  void manualTakeover(const std::string &task);

private:
  std::string revoked_task_;
};
} // namespace dog_navigation
