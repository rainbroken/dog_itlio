#include "benchmark_support.hpp"
#include "dog_navigation/path_tracker.hpp"
#include "dog_navigation/safety_arbiter.hpp"
#include "dog_navigation/timed_trajectory.hpp"

namespace dog_navigation::benchmark {
Json controlKernels(const Points &path, const std::vector<bool> &stairs) {
  PathTracker tracker;
  TrackingPose pose;
  pose.ground = path.front();
  tracker.setRoute(path, stairs, pose, path.back(), 0, false);
  Json result;
  result["trajectory_parameterization"] = kernel(
      [&] { return parameterizeTerrainPath(path, stairs, {}).size(); }, 20, 5);
  result["feedback_tracking"] = kernel([&] {
    auto out = tracker.update(pose, .05);
    pose.ground = out.predicted;
    pose.yaw = out.predicted_yaw;
    pose.measured = out.command;
    if (out.done || out.fault) {
      pose = {};
      pose.ground = path.front();
      tracker = PathTracker();
      tracker.setRoute(path, stairs, pose, path.back(), 0, false);
    }
    return out.command.x;
  });
  SafetyArbiter guard;
  SafetyInput input;
  input.robot_ready = input.feedback_fresh = input.localization_ready =
      input.route_ready = input.auto_enabled = true;
  input.task_id = "benchmark";
  input.command_deadline = 10;
  input.autonomous.x = .1;
  result["safety_arbitration"] =
      kernel([&] { return guard.update(input, 1).command.x; }, 30, 1000);
  return result;
}
} // namespace dog_navigation::benchmark
