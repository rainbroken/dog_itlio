// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "archive_pose_correction.hpp"
#include <Eigen/Geometry>
#include <algorithm>
namespace itera_lio::output::detail {
common::Pose archiveCorrection(double timestamp,
                               const std::vector<backend::Keyframe> &raw_keyframes,
                               const std::vector<backend::Keyframe> &optimized_keyframes) {
  const auto upper = std::upper_bound(raw_keyframes.begin(), raw_keyframes.end(), timestamp,
                                      [](const double timestamp, const backend::Keyframe &frame) {
                                        return timestamp < frame.timestamp;
                                      });
  common::Pose raw_pose;
  common::Pose optimized_pose;
  if (upper == raw_keyframes.begin()) {
    raw_pose = raw_keyframes.front().pose_WI;
    optimized_pose = optimized_keyframes.front().pose_WI;
  } else if (upper == raw_keyframes.end()) {
    raw_pose = raw_keyframes.back().pose_WI;
    optimized_pose = optimized_keyframes.back().pose_WI;
  } else {
    const std::size_t right = static_cast<std::size_t>(std::distance(raw_keyframes.begin(), upper));
    const std::size_t left = right - 1U;
    const double alpha =
        std::clamp((timestamp - raw_keyframes[left].timestamp) /
                       (raw_keyframes[right].timestamp - raw_keyframes[left].timestamp),
                   0.0, 1.0);
    const auto interpolate_pose = [left, right,
                                   alpha](const std::vector<backend::Keyframe> &frames) {
      common::Pose pose;
      Eigen::Quaterniond a(frames[left].pose_WI.R_WI);
      Eigen::Quaterniond b(frames[right].pose_WI.R_WI);
      pose.R_WI = a.slerp(alpha, b).toRotationMatrix();
      pose.p_WI = (1.0 - alpha) * frames[left].pose_WI.p_WI + alpha * frames[right].pose_WI.p_WI;
      return pose;
    };
    raw_pose = interpolate_pose(raw_keyframes);
    optimized_pose = interpolate_pose(optimized_keyframes);
  }
  return backend::composePoses(optimized_pose, backend::inversePose(raw_pose));
}
} // namespace itera_lio::output::detail
