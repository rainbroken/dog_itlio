// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "itera_lio/frontend/deskewer.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace itera_lio::frontend
{
namespace
{

struct InterpolatedPose
{
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
};

bool interpolatePoseFromHint(
  const double timestamp, const std::vector<ImuPose> & trajectory,
  const std::vector<Eigen::Quaterniond> & rotations, std::size_t & hint,
  InterpolatedPose & pose)
{
  if (trajectory.empty() || trajectory.size() != rotations.size() ||
    timestamp < trajectory.front().timestamp || timestamp > trajectory.back().timestamp)
  {
    return false;
  }
  // Livox points are nearly, but not strictly, ordered by offset_time.
  // Rewind the interval when a later point carries an earlier timestamp.
  while (hint > 0U && trajectory[hint].timestamp > timestamp) {
    --hint;
  }
  while (hint + 1U < trajectory.size() && trajectory[hint + 1U].timestamp < timestamp) {
    ++hint;
  }
  if (hint + 1U >= trajectory.size()) {
    pose.rotation = rotations.back();
    pose.position = trajectory.back().p_WI;
    return pose.rotation.coeffs().allFinite() && pose.position.allFinite();
  }

  const auto & left = trajectory[hint];
  const auto & right = trajectory[hint + 1U];
  const double span = right.timestamp - left.timestamp;
  const double alpha = span > 0.0 ? (timestamp - left.timestamp) / span : 0.0;
  Eigen::Quaterniond right_rotation = rotations[hint + 1U];
  // Quaternions q and -q represent the same rotation. Pick the shorter
  // interpolation arc before the normalized linear interpolation below.
  if (rotations[hint].dot(right_rotation) < 0.0) {
    right_rotation.coeffs() *= -1.0;
  }
  Eigen::Quaterniond interpolated;
  interpolated.coeffs() =
    (1.0 - alpha) * rotations[hint].coeffs() + alpha * right_rotation.coeffs();
  const double norm = interpolated.norm();
  if (!std::isfinite(norm) || norm <= 1e-12) {
    return false;
  }
  interpolated.coeffs() /= norm;
  pose.rotation = interpolated;
  pose.position = (1.0 - alpha) * left.p_WI + alpha * right.p_WI;
  return pose.rotation.coeffs().allFinite() && pose.position.allFinite();
}

}  // namespace

namespace
{

bool deskewLidarFrameImpl(
  const sensor::LidarFrame & input,
  const std::deque<ImuPose> & trajectory,
  const common::FixedExtrinsics & extrinsics,
  const std::vector<std::size_t> * selected_indices,
  sensor::LidarFrame & output,
  sensor::LidarFrame * selected_output, const int num_threads)
{
  output = sensor::LidarFrame{};
  if (selected_output != nullptr) {
    *selected_output = sensor::LidarFrame{};
  }
  if (!input.isValid() || !extrinsics.isValid() || trajectory.size() < 2U || num_threads < 1) {
    return false;
  }
  if (selected_indices != nullptr &&
    (selected_output == nullptr || selected_indices->empty() ||
    selected_indices->back() >= input.points.size() ||
    !std::is_sorted(selected_indices->begin(), selected_indices->end()) ||
    std::adjacent_find(selected_indices->begin(), selected_indices->end()) !=
    selected_indices->end()))
  {
    return false;
  }

  // The node keeps a bounded history so that later scans can be deskewed,
  // but a scan only needs the few poses covering its own time interval. Do
  // not rebuild quaternions for the entire history on every scan.
  const auto first_pose = std::lower_bound(
    trajectory.begin(), trajectory.end(), input.scan_begin_time,
    [](const ImuPose & value, const double time) {return value.timestamp < time;});
  const auto last_pose = std::upper_bound(
    trajectory.begin(), trajectory.end(), input.scan_end_time,
    [](const double time, const ImuPose & value) {return time < value.timestamp;});
  const auto local_begin = first_pose == trajectory.begin() ? first_pose : first_pose - 1;
  const auto local_end = last_pose == trajectory.end() ? last_pose : last_pose + 1;
  const std::vector<ImuPose> local_trajectory(local_begin, local_end);
  if (local_trajectory.size() < 2U ||
    local_trajectory.front().timestamp > input.scan_begin_time ||
    local_trajectory.back().timestamp < input.scan_end_time)
  {
    return false;
  }

  // Construct the trajectory quaternions once per scan. Converting every
  // point's pair of rotation matrices to quaternions and calling SLERP was
  // dominating the deskew time for 20k-point Mid-360 scans. Normalized
  // quaternion interpolation stays on SO(3) and is accurate for the small
  // (about 5 ms) IMU intervals used here, while avoiding per-point matrix
  // construction.
  std::vector<Eigen::Quaterniond> rotations;
  rotations.reserve(local_trajectory.size());
  for (const auto & pose : local_trajectory) {
    if (!pose.R_WI.allFinite() || !pose.p_WI.allFinite()) {
      return false;
    }
    const Eigen::Quaterniond rotation = Eigen::Quaterniond(pose.R_WI).normalized();
    if (!rotation.coeffs().allFinite() || rotation.norm() <= 1e-12) {
      return false;
    }
    rotations.push_back(rotation);
  }

  std::size_t end_hint = 0U;
  InterpolatedPose end_pose;
  if (!interpolatePoseFromHint(
      input.scan_end_time, local_trajectory, rotations, end_hint, end_pose))
  {
    return false;
  }
  output = input;
  if (selected_output != nullptr) {
    selected_output->scan_begin_time = input.scan_begin_time;
    selected_output->scan_end_time = input.scan_end_time;
    selected_output->points.reserve(selected_indices->size());
  }
  // Record the exact serial hint progression first. This preserves which
  // interval is selected at an IMU timestamp, including unordered point times.
  std::vector<std::size_t> hints(output.points.size());
  std::size_t trajectory_hint = 0U;
  for (std::size_t i = 0U; i < output.points.size(); ++i) {
    const double time = input.scan_begin_time + output.points[i].relative_time;
    while (trajectory_hint > 0U && local_trajectory[trajectory_hint].timestamp > time) {
      --trajectory_hint;
    }
    while (trajectory_hint + 1U < local_trajectory.size() &&
      local_trajectory[trajectory_hint + 1U].timestamp < time) {++trajectory_hint;}
    hints[i] = trajectory_hint;
  }
  int invalid_output = 0;
  #pragma omp parallel for schedule(static) num_threads(num_threads) reduction(|:invalid_output) if(num_threads > 1 && output.points.size() >= 4096U)
  for (std::size_t point_index = 0U; point_index < output.points.size(); ++point_index) {
    auto & point = output.points[point_index];
    InterpolatedPose point_pose;
    const double point_time = input.scan_begin_time + point.relative_time;
    auto hint = hints[point_index];
    if (!interpolatePoseFromHint(
        point_time, local_trajectory, rotations, hint, point_pose))
    {
      invalid_output |= 1;
      continue;
    }
    const Eigen::Vector3d p_I = extrinsics.R_IL * point.position + extrinsics.p_IL;
    const Eigen::Vector3d p_W = point_pose.rotation._transformVector(p_I) + point_pose.position;
    const Eigen::Vector3d p_I_end =
      end_pose.rotation.conjugate()._transformVector(p_W - end_pose.position);
    point.position = extrinsics.R_IL.transpose() * (p_I_end - extrinsics.p_IL);
    // Input time/intensity were validated above and remain unchanged. Only
    // transformed coordinates can become invalid; check them in this pass.
    if (!point.position.allFinite()) {invalid_output |= 2;}
  }
  if ((invalid_output & 1) != 0) {
    output = sensor::LidarFrame{};
    return false;
  }
  std::size_t selected_cursor = 0U;
  if (selected_output != nullptr) {
    for (const auto index : *selected_indices) {
      selected_output->points.push_back(output.points[index]);
      ++selected_cursor;
    }
  }
  // Selected output is an exact, nonempty subset of the validated full scan.
  return invalid_output == 0 &&
         (selected_output == nullptr || selected_cursor == selected_indices->size());
}

}  // namespace

bool deskewLidarFrame(
  const sensor::LidarFrame & input,
  const std::deque<ImuPose> & trajectory,
  const common::FixedExtrinsics & extrinsics,
  sensor::LidarFrame & output, const int num_threads)
{
  return deskewLidarFrameImpl(input, trajectory, extrinsics, nullptr, output, nullptr, num_threads);
}

bool deskewLidarFrameAndSelect(
  const sensor::LidarFrame & input,
  const std::vector<std::size_t> & selected_indices,
  const std::deque<ImuPose> & trajectory,
  const common::FixedExtrinsics & extrinsics,
  sensor::LidarFrame & full_output,
  sensor::LidarFrame & selected_output, const int num_threads)
{
  return deskewLidarFrameImpl(
    input, trajectory, extrinsics, &selected_indices, full_output, &selected_output, num_threads);
}

}  // namespace itera_lio::frontend
