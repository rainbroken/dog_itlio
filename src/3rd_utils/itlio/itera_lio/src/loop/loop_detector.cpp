// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "itera_lio/loop/loop_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace itera_lio::loop
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double wrappedYaw(const double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}
}

LoopDetector::LoopDetector(const LoopDetectorOptions & options)
: options_(options)
{
  if (options_.azimuth_bins <= 0 || options_.radial_bins <= 0 ||
    options_.max_candidates == 0U || options_.max_alignment_candidates == 0U ||
    options_.max_alignment_candidates < options_.max_candidates ||
    !std::isfinite(options_.max_range) ||
    options_.max_range <= 0.0 || !std::isfinite(options_.min_time_separation_sec) ||
    options_.min_time_separation_sec <= 0.0 ||
    !std::isfinite(options_.max_descriptor_distance) ||
    options_.max_descriptor_distance <= 0.0 ||
    !std::isfinite(options_.min_descriptor_overlap_ratio) ||
    options_.min_descriptor_overlap_ratio <= 0.0 ||
    options_.min_descriptor_overlap_ratio > 1.0 ||
    !std::isfinite(options_.max_odometry_vertical_separation) ||
    options_.max_odometry_vertical_separation <= 0.0 ||
    !std::isfinite(options_.max_gravity_angle_rad) ||
    options_.max_gravity_angle_rad <= 0.0 || options_.max_gravity_angle_rad > kPi)
  {
    throw std::invalid_argument("invalid loop detector options");
  }
}

std::vector<float> LoopDetector::describe(const KeyframeCloud & cloud) const
{
  const std::size_t bin_count = static_cast<std::size_t>(options_.azimuth_bins) *
    static_cast<std::size_t>(options_.radial_bins);
  std::vector<float> descriptor(bin_count, std::numeric_limits<float>::quiet_NaN());
  if (!cloud.isValid()) {return descriptor;}
  for (const auto & point : cloud.points_I) {
    const double x = point.position.x();
    const double y = point.position.y();
    const double range = std::hypot(x, y);
    if (!std::isfinite(range) || range >= options_.max_range || range <= 1e-6) {continue;}
    double azimuth = std::atan2(y, x);
    if (azimuth < 0.0) {azimuth += 2.0 * kPi;}
    const auto azimuth_bin = static_cast<int>(azimuth / (2.0 * kPi) * options_.azimuth_bins);
    const auto radial_bin = static_cast<int>(range / options_.max_range * options_.radial_bins);
    const int a = std::min(options_.azimuth_bins - 1, std::max(0, azimuth_bin));
    const int r = std::min(options_.radial_bins - 1, std::max(0, radial_bin));
    const auto index = static_cast<std::size_t>(r * options_.azimuth_bins + a);
    if (!std::isfinite(descriptor[index]) || point.position.z() > descriptor[index]) {
      descriptor[index] = point.position.z();
    }
  }
  return descriptor;
}

std::vector<float> LoopDetector::ringKey(const std::vector<float> & descriptor) const
{
  const std::size_t expected_size = static_cast<std::size_t>(options_.azimuth_bins) *
    static_cast<std::size_t>(options_.radial_bins);
  if (descriptor.size() != expected_size) {return {};}
  std::vector<float> key(
    static_cast<std::size_t>(options_.radial_bins),
    std::numeric_limits<float>::quiet_NaN());
  for (int radial = 0; radial < options_.radial_bins; ++radial) {
    double sum = 0.0;
    std::size_t count = 0U;
    for (int azimuth = 0; azimuth < options_.azimuth_bins; ++azimuth) {
      const float value = descriptor[static_cast<std::size_t>(
            radial * options_.azimuth_bins + azimuth)];
      if (!std::isfinite(value)) {continue;}
      sum += value;
      ++count;
    }
    if (count > 0U) {
      key[static_cast<std::size_t>(radial)] =
        static_cast<float>(sum / static_cast<double>(count));
    }
  }
  return key;
}

LoopCandidate LoopDetector::compare(
  const KeyframeCloud & current, const KeyframeCloud & past) const
{
  LoopCandidate result;
  result.current_id = current.keyframe.id;
  result.matched_id = past.keyframe.id;
  result.descriptor_distance = std::numeric_limits<double>::infinity();
  result.odometry_distance =
    (current.keyframe.pose_WI.p_WI - past.keyframe.pose_WI.p_WI).norm();
  result.odometry_vertical_distance = std::abs(
    current.keyframe.pose_WI.p_WI.z() - past.keyframe.pose_WI.p_WI.z());
  result.gravity_angle_rad = std::acos(std::clamp(
      (current.keyframe.pose_WI.R_WI * Eigen::Vector3d::UnitZ()).dot(
        past.keyframe.pose_WI.R_WI * Eigen::Vector3d::UnitZ()), -1.0, 1.0));
  const std::size_t expected_size = static_cast<std::size_t>(options_.azimuth_bins) *
    static_cast<std::size_t>(options_.radial_bins);
  if (current.descriptor.size() != expected_size || past.descriptor.size() != expected_size) {
    return result;
  }

  const std::size_t occupied_current = static_cast<std::size_t>(std::count_if(
      current.descriptor.begin(), current.descriptor.end(),
      [](const float value) {return std::isfinite(value);}));
  const std::size_t occupied_past = static_cast<std::size_t>(std::count_if(
      past.descriptor.begin(), past.descriptor.end(),
      [](const float value) {return std::isfinite(value);}));
  const std::size_t minimum_overlap = static_cast<std::size_t>(std::ceil(
      options_.min_descriptor_overlap_ratio *
      static_cast<double>(std::max<std::size_t>(1U, std::min(occupied_current, occupied_past)))));

  for (int shift = 0; shift < options_.azimuth_bins; ++shift) {
    double absolute_error = 0.0;
    std::size_t overlap = 0U;
    for (int radial = 0; radial < options_.radial_bins; ++radial) {
      for (int azimuth = 0; azimuth < options_.azimuth_bins; ++azimuth) {
        const std::size_t current_index = static_cast<std::size_t>(
          radial * options_.azimuth_bins + azimuth);
        const int past_azimuth = (azimuth + shift) % options_.azimuth_bins;
        const std::size_t past_index = static_cast<std::size_t>(
          radial * options_.azimuth_bins + past_azimuth);
        const float current_height = current.descriptor[current_index];
        const float past_height = past.descriptor[past_index];
        if (!std::isfinite(current_height) || !std::isfinite(past_height)) {continue;}
        absolute_error += std::abs(static_cast<double>(current_height - past_height));
        ++overlap;
      }
    }
    if (overlap < minimum_overlap) {continue;}
    const double distance = absolute_error / static_cast<double>(overlap);
    if (distance < result.descriptor_distance) {
      result.descriptor_distance = distance;
      result.yaw_offset_rad = wrappedYaw(
        static_cast<double>(shift) * 2.0 * kPi /
        static_cast<double>(options_.azimuth_bins));
    }
  }
  return result;
}

std::vector<LoopCandidate> LoopDetector::detect(
  const KeyframeCloud & current, const std::vector<KeyframeCloud> & history) const
{
  std::vector<LoopCandidate> candidates;
  if (!current.isValid()) {return candidates;}
  struct PreliminaryCandidate
  {
    const KeyframeCloud * cloud{nullptr};
    double distance{0.0};
    bool nearby{false};
  };
  std::vector<PreliminaryCandidate> preliminary;
  const auto current_ring_key = current.ring_key.empty() ?
    ringKey(current.descriptor) : current.ring_key;
  for (const auto & past : history) {
    if (!past.keyframe.isValid() || past.descriptor.empty() ||
      current.keyframe.id <= past.keyframe.id ||
      current.keyframe.id - past.keyframe.id < options_.min_keyframe_separation ||
      current.keyframe.timestamp - past.keyframe.timestamp < options_.min_time_separation_sec ||
      std::abs(current.keyframe.pose_WI.p_WI.z() - past.keyframe.pose_WI.p_WI.z()) >
      options_.max_odometry_vertical_separation ||
      std::acos(std::clamp(
        (current.keyframe.pose_WI.R_WI * Eigen::Vector3d::UnitZ()).dot(
          past.keyframe.pose_WI.R_WI * Eigen::Vector3d::UnitZ()), -1.0, 1.0)) >
      options_.max_gravity_angle_rad)
    {continue;}
    const auto past_ring_key = past.ring_key.empty() ?
      ringKey(past.descriptor) : past.ring_key;
    if (past_ring_key.size() != current_ring_key.size()) {continue;}
    double absolute_error = 0.0;
    std::size_t overlap = 0U;
    for (std::size_t radial = 0U; radial < current_ring_key.size(); ++radial) {
      if (!std::isfinite(current_ring_key[radial]) ||
        !std::isfinite(past_ring_key[radial])) {continue;}
      absolute_error += std::abs(
        static_cast<double>(current_ring_key[radial] - past_ring_key[radial]));
      ++overlap;
    }
    if (overlap == 0U) {continue;}
    const double xy_distance =
      (current.keyframe.pose_WI.p_WI - past.keyframe.pose_WI.p_WI).head<2>().norm();
    preliminary.push_back({&past, absolute_error / static_cast<double>(overlap),
      xy_distance <= 3.0});
  }
  std::sort(
    preliminary.begin(), preliminary.end(), [](const auto & lhs, const auto & rhs) {
      if (lhs.nearby != rhs.nearby) {return lhs.nearby;}
      return lhs.distance < rhs.distance;
    });
  if (preliminary.size() > options_.max_alignment_candidates) {
    preliminary.resize(options_.max_alignment_candidates);
  }
  for (const auto & entry : preliminary) {
    const auto candidate = compare(current, *entry.cloud);
    if (!candidate.isValid() ||
      candidate.descriptor_distance > options_.max_descriptor_distance) {continue;}
    candidates.push_back(candidate);
  }
  std::sort(
    candidates.begin(), candidates.end(), [](const auto & lhs, const auto & rhs) {
      const bool left_nearby = lhs.odometry_distance <= 3.0;
      const bool right_nearby = rhs.odometry_distance <= 3.0;
      if (left_nearby != right_nearby) {return left_nearby;}
      return lhs.descriptor_distance < rhs.descriptor_distance;
    });
  if (candidates.size() > options_.max_candidates) {candidates.resize(options_.max_candidates);}
  return candidates;
}

}  // namespace itera_lio::loop
