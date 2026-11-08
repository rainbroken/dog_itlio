// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ITERA_LIO__LOOP__LOOP_DETECTOR_HPP_
#define ITERA_LIO__LOOP__LOOP_DETECTOR_HPP_

#include <cstddef>
#include <vector>

#include "itera_lio/loop/loop_types.hpp"

namespace itera_lio::loop
{

struct LoopDetectorOptions
{
  std::size_t min_keyframe_separation{30U};
  std::size_t max_candidates{1U};
  std::size_t max_alignment_candidates{20U};
  double min_time_separation_sec{30.0};
  int azimuth_bins{60};
  int radial_bins{20};
  double max_range{80.0};
  double max_descriptor_distance{0.35};
  double min_descriptor_overlap_ratio{0.10};
  double max_odometry_vertical_separation{1.5};
  double max_gravity_angle_rad{0.2};
};

class LoopDetector
{
public:
  explicit LoopDetector(const LoopDetectorOptions & options = {});

  std::vector<float> describe(const KeyframeCloud & cloud) const;
  std::vector<float> ringKey(const std::vector<float> & descriptor) const;
  LoopCandidate compare(
    const KeyframeCloud & current, const KeyframeCloud & past) const;
  std::vector<LoopCandidate> detect(
    const KeyframeCloud & current,
    const std::vector<KeyframeCloud> & history) const;

private:
  LoopDetectorOptions options_;
};

}  // namespace itera_lio::loop

#endif  // ITERA_LIO__LOOP__LOOP_DETECTOR_HPP_
