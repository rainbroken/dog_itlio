// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ITERA_LIO__COMMON__FRAME_METRICS_HPP_
#define ITERA_LIO__COMMON__FRAME_METRICS_HPP_

#include <cstddef>

namespace itera_lio::common
{

struct FrameProcessingTiming
{
  double callback_start_steady_sec{0.0};
  double input_callback_ms{0.0};
  double pre_processing_wait_ms{0.0};
  double imu_to_scan_end_time_ms{0.0};
  double range_filter_time_ms{0.0};
  double voxel_time_ms{0.0};
  double deskew_time_ms{0.0};
  double trajectory_reanchor_time_ms{0.0};
  double repropagation_time_ms{0.0};
  double core_end_to_end_time_ms{0.0};
  std::size_t repropagated_imu_samples{0U};
  double post_scan_imu_duration_ms{0.0};
};

// Output-side timing is kept beside processing timing so evaluation and
// diagnostics can consume a stable frame-level contract without depending on
// the ROS node implementation.
struct FrameOutputTiming
{
  double callback_to_output_ms{0.0};
  double callback_to_state_publish_ms{0.0};
  double full_archive_conversion_time_ms{0.0};
  double full_archive_wait_time_ms{0.0};
  double full_archive_write_time_ms{0.0};
  double preparation_time_ms{0.0};
  double serialization_time_ms{0.0};
  double publish_call_time_ms{0.0};
  double odometry_log_time_ms{0.0};
  double output_total_time_ms{0.0};
  double end_to_end_time_ms{0.0};
  double resident_memory_mib{0.0};
  double peak_resident_memory_mib{0.0};
};

}  // namespace itera_lio::common

#endif  // ITERA_LIO__COMMON__FRAME_METRICS_HPP_
