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

#ifndef ITERA_LIO__COMMON__EVALUATION_TYPES_HPP_
#define ITERA_LIO__COMMON__EVALUATION_TYPES_HPP_

#include <cstddef>
#include <cstdint>

namespace itera_lio::common
{

// A point-in-time view of the same state printed by the STATUS/TIMING/MAP
// diagnostics. It is ROS-independent so the output layer can serialize and
// aggregate it without depending on the node implementation.
struct RuntimeSnapshot
{
  double wall_timestamp{0.0};
  double sensor_timestamp{0.0};

  std::uint64_t lidar_received{0U};
  std::uint64_t imu_received{0U};
  std::size_t lidar_queue_size{0U};
  std::size_t imu_queue_size{0U};
  std::uint64_t lidar_processed{0U};
  std::uint64_t imu_processed{0U};
  std::uint64_t lidar_deskewed{0U};
  std::uint64_t lidar_parse_failures{0U};
  std::uint64_t lidar_deskew_failures{0U};
  std::uint64_t lidar_timestamp_regressions{0U};
  std::uint64_t imu_timestamp_regressions{0U};
  std::uint64_t lidar_capacity_drops{0U};
  std::uint64_t imu_capacity_drops{0U};
  std::uint64_t pending_lidar_drops{0U};
  std::uint64_t imu_gap_resets{0U};

  double range_filter_ms{0.0};
  double voxel_ms{0.0};
  double deskew_ms{0.0};
  double registration_ms{0.0};
  double correction_ms{0.0};
  double map_delete_ms{0.0};
  double archive_update_ms{0.0};
  double map_insert_ms{0.0};
  double trajectory_reanchor_ms{0.0};
  double repropagation_ms{0.0};
  double core_ms{0.0};
  double output_ms{0.0};
  double end_to_end_ms{0.0};

  std::size_t local_map_points{0U};
  std::size_t output_global_map_points{0U};
  std::size_t map_candidates{0U};
  std::size_t map_inserted{0U};
  std::size_t map_replaced{0U};
  std::size_t map_rejected{0U};
  std::size_t pending_delete_boxes{0U};
  std::uint64_t rebuild_total{0U};
  std::uint64_t rebuild_synchronous{0U};
  std::uint64_t rebuild_asynchronous{0U};
  std::uint8_t rebuild_last_reason{0U};
  std::size_t rebuild_last_points{0U};
  std::size_t rebuild_max_points{0U};
  double rebuild_last_ms{0.0};
  double rebuild_max_ms{0.0};
  bool rebuild_active{false};

  std::size_t backend_keyframes{0U};
  std::size_t backend_odometry_constraints{0U};
  std::size_t backend_loop_constraints{0U};
  std::size_t loop_worker_queue_depth{0U};
  std::size_t loop_worker_max_queue_depth{0U};
  std::uint64_t loop_worker_submitted{0U};
  std::uint64_t loop_worker_completed{0U};
  std::uint64_t loop_worker_dropped{0U};
  std::uint64_t loop_worker_timed_out{0U};
  std::uint64_t loop_worker_cancelled{0U};
  std::size_t backend_worker_queue_depth{0U};
  std::size_t backend_worker_max_queue_depth{0U};
  std::uint64_t backend_worker_submitted{0U};
  std::uint64_t backend_worker_completed{0U};
  std::uint64_t backend_worker_dropped{0U};
  std::uint64_t backend_worker_timed_out{0U};
  std::uint64_t backend_worker_cancelled{0U};
  double resident_memory_mib{0.0};
  double peak_resident_memory_mib{0.0};
};

}  // namespace itera_lio::common

#endif  // ITERA_LIO__COMMON__EVALUATION_TYPES_HPP_
