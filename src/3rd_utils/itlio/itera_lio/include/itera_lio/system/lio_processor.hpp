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

#ifndef ITERA_LIO__SYSTEM__LIO_PROCESSOR_HPP_
#define ITERA_LIO__SYSTEM__LIO_PROCESSOR_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "itera_lio/estimator/imu_initializer.hpp"
#include "itera_lio/estimator/imu_propagator.hpp"
#include "itera_lio/common/frame_metrics.hpp"
#include "itera_lio/sensor/sensor_buffer.hpp"
#include "itera_lio/system/lio_pipeline.hpp"
#include "itera_lio/frontend/ikd_map.hpp"

namespace itera_lio::system
{

struct LioProcessorOptions
{
  estimator::ImuInitializerOptions imu_initialization;
  estimator::ImuNoiseParameters imu_noise;
  // Optional stationary-window covariance, including gravity/bias coupling.
  bool initialize_covariance_from_static_window{false};
  double initial_rotation_stddev{0.001};
  double initial_position_stddev{0.001};
  double initial_velocity_stddev{0.01};
  double initial_accel_bias_stddev{0.1};
  LioPipelineOptions pipeline;
  common::FixedExtrinsics lidar_extrinsics;
  double lidar_min_range{0.5};
  // Keep every Nth input point before range filtering, matching FAST-LIO2 semantics.
  std::size_t point_filter_num{1U};
  bool full_deskewed_enabled{false};
  float deskew_voxel_size{0.2F};
  std::size_t imu_history_max_samples{3000U};
  double imu_history_duration_sec{0.5};
  double max_imu_interval_sec{0.05};
  std::size_t max_pending_lidar{8U};
};

struct LioProcessorStats
{
  std::uint64_t lidar_processed{0U};
  std::uint64_t imu_processed{0U};
  std::uint64_t lidar_deskewed{0U};
  std::uint64_t lidar_deskew_failures{0U};
  std::uint64_t lidar_range_filtered_points{0U};
  std::uint64_t lidar_range_filtered_frames{0U};
  std::uint64_t lidar_pending_drops{0U};
  std::uint64_t lidar_before_initialization{0U};
  std::uint64_t trajectory_repropagations{0U};
  std::uint64_t trajectory_reprop_failures{0U};
  std::uint64_t imu_gap_resets{0U};
  bool last_degenerate{true};
  double last_condition_number{0.0};
  double last_range_filter_time_ms{0.0};
  double last_voxel_time_ms{0.0};
  double last_deskew_time_ms{0.0};
  double last_trajectory_reanchor_time_ms{0.0};
  double last_repropagation_time_ms{0.0};
  double last_core_end_to_end_time_ms{0.0};
  std::size_t last_repropagated_imu_samples{0U};
  std::size_t imu_history_samples{0U};
};

using FrameProcessingTiming = common::FrameProcessingTiming;

struct ProcessedFrame
{
  std::size_t raw_points{0U};
  std::size_t range_filtered_points{0U};
  sensor::LidarFrame deskewed;
  sensor::LidarFrame full_deskewed;
  PipelineResult pipeline;
  estimator::NominalState state_at_scan_end;
  // Paired with state_at_scan_end, before propagating any post-scan IMU samples.
  estimator::ErrorStateVector covariance_diagonal_at_scan_end{estimator::ErrorStateVector::Zero()};
  FrameProcessingTiming timing;
};

struct ProcessorUpdate
{
  bool initialized_now{false};
  bool propagation_failed{false};
  estimator::ImuInitializationStatus initialization_status{
    estimator::ImuInitializationStatus::kCollecting};
  bool frame_ready{false};
  std::vector<ProcessedFrame> frames;
};

class LioProcessor
{
public:
  explicit LioProcessor(const LioProcessorOptions & options);

  ProcessorUpdate process(sensor::BufferedMeasurement measurement);

  const estimator::NominalState & state() const {return state_;}
  const estimator::StateCovariance & covariance() const {return covariance_;}
  const common::FixedExtrinsics & lidarExtrinsics() const {return options_.lidar_extrinsics;}
  const estimator::ImuInitializationStats & initializationStats() const;
  const LioProcessorStats & stats() const {return stats_;}
  const frontend::IkdMap & localMap() const {return pipeline_.map();}
  const backend::BackendManager & backend() const {return pipeline_.backend();}
  bool getOptimizedMapPoints(
    std::vector<common::MapPoint> & points, float voxel_size) const;
  OptimizedLoopRefinementReport refineOptimizedLoops(const std::string & cloud_archive = {});

private:
  struct ImuStateHistoryEntry
  {
    sensor::ImuData imu;
    estimator::NominalState state;
    estimator::StateCovariance covariance{estimator::StateCovariance::Identity()};
  };

  bool processPendingLidar(ProcessedFrame & frame);
  void processReadyPendingLidar(std::vector<ProcessedFrame> & frames);
  void appendImuHistory(const sensor::ImuData & sample);
  static sensor::ImuData interpolateImu(
    const sensor::ImuData & previous,
    const sensor::ImuData & current,
    double timestamp);

  LioProcessorOptions options_;
  estimator::ImuInitializer imu_initializer_;
  estimator::ImuPropagator imu_propagator_;
  LioPipeline pipeline_;
  estimator::NominalState state_;
  estimator::StateCovariance covariance_{estimator::StateCovariance::Identity()};
  sensor::ImuData previous_imu_sample_;
  std::deque<common::ImuPose> imu_trajectory_;
  std::deque<ImuStateHistoryEntry> imu_state_history_;
  std::deque<sensor::LidarFrame> pending_lidar_;
  // Processing is serialized; retain capacity between scans without sharing
  // mutable storage with returned frames or the output thread.
  sensor::LidarFrame range_filtered_buffer_;
  sensor::LidarFrame point_filtered_buffer_;
  std::vector<std::size_t> selected_indices_buffer_;
  bool has_previous_imu_sample_{false};
  LioProcessorStats stats_;
};

}  // namespace itera_lio::system

#endif  // ITERA_LIO__SYSTEM__LIO_PROCESSOR_HPP_
