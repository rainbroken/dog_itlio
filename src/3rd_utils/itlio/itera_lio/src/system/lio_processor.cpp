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

#include "itera_lio/system/lio_processor.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <utility>

#include "itera_lio/common/so3.hpp"
#include "itera_lio/frontend/deskewer.hpp"
#include "itera_lio/frontend/lidar_preprocessor.hpp"
#include "itera_lio/system/imu_trajectory.hpp"

namespace itera_lio::system
{

LioProcessor::LioProcessor(const LioProcessorOptions & options)
: options_(options), imu_initializer_(options.imu_initialization),
  imu_propagator_(options.imu_noise), pipeline_(options.pipeline)
{
  if (!options_.lidar_extrinsics.isValid() ||
    !std::isfinite(options_.lidar_min_range) || options_.lidar_min_range < 0.0 ||
    options_.point_filter_num == 0U ||
    !std::isfinite(options_.initial_rotation_stddev) || options_.initial_rotation_stddev <= 0.0 ||
    !std::isfinite(options_.initial_position_stddev) || options_.initial_position_stddev <= 0.0 ||
    !std::isfinite(options_.initial_velocity_stddev) || options_.initial_velocity_stddev <= 0.0 ||
    !std::isfinite(options_.initial_accel_bias_stddev) || options_.initial_accel_bias_stddev <= 0.0 ||
    !std::isfinite(options_.deskew_voxel_size) || options_.deskew_voxel_size <= 0.0F ||
    options_.imu_history_max_samples < 2U ||
    !std::isfinite(options_.imu_history_duration_sec) ||
    options_.imu_history_duration_sec <= 0.0 ||
    !std::isfinite(options_.max_imu_interval_sec) || options_.max_imu_interval_sec <= 0.0 ||
    options_.max_pending_lidar == 0U)
  {
    throw std::invalid_argument("invalid LIO processor options");
  }
}

ProcessorUpdate LioProcessor::process(sensor::BufferedMeasurement measurement)
{
  ProcessorUpdate update;
  const auto process_start = std::chrono::steady_clock::now();
  if (measurement.type == sensor::MeasurementType::kLidar) {
    ++stats_.lidar_processed;
    if (!imu_initializer_.isInitialized()) {
      ++stats_.lidar_before_initialization;
      return update;
    }
    auto insert_position = pending_lidar_.end();
    while (insert_position != pending_lidar_.begin() &&
      std::prev(insert_position)->scan_begin_time > measurement.lidar.scan_begin_time)
    {
      --insert_position;
    }
    pending_lidar_.insert(insert_position, std::move(measurement.lidar));
    while (pending_lidar_.size() > options_.max_pending_lidar) {
      pending_lidar_.pop_front();
      ++stats_.lidar_pending_drops;
    }
    if (has_previous_imu_sample_) {
      processReadyPendingLidar(update.frames);
    }
    update.frame_ready = !update.frames.empty();
    if (update.frame_ready) {
      const double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - process_start).count();
      for (auto & frame : update.frames) {
        frame.timing.core_end_to_end_time_ms = elapsed;
      }
      stats_.last_core_end_to_end_time_ms = elapsed;
    }
    return update;
  }

  ++stats_.imu_processed;
  const sensor::ImuData & imu_sample = measurement.imu;
  const bool was_initialized = imu_initializer_.isInitialized();
  update.initialization_status = imu_initializer_.addSample(imu_sample);
  if (!was_initialized &&
    update.initialization_status == estimator::ImuInitializationStatus::kInitialized)
  {
    update.initialized_now = true;
    state_ = imu_initializer_.initializedState();
    covariance_.setIdentity();
    if (options_.initialize_covariance_from_static_window &&
      !options_.imu_initialization.initialize_accel_bias_from_static_mean)
    {
      const auto & stats = imu_initializer_.stats();
      using Layout = estimator::ErrorStateLayout;
      const auto variance = [](double value) {return value * value;};
      covariance_.setZero();
      covariance_.block<3, 3>(Layout::kRotation, Layout::kRotation).diagonal().setConstant(
        variance(options_.initial_rotation_stddev));
      covariance_.block<3, 3>(Layout::kPosition, Layout::kPosition).diagonal().setConstant(
        variance(options_.initial_position_stddev));
      covariance_.block<3, 3>(Layout::kVelocity, Layout::kVelocity).diagonal().setConstant(
        variance(options_.initial_velocity_stddev));
      covariance_.block<3, 3>(Layout::kAccelerometerBias,
        Layout::kAccelerometerBias).diagonal().setConstant(
        variance(options_.initial_accel_bias_stddev));
      const double count = static_cast<double>(stats.sample_count);
      const double duration = stats.duration_sec;
      const Eigen::Vector3d gyro_variance = (stats.gyro_stddev.array().square() / count).
        max(variance(options_.imu_noise.gyro_noise_density) / duration).max(1e-12);
      const Eigen::Vector3d accel_variance = (stats.accel_stddev.array().square() / count).
        max(variance(options_.imu_noise.accel_noise_density) / duration).max(1e-12);
      covariance_.block<3, 3>(Layout::kGyroscopeBias, Layout::kGyroscopeBias) =
        gyro_variance.asDiagonal();
      covariance_.block<3, 3>(Layout::kGravity, Layout::kGravity) =
        state_.R_WI * accel_variance.asDiagonal() * state_.R_WI.transpose();
      // Static dynamics: -R Hat(a) dtheta - R db_a + dg = 0.
      // A single static pose cannot independently determine b_a and g.
      // Preserve this correlation instead of assigning independent unit
      // variance to every state. No gravity magnitude or floor is prescribed.
      estimator::StateCovariance transform = estimator::StateCovariance::Identity();
      transform.block<3, 3>(Layout::kGravity, Layout::kRotation) =
        state_.R_WI * common::Hat(stats.mean_accel);
      transform.block<3, 3>(Layout::kGravity, Layout::kAccelerometerBias) = state_.R_WI;
      covariance_ = transform * covariance_ * transform.transpose();
    }
    previous_imu_sample_ = imu_sample;
    has_previous_imu_sample_ = true;
    appendImuHistory(imu_sample);
    return update;
  }
  if (!imu_initializer_.isInitialized()) {
    return update;
  }
  if (!has_previous_imu_sample_) {
    previous_imu_sample_ = imu_sample;
    has_previous_imu_sample_ = true;
    return update;
  }
  const double imu_interval = imu_sample.timestamp - previous_imu_sample_.timestamp;
  if (std::isfinite(imu_interval) && imu_interval > options_.max_imu_interval_sec) {
    ++stats_.imu_gap_resets;
    state_.v_WI.setZero();
    previous_imu_sample_ = imu_sample;
    imu_trajectory_.clear();
    imu_state_history_.clear();
    appendImuHistory(imu_sample);
    return update;
  }
  if (!imu_propagator_.propagate(previous_imu_sample_, imu_sample, state_, covariance_)) {
    update.propagation_failed = true;
    return update;
  }
  previous_imu_sample_ = imu_sample;
  appendImuHistory(imu_sample);
  processReadyPendingLidar(update.frames);
  update.frame_ready = !update.frames.empty();
  if (update.frame_ready) {
    const double elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - process_start).count();
    for (auto & frame : update.frames) {
      frame.timing.core_end_to_end_time_ms = elapsed;
    }
    stats_.last_core_end_to_end_time_ms = elapsed;
  }
  return update;
}

void LioProcessor::processReadyPendingLidar(std::vector<ProcessedFrame> & frames)
{
  ProcessedFrame frame;
  while (processPendingLidar(frame)) {
    frames.push_back(std::move(frame));
    frame = ProcessedFrame{};
  }
}

const estimator::ImuInitializationStats & LioProcessor::initializationStats() const
{
  return imu_initializer_.stats();
}

bool LioProcessor::getOptimizedMapPoints(
  std::vector<common::MapPoint> & points, const float voxel_size) const
{
  return pipeline_.getOptimizedMapPoints(points, voxel_size);
}

OptimizedLoopRefinementReport LioProcessor::refineOptimizedLoops(const std::string & cloud_archive)
{
  return pipeline_.refineOptimizedLoops(cloud_archive);
}

bool LioProcessor::processPendingLidar(ProcessedFrame & frame)
{
  stats_.last_trajectory_reanchor_time_ms = 0.0;
  if (pending_lidar_.empty() || imu_trajectory_.size() < 2U) {
    return false;
  }
  const auto & lidar_frame = pending_lidar_.front();
  if (lidar_frame.scan_begin_time < imu_trajectory_.front().timestamp) {
    ++stats_.lidar_deskew_failures;
    pending_lidar_.pop_front();
    return false;
  }
  if (lidar_frame.scan_end_time > imu_trajectory_.back().timestamp) {
    return false;
  }

  constexpr double kTimestampTolerance = 1e-9;
  if (imu_state_history_.empty() ||
    lidar_frame.scan_end_time < imu_state_history_.front().imu.timestamp - kTimestampTolerance)
  {
    ++stats_.lidar_deskew_failures;
    pending_lidar_.pop_front();
    return false;
  }
  const auto to_scan_end_start = std::chrono::steady_clock::now();
  frame.timing.callback_start_steady_sec = lidar_frame.callback_start_steady_sec;
  frame.timing.input_callback_ms = lidar_frame.input_callback_ms;
  if (lidar_frame.callback_start_steady_sec > 0.0) {
    frame.timing.pre_processing_wait_ms =
      (std::chrono::duration<double>(to_scan_end_start.time_since_epoch()).count() -
      lidar_frame.callback_start_steady_sec) * 1000.0 - lidar_frame.input_callback_ms;
  }
  const auto right = std::lower_bound(
    imu_state_history_.begin(), imu_state_history_.end(), lidar_frame.scan_end_time,
    [](const ImuStateHistoryEntry & entry, const double timestamp) {
      return entry.imu.timestamp < timestamp;
    });
  if (right == imu_state_history_.end()) {
    return false;
  }
  sensor::ImuData scan_end_imu;
  estimator::NominalState predicted_scan_end_state;
  estimator::StateCovariance predicted_scan_end_covariance;
  if (std::abs(right->imu.timestamp - lidar_frame.scan_end_time) <= kTimestampTolerance) {
    scan_end_imu = right->imu;
    predicted_scan_end_state = right->state;
    predicted_scan_end_covariance = right->covariance;
  } else {
    if (right == imu_state_history_.begin()) {
      ++stats_.lidar_deskew_failures;
      pending_lidar_.pop_front();
      return false;
    }
    const auto left = std::prev(right);
    scan_end_imu = interpolateImu(left->imu, right->imu, lidar_frame.scan_end_time);
    predicted_scan_end_state = left->state;
    predicted_scan_end_covariance = left->covariance;
    if (!imu_propagator_.propagate(
        left->imu, scan_end_imu, predicted_scan_end_state,
        predicted_scan_end_covariance))
    {
      ++stats_.lidar_deskew_failures;
      pending_lidar_.pop_front();
      return false;
    }
  }
  frame.timing.imu_to_scan_end_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - to_scan_end_start).count();
  std::vector<sensor::ImuData> future_imu_samples;
  future_imu_samples.reserve(
    static_cast<std::size_t>(std::distance(right, imu_state_history_.end())));
  for (auto iterator = right; iterator != imu_state_history_.end(); ++iterator) {
    if (iterator->imu.timestamp > lidar_frame.scan_end_time + kTimestampTolerance) {
      future_imu_samples.push_back(iterator->imu);
    }
  }

  // N=1 preserves the input: the range filter below still validates it in full.
  // Borrow it for this scope instead of copying the entire scan.
  if (options_.point_filter_num != 1U) {
    frontend::sampleLidarFrameByStride(
      lidar_frame, options_.point_filter_num, point_filtered_buffer_);
  }
  const sensor::LidarFrame & point_filtered = options_.point_filter_num == 1U ?
    lidar_frame : point_filtered_buffer_;
  const auto range_filter_start = std::chrono::steady_clock::now();
  frontend::filterLidarFrameByRange(
    point_filtered, options_.lidar_min_range, range_filtered_buffer_);
  const sensor::LidarFrame & range_filtered = range_filtered_buffer_;
  frame.timing.range_filter_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - range_filter_start).count();
  stats_.last_range_filter_time_ms = frame.timing.range_filter_time_ms;
  stats_.lidar_range_filtered_points += point_filtered.points.size() - range_filtered.points.size();
  if (range_filtered.points.empty()) {
    ++stats_.lidar_range_filtered_frames;
    pending_lidar_.pop_front();
    return false;
  }
  const auto voxel_start = std::chrono::steady_clock::now();
  sensor::LidarFrame deskew_input;
  auto & selected_indices = selected_indices_buffer_;
  if (options_.full_deskewed_enabled) {
    frontend::voxelSelectLidarPointIndicesFromValidatedFrame(
      range_filtered, options_.deskew_voxel_size, selected_indices);
  } else {
    deskew_input = frontend::voxelSampleLidarFrame(
      range_filtered, options_.deskew_voxel_size);
  }
  frame.timing.voxel_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - voxel_start).count();
  stats_.last_voxel_time_ms = frame.timing.voxel_time_ms;
  sensor::LidarFrame deskewed;
  sensor::LidarFrame full_deskewed;
  const auto deskew_start = std::chrono::steady_clock::now();
  const bool deskew_ok = options_.full_deskewed_enabled ?
    frontend::deskewLidarFrameAndSelect(
    range_filtered, selected_indices, imu_trajectory_, options_.lidar_extrinsics,
    full_deskewed, deskewed, std::min(options_.pipeline.registration.num_threads, 3)) :
    frontend::deskewLidarFrame(
    deskew_input, imu_trajectory_, options_.lidar_extrinsics, deskewed,
    std::min(options_.pipeline.registration.num_threads, 3));
  stats_.last_deskew_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - deskew_start).count();
  frame.timing.deskew_time_ms = stats_.last_deskew_time_ms;
  if (!deskew_ok) {
    ++stats_.lidar_deskew_failures;
    pending_lidar_.pop_front();
    return false;
  }
  ++stats_.lidar_deskewed;
  state_ = predicted_scan_end_state;
  covariance_ = predicted_scan_end_covariance;
  const estimator::NominalState state_before_lidar_update = predicted_scan_end_state;
  PipelineResult pipeline_result = pipeline_.processScan(
    deskewed, state_, covariance_, options_.lidar_extrinsics, &deskewed);
  const estimator::NominalState corrected_scan_end_state = state_;
  const estimator::StateCovariance corrected_scan_end_covariance = covariance_;
  if (pipeline_result.corrected) {
    const auto reanchor_start = std::chrono::steady_clock::now();
    applyStateCorrectionToTrajectory(
      common::Pose{state_before_lidar_update.R_WI, state_before_lidar_update.p_WI},
      common::Pose{state_.R_WI, state_.p_WI}, imu_trajectory_);
    frame.timing.trajectory_reanchor_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - reanchor_start).count();
    stats_.last_trajectory_reanchor_time_ms = frame.timing.trajectory_reanchor_time_ms;
  }
  const auto first_trajectory_at_or_after_scan_end = std::lower_bound(
    imu_trajectory_.begin(), imu_trajectory_.end(), lidar_frame.scan_end_time,
    [](const common::ImuPose & pose, const double timestamp) {
      return pose.timestamp < timestamp;
    });
  imu_trajectory_.erase(first_trajectory_at_or_after_scan_end, imu_trajectory_.end());
  while (!imu_state_history_.empty() &&
    imu_state_history_.back().imu.timestamp >= lidar_frame.scan_end_time - kTimestampTolerance)
  {
    imu_state_history_.pop_back();
  }
  state_ = corrected_scan_end_state;
  covariance_ = corrected_scan_end_covariance;
  previous_imu_sample_ = scan_end_imu;
  appendImuHistory(scan_end_imu);
  const auto repropagation_start = std::chrono::steady_clock::now();
  for (const auto & future_imu : future_imu_samples) {
    if (!imu_propagator_.propagate(
        previous_imu_sample_, future_imu, state_, covariance_))
    {
      ++stats_.trajectory_reprop_failures;
      break;
    }
    previous_imu_sample_ = future_imu;
    appendImuHistory(future_imu);
    ++frame.timing.repropagated_imu_samples;
  }
  frame.timing.repropagation_time_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - repropagation_start).count();
  if (!future_imu_samples.empty()) {
    frame.timing.post_scan_imu_duration_ms =
      (future_imu_samples.back().timestamp - lidar_frame.scan_end_time) * 1000.0;
  }
  if (frame.timing.repropagated_imu_samples > 0U) {
    ++stats_.trajectory_repropagations;
  }
  stats_.last_repropagation_time_ms = frame.timing.repropagation_time_ms;
  stats_.last_repropagated_imu_samples = frame.timing.repropagated_imu_samples;
  stats_.last_degenerate = pipeline_result.degeneracy.degenerate;
  stats_.last_condition_number = pipeline_result.degeneracy.condition_number;
  frame.raw_points = lidar_frame.points.size();
  frame.range_filtered_points = range_filtered.points.size();
  frame.deskewed = std::move(deskewed);
  frame.full_deskewed = std::move(full_deskewed);
  frame.pipeline = std::move(pipeline_result);
  frame.state_at_scan_end = corrected_scan_end_state;
  frame.covariance_diagonal_at_scan_end = corrected_scan_end_covariance.diagonal();
  pending_lidar_.pop_front();
  return true;
}

void LioProcessor::appendImuHistory(const sensor::ImuData & sample)
{
  common::ImuPose pose;
  pose.timestamp = sample.timestamp;
  pose.R_WI = state_.R_WI;
  pose.p_WI = state_.p_WI;
  pose.v_WI = state_.v_WI;
  if (!imu_trajectory_.empty() &&
    std::abs(imu_trajectory_.back().timestamp - sample.timestamp) <= 1e-9)
  {
    imu_trajectory_.back() = pose;
    imu_state_history_.back() = ImuStateHistoryEntry{sample, state_, covariance_};
  } else {
    imu_trajectory_.push_back(pose);
    imu_state_history_.push_back(ImuStateHistoryEntry{sample, state_, covariance_});
  }
  const double cutoff = sample.timestamp - options_.imu_history_duration_sec;
  const auto first_after_cutoff = std::lower_bound(
    imu_trajectory_.begin(), imu_trajectory_.end(), cutoff,
    [](const common::ImuPose & value, const double timestamp) {
      return value.timestamp < timestamp;
    });
  auto first_to_keep = first_after_cutoff;
  if (first_to_keep != imu_trajectory_.begin()) {
    --first_to_keep;
  }
  if (first_to_keep != imu_trajectory_.begin()) {
    const std::size_t erase_count = static_cast<std::size_t>(
      std::distance(imu_trajectory_.begin(), first_to_keep));
    imu_trajectory_.erase(imu_trajectory_.begin(), first_to_keep);
    for (std::size_t index = 0U; index < erase_count; ++index) {
      imu_state_history_.pop_front();
    }
  }
  if (imu_trajectory_.size() > options_.imu_history_max_samples) {
    const std::size_t excess = imu_trajectory_.size() - options_.imu_history_max_samples;
    imu_trajectory_.erase(
      imu_trajectory_.begin(),
      imu_trajectory_.begin() + static_cast<std::ptrdiff_t>(excess));
    for (std::size_t index = 0U; index < excess; ++index) {
      imu_state_history_.pop_front();
    }
  }
  stats_.imu_history_samples = imu_trajectory_.size();
}

sensor::ImuData LioProcessor::interpolateImu(
  const sensor::ImuData & previous,
  const sensor::ImuData & current,
  const double timestamp)
{
  sensor::ImuData result;
  result.timestamp = timestamp;
  const double span = current.timestamp - previous.timestamp;
  const double alpha = span > 0.0 ?
    std::clamp((timestamp - previous.timestamp) / span, 0.0, 1.0) : 0.0;
  result.gyro = (1.0 - alpha) * previous.gyro + alpha * current.gyro;
  result.accel = (1.0 - alpha) * previous.accel + alpha * current.accel;
  return result;
}

}  // namespace itera_lio::system
