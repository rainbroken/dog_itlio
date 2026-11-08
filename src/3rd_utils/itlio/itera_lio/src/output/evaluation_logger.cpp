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

#include "itera_lio/output/evaluation_logger.hpp"
#include "evaluation_format.hpp"

#include <chrono>
#include <filesystem>
#include <iomanip>

namespace itera_lio::output
{
namespace
{

template<typename ValueT>
std::uint64_t increase(const ValueT current, const ValueT previous)
{
  return current >= previous ? static_cast<std::uint64_t>(current - previous) : 0U;
}

double systemWallTimestamp()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

}  // namespace

bool EvaluationLogger::initialize(
  const std::string & output_directory, const EvaluationMetadata & metadata,
  const EvaluationOutputOptions & options,
  const std::vector<std::pair<std::string, std::string>> & parameters,
  std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!options.anyEnabled()) {
    error = "no evaluation outputs are enabled";
    return false;
  }
  options_ = options;
  try {
    namespace fs = std::filesystem;
    const fs::path root = fs::absolute(fs::path(output_directory));
    fs::create_directories(root);
    run_directory_ = root.string();
    fs::create_directories(run_directory_);

    auto open = [&](std::ofstream & stream, const char * filename, const bool enabled) {
        if (!enabled) {
          return true;
        }
        stream.open((fs::path(run_directory_) / filename).string(), std::ios::out);
        if (!stream.is_open()) {
          error = std::string("cannot open evaluation output file: ") + filename;
          return false;
        }
        return true;
      };
    if (!open(frames_, "frames.csv", options_.frames) ||
      !open(odometry_, "odom.csv", options_.odometry) ||
      !open(runtime_, "runtime.csv", options_.runtime) ||
      !open(events_, "events.csv", options_.events) ||
      !open(
        loop_verifications_file_, "loop_verifications.csv",
        options_.loop_verifications))
    {
      return false;
    }

    if (frames_.is_open()) {
      detail::writeFrameHeader(frames_, options_.detailed);
    }
    if (odometry_.is_open()) {
      detail::writeOdometryHeader(odometry_);
    }
    if (runtime_.is_open()) {
      detail::writeRuntimeHeader(runtime_, options_.detailed);
    }
    if (events_.is_open()) {
      events_ << "wall_timestamp,sensor_timestamp,frame_index,event_type,severity,"
        "count,detail,value1,value2\n";
    }
    if (loop_verifications_file_.is_open()) {
      detail::writeLoopVerificationHeader(loop_verifications_file_);
    }

    std::vector<std::string> files;
    if (options_.frames) {files.emplace_back("frames.csv");}
    if (options_.runtime) {files.emplace_back("runtime.csv");}
    if (options_.events) {files.emplace_back("events.csv");}
    if (options_.loop_verifications) {files.emplace_back("loop_verifications.csv");}
    if (options_.summary) {files.emplace_back("summary.json");}
    if (options_.odometry) {files.emplace_back("odom.csv");}
    if (options_.parameters) {files.emplace_back("parameters.yaml");}
    if (options_.manifest) {files.emplace_back("manifest.json");}

    if (options_.manifest) {
      std::ofstream manifest(
        (fs::path(run_directory_) / "manifest.json").string(), std::ios::out);
      if (!manifest.is_open()) {
        error = "cannot open evaluation output file: manifest.json";
        return false;
      }
      manifest << "{\n"
               << "  \"schema_version\": 4,\n"
               << "  \"detailed\": " << (options_.detailed ? "true" : "false") << ",\n"
               << "  \"run_dir\": \"" << detail::jsonEscape(run_directory_) << "\",\n"
               << "  \"lidar_topic\": \"" << detail::jsonEscape(metadata.lidar_topic) << "\",\n"
               << "  \"imu_topic\": \"" << detail::jsonEscape(metadata.imu_topic) << "\",\n"
               << "  \"world_frame\": \"" << detail::jsonEscape(metadata.world_frame) << "\",\n"
               << "  \"imu_frame\": \"" << detail::jsonEscape(metadata.imu_frame) << "\",\n"
               << "  \"lidar_frame\": \"" << detail::jsonEscape(metadata.lidar_frame) << "\",\n"
               << "  \"files\": [";
      for (std::size_t index = 0U; index < files.size(); ++index) {
        manifest << (index == 0U ? "" : ", ") << '"' << files[index] << '"';
      }
      manifest << "],\n  \"evaluation_enabled\": true\n}\n";
    }
    if (options_.parameters) {
      std::ofstream parameter_file(
        (fs::path(run_directory_) / "parameters.yaml").string(), std::ios::out);
      if (!parameter_file.is_open()) {
        error = "cannot open evaluation output file: parameters.yaml";
        return false;
      }
      parameter_file << "parameters:\n";
      for (const auto & parameter : parameters) {
        parameter_file << "  " << parameter.first << ": " << parameter.second << '\n';
      }
    }
    previous_runtime_.wall_timestamp = systemWallTimestamp();
    has_previous_runtime_ = true;
    initialized_ = true;
    return true;
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  }
}

bool EvaluationLogger::isOpen() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_ && !finalized_;
}

void EvaluationLogger::writeFrame(
  const std::size_t raw_scan_points, const sensor::LidarFrame & scan,
  const system::PipelineResult & result,
  const system::FrameProcessingTiming & processing_timing,
  const FrameOutputTiming & output_timing)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || finalized_) {
    return;
  }
  const auto current_frame_index = frame_index_++;
  const double wall_timestamp = systemWallTimestamp();
  const auto & observation_update = result.correction.observation_update;
  if (frames_.is_open()) {
    detail::writeFrameRow(
      frames_, current_frame_index, raw_scan_points, scan, result,
      processing_timing, output_timing, options_.detailed);
  }

  for (const auto & verification : result.loop_verification_results) {
    if (loop_verifications_file_.is_open()) {
      detail::writeLoopVerificationRow(
        loop_verifications_file_, wall_timestamp, scan.scan_end_time,
        current_frame_index, verification);
    }
    if (options_.detailed || !loop_verifications_file_.is_open()) {
      writeEvent(
        wall_timestamp, scan.scan_end_time, current_frame_index,
        verification.isValid() ? "loop_verified" : "loop_rejected",
        verification.isValid() ? "info" : "warning", 1U,
        std::to_string(verification.current_id) + "->" +
        std::to_string(verification.matched_id) + ":" +
        verification.initialization + ":" + verification.rejection_reason,
        verification.fitness, static_cast<double>(verification.inliers));
    }
  }
  if (result.tracking.transitioned) {
    writeEvent(
      wall_timestamp, scan.scan_end_time, current_frame_index, "tracking_transition",
      result.tracking.state == estimator::TrackingState::kLost ? "error" : "warning", 1U,
      std::string(estimator::trackingStateName(result.tracking.previous_state)) + "_to_" +
      estimator::trackingStateName(result.tracking.state),
      static_cast<double>(result.correction.effective_points),
      observation_update.whitened_condition_number);
  }
  if (result.backend.loop_constraint_added) {
    writeEvent(
      wall_timestamp, scan.scan_end_time, current_frame_index, "loop_committed", "info", 1U,
      "backend_optimized", result.backend.optimization.initial_cost,
      result.backend.optimization.final_cost);
  }
  if (result.loop_verification_results.empty() &&
    result.loop_rejected_verifications > 0U)
  {
    writeEvent(
      wall_timestamp, scan.scan_end_time, current_frame_index, "loop_rejected", "warning",
      result.loop_rejected_verifications, result.loop_rejection_reason, result.loop_fitness,
      static_cast<double>(result.loop_inliers));
  } else if (result.loop_verification_results.empty() &&
    result.loop_accepted_verifications > 0U)
  {
    writeEvent(
      wall_timestamp, scan.scan_end_time, current_frame_index, "loop_verified", "info",
      result.loop_accepted_verifications, result.loop_rejection_reason, result.loop_fitness,
      static_cast<double>(result.loop_inliers));
  }
  if (result.ikd_rebuild.total_count > last_frame_rebuild_total_ &&
    (options_.detailed || result.ikd_rebuild.last_duration_ms >= options_.slow_rebuild_ms))
  {
    writeEvent(
      wall_timestamp, scan.scan_end_time, current_frame_index,
      options_.detailed ? "map_rebuild" : "slow_map_rebuild",
      result.ikd_rebuild.last_duration_ms >= options_.slow_rebuild_ms ? "warning" : "info",
      options_.detailed ? result.ikd_rebuild.total_count - last_frame_rebuild_total_ : 1U,
      result.ikd_rebuild.asynchronous_in_progress ? "asynchronous_active" : "completed",
      result.ikd_rebuild.last_duration_ms,
      static_cast<double>(result.ikd_rebuild.last_subtree_size));
  }
  last_frame_rebuild_total_ = result.ikd_rebuild.total_count;
  updateSummary(scan, result, processing_timing, output_timing);
}

void EvaluationLogger::writeOdometry(
  const double timestamp, const estimator::NominalState & state,
  const estimator::ErrorStateVector & covariance_diagonal)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || finalized_ || !odometry_.is_open()) {
    return;
  }
  detail::writeOdometryRow(odometry_, timestamp, state, covariance_diagonal);
}

void EvaluationLogger::writeRuntime(const common::RuntimeSnapshot & snapshot)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || finalized_) {
    return;
  }
  const double elapsed = has_previous_runtime_ ?
    snapshot.wall_timestamp - previous_runtime_.wall_timestamp : 0.0;
  const double lidar_rate = elapsed > 0.0 ?
    static_cast<double>(increase(snapshot.lidar_received, previous_runtime_.lidar_received)) /
    elapsed : 0.0;
  const double imu_rate = elapsed > 0.0 ?
    static_cast<double>(increase(snapshot.imu_received, previous_runtime_.imu_received)) /
    elapsed : 0.0;
  if (runtime_.is_open()) {
    detail::writeRuntimeRow(runtime_, snapshot, lidar_rate, imu_rate, options_.detailed);
  }
  writeRuntimeEvents(snapshot);
  previous_runtime_ = snapshot;
  has_previous_runtime_ = true;
}

void EvaluationLogger::recordRevisitRefinement(
  const std::size_t verifications, const std::size_t accepted, const std::size_t committed)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || finalized_) {return;}
  ++refinement_calls_;
  refinement_verifications_ += verifications;
  refinement_accepted_ += accepted;
  refinement_committed_ += committed;
  backend_optimization_runs_ += committed;
}

void EvaluationLogger::finalize(const common::RuntimeSnapshot & snapshot)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || finalized_) {
    return;
  }
  const double elapsed = has_previous_runtime_ ?
    snapshot.wall_timestamp - previous_runtime_.wall_timestamp : 0.0;
  const double lidar_rate = elapsed > 0.0 ?
    static_cast<double>(increase(snapshot.lidar_received, previous_runtime_.lidar_received)) /
    elapsed : 0.0;
  const double imu_rate = elapsed > 0.0 ?
    static_cast<double>(increase(snapshot.imu_received, previous_runtime_.imu_received)) /
    elapsed : 0.0;
  if (runtime_.is_open()) {
    detail::writeRuntimeRow(runtime_, snapshot, lidar_rate, imu_rate, options_.detailed);
  }
  writeRuntimeEvents(snapshot);
  writeSummary(snapshot);
  frames_.flush();
  odometry_.flush();
  runtime_.flush();
  events_.flush();
  loop_verifications_file_.flush();
  finalized_ = true;
}

void EvaluationLogger::writeEvent(
  const double wall_timestamp, const double sensor_timestamp,
  const std::uint64_t frame_index, const std::string & event_type,
  const std::string & severity, const std::uint64_t count,
  const std::string & detail, const double value1, const double value2)
{
  if (!events_.is_open()) {
    return;
  }
  events_ << std::setprecision(17) << wall_timestamp << ',' << sensor_timestamp << ','
          << frame_index << ',' << detail::csvEscape(event_type) << ',' << detail::csvEscape(severity) << ','
          << count << ',' << detail::csvEscape(detail) << ',' << value1 << ',' << value2 << '\n';
}

void EvaluationLogger::writeRuntimeEvents(const common::RuntimeSnapshot & snapshot)
{
  if (!has_previous_runtime_) {
    return;
  }
  const auto frame = frame_index_ == 0U ? 0U : frame_index_ - 1U;
  const auto emit = [&](const char * type, const char * severity, const std::uint64_t count,
      const char * detail) {
      if (count > 0U) {
        writeEvent(
          snapshot.wall_timestamp, snapshot.sensor_timestamp, frame, type, severity, count,
          detail);
      }
    };
  emit(
    "lidar_timestamp_regression", "error",
    increase(
      snapshot.lidar_timestamp_regressions,
      previous_runtime_.lidar_timestamp_regressions),
    "LiDAR timestamp moved backward");
  emit(
    "imu_timestamp_regression", "error",
    increase(snapshot.imu_timestamp_regressions, previous_runtime_.imu_timestamp_regressions),
    "IMU timestamp moved backward");
  emit(
    "lidar_capacity_drop", "warning",
    increase(snapshot.lidar_capacity_drops, previous_runtime_.lidar_capacity_drops),
    "LiDAR input queue reached capacity");
  emit(
    "imu_capacity_drop", "warning",
    increase(snapshot.imu_capacity_drops, previous_runtime_.imu_capacity_drops),
    "IMU input queue reached capacity");
  emit(
    "pending_lidar_drop", "warning",
    increase(snapshot.pending_lidar_drops, previous_runtime_.pending_lidar_drops),
    "pending LiDAR queue reached capacity");
  emit(
    "lidar_parse_failure", "warning",
    increase(snapshot.lidar_parse_failures, previous_runtime_.lidar_parse_failures),
    "PointCloud2 parsing failed");
  emit(
    "lidar_deskew_failure", "warning",
    increase(snapshot.lidar_deskew_failures, previous_runtime_.lidar_deskew_failures),
    "LiDAR deskew failed");
  emit(
    "imu_gap_reset", "warning",
    increase(snapshot.imu_gap_resets, previous_runtime_.imu_gap_resets),
    "IMU interval exceeded configured limit");
}

}  // namespace itera_lio::output
