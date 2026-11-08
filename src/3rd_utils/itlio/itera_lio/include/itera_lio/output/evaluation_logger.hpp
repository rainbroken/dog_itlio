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

#ifndef ITERA_LIO__OUTPUT__EVALUATION_LOGGER_HPP_
#define ITERA_LIO__OUTPUT__EVALUATION_LOGGER_HPP_

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "itera_lio/common/frame_metrics.hpp"
#include "itera_lio/common/evaluation_types.hpp"
#include "itera_lio/estimator/state.hpp"
#include "itera_lio/sensor/lidar_frame.hpp"
#include "itera_lio/system/lio_pipeline.hpp"
#include "itera_lio/system/lio_processor.hpp"

namespace itera_lio::output
{

struct EvaluationMetadata
{
  std::string lidar_topic;
  std::string imu_topic;
  std::string world_frame;
  std::string imu_frame;
  std::string lidar_frame;
};

struct EvaluationOutputOptions
{
  bool detailed{false};
  double slow_rebuild_ms{100.0};
  bool frames{true};
  bool runtime{true};
  bool events{true};
  bool loop_verifications{true};
  bool summary{true};
  bool odometry{true};
  bool parameters{true};
  bool manifest{true};
  double runtime_frequency_hz{0.2};
  double warmup_sec{2.0};
  double comparison_percentile{0.95};

  bool anyEnabled() const
  {
    return frames || runtime || events || loop_verifications || summary ||
           odometry || parameters || manifest;
  }
};

using FrameOutputTiming = common::FrameOutputTiming;

class EvaluationLogger
{
public:
  bool
  initialize(
    const std::string & output_directory,
    const EvaluationMetadata & metadata,
    const EvaluationOutputOptions & options,
    const std::vector<std::pair<std::string, std::string>> & parameters,
    std::string & error);

  bool isOpen() const;
  const std::string & runDirectory() const {return run_directory_;}

  void writeFrame(
    std::size_t raw_scan_points, const sensor::LidarFrame & scan,
    const system::PipelineResult & result,
    const system::FrameProcessingTiming & processing_timing,
    const FrameOutputTiming & output_timing);

  void writeOdometry(double timestamp, const estimator::NominalState & state,
    const estimator::ErrorStateVector & covariance_diagonal);
  void writeRuntime(const common::RuntimeSnapshot & snapshot);
  void recordRevisitRefinement(std::size_t verifications, std::size_t accepted, std::size_t committed);
  void finalize(const common::RuntimeSnapshot & snapshot);

private:
  struct Series
  {
    std::vector<double> samples;
  };

  void writeEvent(
    double wall_timestamp, double sensor_timestamp,
    std::uint64_t frame_index, const std::string & event_type,
    const std::string & severity, std::uint64_t count,
    const std::string & detail, double value1 = 0.0,
    double value2 = 0.0);
  void writeRuntimeEvents(const common::RuntimeSnapshot & snapshot);
  void updateSummary(
    const sensor::LidarFrame & scan,
    const system::PipelineResult & result,
    const system::FrameProcessingTiming & processing_timing,
    const FrameOutputTiming & output_timing);
  void writeSummary(const common::RuntimeSnapshot & snapshot);

  EvaluationOutputOptions options_;
  std::string run_directory_;
  std::ofstream frames_;
  std::ofstream odometry_;
  std::ofstream runtime_;
  std::ofstream events_;
  std::ofstream loop_verifications_file_;
  mutable std::mutex mutex_;
  bool initialized_{false};
  bool finalized_{false};
  std::uint64_t frame_index_{0U};
  bool has_first_sensor_timestamp_{false};
  double first_sensor_timestamp_{0.0};
  double last_sensor_timestamp_{0.0};
  std::uint64_t map_updated_frames_{0U};
  std::uint64_t degenerate_frames_{0U};
  std::uint64_t loop_candidates_{0U};
  std::uint64_t loop_verifications_{0U};
  std::uint64_t loop_accepted_{0U};
  std::uint64_t loop_committed_{0U};
  std::uint64_t loop_rejected_{0U};
  std::uint64_t total_map_candidates_{0U};
  std::uint64_t total_map_inserted_{0U};
  std::uint64_t total_map_replaced_{0U};
  std::uint64_t total_map_rejected_{0U};
  std::uint64_t last_frame_rebuild_total_{0U};
  std::map<std::string, std::uint64_t> loop_rejection_reasons_;
  std::size_t final_pipeline_global_map_points_{0U};
  std::size_t final_dense_archive_points_{0U};
  std::size_t final_stored_loop_keyframe_clouds_{0U};
  std::uint64_t backend_optimization_runs_{0U};
  std::uint64_t refinement_calls_{0U}, refinement_verifications_{0U};
  std::uint64_t refinement_accepted_{0U}, refinement_committed_{0U};
  Series residual_before_;
  Series residual_after_;
  Series registration_ms_;
  Series correction_ms_;
  Series map_update_ms_;
  Series core_ms_;
  Series output_ms_;
  Series end_to_end_ms_;
  common::RuntimeSnapshot previous_runtime_;
  bool has_previous_runtime_{false};
};

}  // namespace itera_lio::output

#endif  // ITERA_LIO__OUTPUT__EVALUATION_LOGGER_HPP_
