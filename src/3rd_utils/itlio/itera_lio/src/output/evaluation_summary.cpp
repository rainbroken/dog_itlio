// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/output/evaluation_logger.hpp"
#include "evaluation_format.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <numeric>
#include <sstream>

namespace itera_lio::output
{
namespace detail
{
std::string jsonEscape(const std::string & value)
{
  std::ostringstream escaped;
  for (const char character : value) {
    switch (character) {
      case '\\': escaped << "\\\\"; break;
      case '"': escaped << "\\\""; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default: escaped << character; break;
    }
  }
  return escaped.str();
}
}  // namespace detail
namespace
{

struct SummaryStatistics
{
  std::size_t count{0U};
  double mean{0.0};
  double maximum{0.0};
  double p95{0.0};
  double p99{0.0};
  double configured_percentile{0.0};
};

double percentile(const std::vector<double> & sorted, const double probability)
{
  if (sorted.empty()) {
    return 0.0;
  }
  const double index = probability * static_cast<double>(sorted.size() - 1U);
  const auto lower = static_cast<std::size_t>(std::floor(index));
  const auto upper = static_cast<std::size_t>(std::ceil(index));
  const double fraction = index - static_cast<double>(lower);
  return sorted[lower] * (1.0 - fraction) + sorted[upper] * fraction;
}

SummaryStatistics summarize(
  const std::vector<double> & samples, const double configured_probability)
{
  SummaryStatistics result;
  std::vector<double> sorted;
  sorted.reserve(samples.size());
  for (const double sample : samples) {
    if (std::isfinite(sample)) {
      sorted.push_back(sample);
    }
  }
  result.count = sorted.size();
  if (sorted.empty()) {
    return result;
  }
  std::sort(sorted.begin(), sorted.end());
  result.mean = std::accumulate(sorted.begin(), sorted.end(), 0.0) /
    static_cast<double>(sorted.size());
  result.maximum = sorted.back();
  result.p95 = percentile(sorted, 0.95);
  result.p99 = percentile(sorted, 0.99);
  result.configured_percentile = percentile(sorted, configured_probability);
  return result;
}

void writeStatistics(
  std::ostream & stream, const char * name, const SummaryStatistics & statistics,
  const double configured_probability, const bool trailing_comma)
{
  stream << "    \"" << name << "\": {"
         << "\"count\": " << statistics.count
         << ", \"mean\": " << statistics.mean
         << ", \"max\": " << statistics.maximum
         << ", \"p95\": " << statistics.p95
         << ", \"p99\": " << statistics.p99
         << ", \"configured_percentile_probability\": "
         << configured_probability
         << ", \"configured_percentile\": "
         << statistics.configured_percentile << "}"
         << (trailing_comma ? "," : "") << '\n';
}

}  // namespace

// These operations run under the logger's existing mutex. The logger owns all samples.
void EvaluationLogger::updateSummary(
  const sensor::LidarFrame & scan, const system::PipelineResult & result,
  const system::FrameProcessingTiming & processing_timing,
  const FrameOutputTiming & output_timing)
{
  if (!options_.summary) {
    return;
  }
  if (!has_first_sensor_timestamp_) {
    has_first_sensor_timestamp_ = true;
    first_sensor_timestamp_ = scan.scan_end_time;
  }
  last_sensor_timestamp_ = scan.scan_end_time;
  map_updated_frames_ += result.map_updated ? 1U : 0U;
  degenerate_frames_ += result.degeneracy.degenerate ? 1U : 0U;
  loop_candidates_ += result.loop_candidates;
  loop_verifications_ += result.loop_verifications;
  loop_accepted_ += result.loop_accepted_verifications;
  loop_committed_ += result.backend.loop_constraint_added ? 1U : 0U;
  if (!result.loop_verification_results.empty()) {
    for (const auto & verification : result.loop_verification_results) {
      if (!verification.isValid()) {
        ++loop_rejected_;
        ++loop_rejection_reasons_[verification.rejection_reason];
      }
    }
  } else if (result.loop_rejected_verifications > 0U) {
    loop_rejected_ += result.loop_rejected_verifications;
    loop_rejection_reasons_[result.loop_rejection_reason] +=
      result.loop_rejected_verifications;
  }
  total_map_candidates_ += result.map_candidate_points;
  total_map_inserted_ += result.map_inserted_points;
  total_map_replaced_ += result.map_replaced_points;
  total_map_rejected_ += result.map_rejected_points;
  final_pipeline_global_map_points_ = result.global_map_size;
  final_dense_archive_points_ = result.dense_archive_size;
  final_stored_loop_keyframe_clouds_ = result.stored_loop_keyframe_clouds;
  backend_optimization_runs_ += result.backend.optimization.optimized ? 1U : 0U;

  if (scan.scan_end_time - first_sensor_timestamp_ < options_.warmup_sec) {
    return;
  }
  residual_before_.samples.push_back(result.correction.mean_abs_residual_before);
  residual_after_.samples.push_back(result.correction.mean_abs_residual_after);
  registration_ms_.samples.push_back(result.registration_time_ms);
  correction_ms_.samples.push_back(result.correction_time_ms);
  map_update_ms_.samples.push_back(result.map_update_time_ms);
  core_ms_.samples.push_back(processing_timing.core_end_to_end_time_ms);
  output_ms_.samples.push_back(output_timing.output_total_time_ms);
  end_to_end_ms_.samples.push_back(output_timing.end_to_end_time_ms);
}

void EvaluationLogger::writeSummary(const common::RuntimeSnapshot & snapshot)
{
  if (!options_.summary) {
    return;
  }
  namespace fs = std::filesystem;
  std::ofstream summary(
    (fs::path(run_directory_) / "summary.json").string(), std::ios::out);
  if (!summary.is_open()) {
    return;
  }
  summary << std::setprecision(17);
  const double duration = has_first_sensor_timestamp_ ?
    std::max(0.0, last_sensor_timestamp_ - first_sensor_timestamp_) : 0.0;
  summary << "{\n"
          << "  \"normal_exit\": true,\n"
          << "  \"warmup_sec\": " << options_.warmup_sec << ",\n"
          << "  \"configured_percentile\": " << options_.comparison_percentile << ",\n"
          << "  \"frames\": {\"total\": " << frame_index_
          << ", \"after_warmup\": " << end_to_end_ms_.samples.size()
          << ", \"duration_sec\": " << duration
          << ", \"map_updated\": " << map_updated_frames_
          << ", \"degenerate\": " << degenerate_frames_ << "},\n"
          << "  \"sensor\": {\"lidar_received\": " << snapshot.lidar_received
          << ", \"imu_received\": " << snapshot.imu_received
          << ", \"lidar_processed\": " << snapshot.lidar_processed
          << ", \"imu_processed\": " << snapshot.imu_processed
          << ", \"lidar_deskewed\": " << snapshot.lidar_deskewed << "},\n"
          << "  \"failures\": {\"lidar_parse\": " << snapshot.lidar_parse_failures
          << ", \"lidar_deskew\": " << snapshot.lidar_deskew_failures
          << ", \"lidar_timestamp_regressions\": "
          << snapshot.lidar_timestamp_regressions
          << ", \"imu_timestamp_regressions\": " << snapshot.imu_timestamp_regressions
          << ", \"lidar_capacity_drops\": " << snapshot.lidar_capacity_drops
          << ", \"imu_capacity_drops\": " << snapshot.imu_capacity_drops
          << ", \"pending_lidar_drops\": " << snapshot.pending_lidar_drops
          << ", \"imu_gap_resets\": " << snapshot.imu_gap_resets << "},\n"
          << "  \"map\": {\"final_local_points\": " << snapshot.local_map_points
          << ", \"final_pipeline_global_points\": "
          << final_pipeline_global_map_points_
          << ", \"final_output_global_points\": "
          << snapshot.output_global_map_points
          << ", \"final_dense_archive_points\": " << final_dense_archive_points_
          << ", \"candidate_points_total\": " << total_map_candidates_
          << ", \"inserted_points_total\": " << total_map_inserted_
          << ", \"replaced_points_total\": " << total_map_replaced_
          << ", \"rejected_points_total\": " << total_map_rejected_
          << ", \"rebuilds\": " << snapshot.rebuild_total << "},\n"
          << "  \"backend\": {\"keyframes\": " << snapshot.backend_keyframes
          << ", \"odometry_constraints\": " << snapshot.backend_odometry_constraints
          << ", \"loop_constraints\": " << snapshot.backend_loop_constraints
          << ", \"optimization_runs\": " << backend_optimization_runs_ << "},\n"
          << "  \"revisit_refinement\": {\"calls\": " << refinement_calls_
          << ", \"verifications\": " << refinement_verifications_
          << ", \"accepted\": " << refinement_accepted_
          << ", \"committed\": " << refinement_committed_ << "},\n"
          << "  \"loop\": {\"candidates\": " << loop_candidates_
          << ", \"verifications\": " << loop_verifications_
          << ", \"accepted\": " << loop_accepted_
          << ", \"committed\": " << loop_committed_
          << ", \"rejected\": " << loop_rejected_
          << ", \"stored_keyframe_clouds\": " << final_stored_loop_keyframe_clouds_
          << ", \"rejection_reasons\": {";
  std::size_t reason_index = 0U;
  for (const auto & reason : loop_rejection_reasons_) {
    summary << (reason_index++ == 0U ? "" : ", ") << '"'
            << detail::jsonEscape(reason.first) << "\": " << reason.second;
  }
  summary << "}},\n"
          << "  \"memory\": {\"final_resident_mib\": "
          << snapshot.resident_memory_mib << ", \"peak_resident_mib\": "
          << snapshot.peak_resident_memory_mib << "},\n"
          << "  \"statistics\": {\n";
  writeStatistics(
    summary, "mean_residual_before",
    summarize(residual_before_.samples, options_.comparison_percentile),
    options_.comparison_percentile, true);
  writeStatistics(
    summary, "mean_residual_after",
    summarize(residual_after_.samples, options_.comparison_percentile),
    options_.comparison_percentile, true);
  writeStatistics(
    summary, "registration_ms",
    summarize(registration_ms_.samples, options_.comparison_percentile),
    options_.comparison_percentile, true);
  writeStatistics(
    summary, "correction_ms",
    summarize(correction_ms_.samples, options_.comparison_percentile),
    options_.comparison_percentile, true);
  writeStatistics(
    summary, "map_update_ms",
    summarize(map_update_ms_.samples, options_.comparison_percentile),
    options_.comparison_percentile, true);
  writeStatistics(
    summary, "core_ms", summarize(core_ms_.samples, options_.comparison_percentile),
    options_.comparison_percentile, true);
  writeStatistics(
    summary, "output_ms", summarize(output_ms_.samples, options_.comparison_percentile),
    options_.comparison_percentile, true);
  writeStatistics(
    summary, "end_to_end_ms",
    summarize(end_to_end_ms_.samples, options_.comparison_percentile),
    options_.comparison_percentile, false);
  summary << "  }\n}\n";
}

}  // namespace itera_lio::output
