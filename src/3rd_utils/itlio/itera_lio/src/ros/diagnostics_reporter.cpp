// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/diagnostics_reporter.hpp"

#include <sys/resource.h>
#include <unistd.h>

#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <chrono>

#include "itera_lio/ros/input_adapter.hpp"

namespace itera_lio::ros {
void DiagnosticsReporter::updateBackendMetrics(const backend::BackendManager &backend) {
  const auto graph = backend.graph();
  std::lock_guard<std::mutex> lock(mutex_);
  frame_snapshot_.backend_keyframes = graph.nodes().size();
  frame_snapshot_.backend_loop_constraints = 0;
  frame_snapshot_.backend_odometry_constraints = 0;
  for (const auto & edge : graph.constraints()) {
    if (edge.type == backend::ConstraintType::kLoop) {++frame_snapshot_.backend_loop_constraints;}
    else {++frame_snapshot_.backend_odometry_constraints;}
  }
}

std::pair<double, double> processMemoryMib() {
  double resident_mib = 0.0;
  std::ifstream statm("/proc/self/statm");
  std::int64_t total_pages = 0;
  std::int64_t resident_pages = 0;
  if (statm >> total_pages >> resident_pages) {
    (void)total_pages;
    const std::int64_t page_size =
        static_cast<std::int64_t>(sysconf(_SC_PAGESIZE));
    if (page_size > 0) {
      resident_mib = static_cast<double>(resident_pages) *
                     static_cast<double>(page_size) / (1024.0 * 1024.0);
    }
  }
  struct rusage usage {};
  double peak_mib = 0.0;
  if (getrusage(RUSAGE_SELF, &usage) == 0) {
    peak_mib = static_cast<double>(usage.ru_maxrss) / 1024.0;
  }
  return {resident_mib, peak_mib};
}

DiagnosticsReporter::DiagnosticsReporter(
    rclcpp::Node &node, const double period_sec,
    const sensor::MeasurementSynchronizer &synchronizer)
    : node_(node), period_sec_(period_sec), synchronizer_(synchronizer) {
  if (!std::isfinite(period_sec_) || period_sec_ <= 0.0) {
    throw std::invalid_argument(
        "diagnostic period must be finite and positive");
  }
}

void DiagnosticsReporter::setInputAdapter(
    const RosInputAdapter *input_adapter) {
  std::lock_guard<std::mutex> lock(mutex_);
  input_adapter_ = input_adapter;
}

void DiagnosticsReporter::updateProcessorStats(
    const system::LioProcessorStats &stats) {
  std::lock_guard<std::mutex> lock(mutex_);
  processor_stats_ = stats;
}

void DiagnosticsReporter::updateFrameMetrics(
    const system::ProcessedFrame &frame,
    const common::FrameOutputTiming &output_timing,
    const std::size_t global_map_size) {
  const auto &result = frame.pipeline;
  std::lock_guard<std::mutex> lock(mutex_);
  frame_snapshot_.processing = frame.timing;
  frame_snapshot_.output = output_timing;
  frame_snapshot_.sensor_timestamp = frame.deskewed.scan_end_time;
  frame_snapshot_.registration_time_ms = result.registration_time_ms;
  frame_snapshot_.correction_time_ms = result.correction_time_ms;
  frame_snapshot_.map_delete_time_ms = result.map_delete_time_ms;
  frame_snapshot_.archive_update_time_ms = result.archive_update_time_ms;
  frame_snapshot_.map_insert_time_ms = result.map_insert_time_ms;
  frame_snapshot_.map_candidate_points = result.map_candidate_points;
  frame_snapshot_.map_inserted_points = result.map_inserted_points;
  frame_snapshot_.map_replaced_points = result.map_replaced_points;
  frame_snapshot_.map_rejected_points = result.map_rejected_points;
  frame_snapshot_.map_size = result.map_size;
  frame_snapshot_.global_map_size = global_map_size;
  frame_snapshot_.pending_delete_boxes = result.pending_delete_boxes;
  frame_snapshot_.rebuild_last_ms = result.ikd_rebuild.last_duration_ms;
  frame_snapshot_.rebuild_max_ms = result.ikd_rebuild.max_duration_ms;
  frame_snapshot_.rebuild_active = result.ikd_rebuild.asynchronous_in_progress;
  frame_snapshot_.rebuild_total = result.ikd_rebuild.total_count;
  frame_snapshot_.rebuild_synchronous = result.ikd_rebuild.synchronous_count;
  frame_snapshot_.rebuild_asynchronous = result.ikd_rebuild.asynchronous_count;
  frame_snapshot_.rebuild_last_reason = result.ikd_rebuild.last_reason;
  frame_snapshot_.rebuild_last_points = result.ikd_rebuild.last_subtree_size;
  frame_snapshot_.rebuild_max_points = result.ikd_rebuild.max_subtree_size;
  frame_snapshot_.backend_keyframes = result.backend.keyframe_count;
  frame_snapshot_.backend_odometry_constraints =
    result.backend.odometry_constraint_count;
  frame_snapshot_.backend_loop_constraints = result.backend.loop_constraint_count;
  frame_snapshot_.loop_worker = result.loop_worker;
  frame_snapshot_.backend_worker = result.backend_worker;
}

common::RuntimeSnapshot DiagnosticsReporter::snapshot() const {
  const auto current = synchronizer_.stats();
  system::LioProcessorStats processor_stats;
  FrameSnapshot frame;
  const RosInputAdapter *input_adapter = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    processor_stats = processor_stats_;
    frame = frame_snapshot_;
    input_adapter = input_adapter_;
  }
  common::RuntimeSnapshot result;
  result.wall_timestamp = std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  result.sensor_timestamp = frame.sensor_timestamp;
  result.lidar_received = current.lidar_received;
  result.imu_received = current.imu_received;
  result.lidar_queue_size = current.lidar_buffer_size;
  result.imu_queue_size = current.imu_buffer_size;
  result.lidar_processed = processor_stats.lidar_processed;
  result.imu_processed = processor_stats.imu_processed;
  result.lidar_deskewed = processor_stats.lidar_deskewed;
  result.lidar_parse_failures =
    input_adapter == nullptr ? 0U : input_adapter->lidarParseFailures();
  result.lidar_deskew_failures = processor_stats.lidar_deskew_failures;
  result.lidar_timestamp_regressions = current.lidar_timestamp_regressions;
  result.imu_timestamp_regressions = current.imu_timestamp_regressions;
  result.lidar_capacity_drops = current.lidar_capacity_drops;
  result.imu_capacity_drops = current.imu_capacity_drops;
  result.pending_lidar_drops = processor_stats.lidar_pending_drops;
  result.imu_gap_resets = processor_stats.imu_gap_resets;
  result.range_filter_ms = frame.processing.range_filter_time_ms;
  result.voxel_ms = frame.processing.voxel_time_ms;
  result.deskew_ms = frame.processing.deskew_time_ms;
  result.registration_ms = frame.registration_time_ms;
  result.correction_ms = frame.correction_time_ms;
  result.map_delete_ms = frame.map_delete_time_ms;
  result.archive_update_ms = frame.archive_update_time_ms;
  result.map_insert_ms = frame.map_insert_time_ms;
  result.trajectory_reanchor_ms = frame.processing.trajectory_reanchor_time_ms;
  result.repropagation_ms = frame.processing.repropagation_time_ms;
  result.core_ms = frame.processing.core_end_to_end_time_ms;
  result.output_ms = frame.output.output_total_time_ms;
  result.end_to_end_ms = frame.output.end_to_end_time_ms;
  result.local_map_points = frame.map_size;
  result.output_global_map_points = frame.global_map_size;
  result.map_candidates = frame.map_candidate_points;
  result.map_inserted = frame.map_inserted_points;
  result.map_replaced = frame.map_replaced_points;
  result.map_rejected = frame.map_rejected_points;
  result.pending_delete_boxes = frame.pending_delete_boxes;
  result.rebuild_total = frame.rebuild_total;
  result.rebuild_synchronous = frame.rebuild_synchronous;
  result.rebuild_asynchronous = frame.rebuild_asynchronous;
  result.rebuild_last_reason = frame.rebuild_last_reason;
  result.rebuild_last_points = frame.rebuild_last_points;
  result.rebuild_max_points = frame.rebuild_max_points;
  result.rebuild_last_ms = frame.rebuild_last_ms;
  result.rebuild_max_ms = frame.rebuild_max_ms;
  result.rebuild_active = frame.rebuild_active;
  result.backend_keyframes = frame.backend_keyframes;
  result.backend_odometry_constraints = frame.backend_odometry_constraints;
  result.backend_loop_constraints = frame.backend_loop_constraints;
  result.loop_worker_queue_depth = frame.loop_worker.queue_depth;
  result.loop_worker_max_queue_depth = frame.loop_worker.max_queue_depth;
  result.loop_worker_submitted = frame.loop_worker.submitted;
  result.loop_worker_completed = frame.loop_worker.completed;
  result.loop_worker_dropped = frame.loop_worker.dropped;
  result.loop_worker_timed_out = frame.loop_worker.timed_out;
  result.loop_worker_cancelled = frame.loop_worker.cancelled;
  result.backend_worker_queue_depth = frame.backend_worker.queue_depth;
  result.backend_worker_max_queue_depth = frame.backend_worker.max_queue_depth;
  result.backend_worker_submitted = frame.backend_worker.submitted;
  result.backend_worker_completed = frame.backend_worker.completed;
  result.backend_worker_dropped = frame.backend_worker.dropped;
  result.backend_worker_timed_out = frame.backend_worker.timed_out;
  result.backend_worker_cancelled = frame.backend_worker.cancelled;
  const auto memory = processMemoryMib();
  result.resident_memory_mib = memory.first;
  result.peak_resident_memory_mib = memory.second;
  return result;
}

void DiagnosticsReporter::log() {
  const auto current = synchronizer_.stats();
  sensor::SensorBufferStats previous;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    previous = previous_stats_;
    previous_stats_ = current;
  }
  const auto runtime = snapshot();
  const double lidar_hz =
      static_cast<double>(current.lidar_received - previous.lidar_received) /
      period_sec_;
  const double imu_hz =
      static_cast<double>(current.imu_received - previous.imu_received) /
      period_sec_;
  const std::uint64_t drops = runtime.lidar_capacity_drops +
                              runtime.imu_capacity_drops +
                              runtime.pending_lidar_drops;
  RCLCPP_INFO(node_.get_logger(),
              "[LIO STATUS]\n"
              "rate(L/I)=%.1f/%.1fHz\n"
              "queue(L/I)=%zu/%zu\n"
              "processed(L/I)=%" PRIu64 "/%" PRIu64 "\n"
              "deskewed=%" PRIu64 "\n"
              "failures(parse/deskew/drop/gap)=%" PRIu64 "/%" PRIu64 "/%" PRIu64
              "/%" PRIu64,
              lidar_hz, imu_hz, current.lidar_buffer_size,
              current.imu_buffer_size, runtime.lidar_processed,
              runtime.imu_processed, runtime.lidar_deskewed,
              runtime.lidar_parse_failures, runtime.lidar_deskew_failures,
              drops, runtime.imu_gap_resets);
  RCLCPP_INFO(
      node_.get_logger(),
      "[LIO TIMING]\n"
      "pre(range/voxel/deskew)=%.1f/%.1f/%.1fms\n"
      "match(registration/correction)=%.1f/%.1fms\n"
      "map(delete/archive/insert)=%.1f/%.1f/%.1fms\n"
      "post(reanchor/reprop)=%.3f/%.3fms\n"
      "core/output/e2e=%.1f/%.1f/%.1fms",
      runtime.range_filter_ms, runtime.voxel_ms, runtime.deskew_ms,
      runtime.registration_ms, runtime.correction_ms, runtime.map_delete_ms,
      runtime.archive_update_ms, runtime.map_insert_ms,
      runtime.trajectory_reanchor_ms, runtime.repropagation_ms,
      runtime.core_ms, runtime.output_ms, runtime.end_to_end_ms);
  RCLCPP_INFO(node_.get_logger(),
              "[LIO MAP]\n"
              "points(local/global)=%zu/%zu\n"
              "update(candidate/insert/replace/reject)=%zu/%zu/%zu/%zu\n"
              "pending_delete=%zu\n"
              "rebuild(last/max/active)=%.1f/%.1fms/%s",
              runtime.local_map_points, runtime.output_global_map_points,
              runtime.map_candidates, runtime.map_inserted,
              runtime.map_replaced, runtime.map_rejected,
              runtime.pending_delete_boxes, runtime.rebuild_last_ms,
              runtime.rebuild_max_ms,
              runtime.rebuild_active ? "true" : "false");
  RCLCPP_INFO(
      node_.get_logger(),
      "[LIO WORKERS]\n"
      "loop(depth/max submitted/completed/drop/timeout/cancel)=%zu/%zu %" PRIu64
      "/%" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64 "\n"
      "backend(depth/max submitted/completed/drop/timeout/cancel)=%zu/%zu %" PRIu64
      "/%" PRIu64 "/%" PRIu64 "/%" PRIu64 "/%" PRIu64,
      runtime.loop_worker_queue_depth, runtime.loop_worker_max_queue_depth,
      runtime.loop_worker_submitted, runtime.loop_worker_completed,
      runtime.loop_worker_dropped, runtime.loop_worker_timed_out,
      runtime.loop_worker_cancelled, runtime.backend_worker_queue_depth,
      runtime.backend_worker_max_queue_depth, runtime.backend_worker_submitted,
      runtime.backend_worker_completed, runtime.backend_worker_dropped,
      runtime.backend_worker_timed_out, runtime.backend_worker_cancelled);
}

} // namespace itera_lio::ros
