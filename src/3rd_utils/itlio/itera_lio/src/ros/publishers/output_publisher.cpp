// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/diagnostics_reporter.hpp"
#include "itera_lio/ros/publishers/cloud_publisher.hpp"
#include "itera_lio/ros/publishers/colored_cloud_publisher.hpp"
#include "itera_lio/ros/map_export_controller.hpp"
#include "itera_lio/ros/publishers/map_preview_publisher.hpp"
#include "itera_lio/ros/publishers/optimized_map_publisher.hpp"
#include "itera_lio/ros/publishers/output_publisher.hpp"
#include "itera_lio/ros/publishers/state_publisher.hpp"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <tuple>
#include <stdexcept>

namespace itera_lio::ros {
LioOutputPublisher::LioOutputPublisher(rclcpp::Node &node, const OutputPublisherOptions &options,
                                       system::LioProcessor &processor)
    : node_(node), options_(options), processor_(processor) {
  if (!std::isfinite(options_.registered_voxel_size) || options_.registered_voxel_size <= 0.0F ||
      !std::isfinite(options_.global_voxel_size) || options_.global_voxel_size <= 0.0F ||
      options_.max_preview_points == 0U || options_.global_map_max_points == 0U ||
      !std::isfinite(options_.optimized_map_voxel_size) ||
      options_.optimized_map_voxel_size <= 0.0F ||
      !std::isfinite(options_.path_publish_frequency_hz) ||
      options_.path_publish_frequency_hz <= 0.0 ||
      !std::isfinite(options_.global_map_snapshot_period_sec) ||
      options_.global_map_snapshot_period_sec < 0.0 ||
      !std::isfinite(options_.image_sync_tolerance_sec) ||
      options_.image_sync_tolerance_sec <= 0.0 || !std::isfinite(options_.image_sync_wait_sec) ||
      options_.image_sync_wait_sec < 0.0 ||
      (options_.image_qos_reliability != "reliable" &&
       options_.image_qos_reliability != "best_effort") ||
      options_.image_qos_depth == 0U || options_.image_queue_capacity == 0U ||
      options_.pcd_directory.empty()) {
    throw std::invalid_argument("invalid LIO output publisher options");
  }
  if (options_.pcd_source != "auto" && options_.pcd_source != "full_deskewed") {
    throw std::invalid_argument("save_pcd.source must be auto or full_deskewed");
  }
  if (options_.pcd_source == "full_deskewed" && !options_.full_deskewed_enabled) {
    throw std::invalid_argument("full_deskewed PCD source requires full scan output");
  }
  const bool archive_enabled = std::getenv("ITERA_LIO_NO_ARCHIVE") == nullptr;
  archive_ = std::make_unique<output::ScanArchive>(output::ScanArchiveOptions{
      archive_enabled && options_.livo_enabled,
      archive_enabled && options_.full_deskewed_enabled &&
          (!options_.livo_enabled || options_.refinement_archive), options_.pcd_directory,
      options_.pcd_voxel_size, options_.static_filter});
  if (!archive_->initializationError().empty()) {
    RCLCPP_ERROR(node_.get_logger(), "%s PCD writer disabled: %s",
                 options_.livo_enabled ? "Dense colored" : "Full deskewed",
                 archive_->initializationError().c_str());
  }
  state_ = std::make_unique<StatePublisher>(node_, options_, processor_.lidarExtrinsics());
  auto local_options = options_;
  local_options.world_frame = options_.odom_frame;
  clouds_ = std::make_unique<CloudPublisher>(node_, local_options, processor_.lidarExtrinsics());
  preview_ = std::make_unique<MapPreviewPublisher>(node_, local_options);
  exports_ = std::make_unique<MapExportController>(node_, options_, processor_, *archive_);
  optimized_ = std::make_unique<OptimizedMapPublisher>(node_, options_, processor_, *archive_);
  colored_ = std::make_unique<ColoredCloudPublisher>(
      node_, local_options, processor_.lidarExtrinsics(),
      [this](const auto &points, const auto &state, const bool trusted, const double timestamp) {
        if (!trusted) {
          return;
        }
        if (archive_->hasColoredScans()) {
          const auto result =
              archive_->appendColored(points, timestamp, common::Pose{state.R_WI, state.p_WI},
                                      processor_.lidarExtrinsics());
          if (!result.success) {
            RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                                  "Dense colored PCD append failed: %s", result.error.c_str());
          }
        }
        preview_->addColoredPoints(points, timestamp);
      },
      [this](const double timestamp) { preview_->requestColoredSnapshot(timestamp); });
}
LioOutputPublisher::~LioOutputPublisher() { stopWorkers(); }

void LioOutputPublisher::stopWorkers() {
  optimized_->stopOptimizedMap();
  colored_->stop();
  preview_->stop();
}

common::FrameOutputTiming LioOutputPublisher::publishFrame(const system::ProcessedFrame &frame) {
  common::FrameOutputTiming timing;
  const auto output_start = std::chrono::steady_clock::now();
  const auto &deskewed = frame.deskewed;
  const auto &display_scan = options_.full_deskewed_enabled && !frame.full_deskewed.points.empty()
                                 ? frame.full_deskewed
                                 : deskewed;
  if (options_.publish_deskewed) {
    clouds_->publishDeskewedCloud(display_scan, timing);
  }
  if (options_.livo_enabled) {
    colored_->enqueueColoredFrame(
        display_scan, frame.state_at_scan_end,
        frame.pipeline.initialized_map ||
            (frame.pipeline.tracking.state == estimator::TrackingState::kNormal &&
             (frame.pipeline.map_insert_reason == "accepted" || frame.pipeline.map_updated)),
        deskewed.scan_end_time);
  }
  if (options_.publish_registered) {
    clouds_->publishRegisteredCloud(display_scan, frame.state_at_scan_end, timing);
  }
  if (options_.publish_effective) {
    clouds_->publishEffectiveCloud(frame.pipeline.registration.observations, deskewed.scan_end_time,
                                   timing);
  }
  if (options_.publish_local_map) {
    clouds_->publishLocalMap(processor_.localMap(), deskewed.scan_end_time, timing);
  }
  if (options_.publish_map_updates) {
    clouds_->publishMapUpdates(deskewed.scan_end_time, frame.pipeline.map_updates, timing);
  }
  if (options_.publish_global_map) {
    if (!options_.livo_enabled) {
      preview_->requestGlobalMapSnapshot(deskewed.scan_end_time, frame.pipeline.map_updates);
    }
  }
  state_->publishState(deskewed.scan_end_time, frame.state_at_scan_end, timing,
                       processor_.backend().mapToOdometry());
  if (frame.timing.callback_start_steady_sec > 0.0) {
    timing.callback_to_state_publish_ms =
      (std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count() -
       frame.timing.callback_start_steady_sec) * 1000.0;
  }
  if (frame.pipeline.backend.optimization.optimized ||
      (frame.pipeline.backend.keyframe_added &&
       frame.pipeline.backend.loop_constraint_count > 0U)) {
    state_->publishOptimizedPath(deskewed.scan_end_time, processor_.backend().optimizedKeyframes(),
                                 timing);
    const auto now = std::chrono::steady_clock::now();
    if (options_.publish_optimized_map &&
        (frame.pipeline.backend.optimization.optimized ||
         now - last_preview_request_ >= std::chrono::seconds(5))) {
      last_preview_request_ = now;
      optimized_->requestOptimizedMap(deskewed.scan_end_time);
    }
  }
  if (archive_->hasFullScans() && !frame.full_deskewed.points.empty()) {
    const auto result = archive_->appendFull(
        frame.full_deskewed,
        common::Pose{frame.state_at_scan_end.R_WI, frame.state_at_scan_end.p_WI},
        processor_.lidarExtrinsics(), &timing);
    if (!result.success) {
      RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                            "Full deskewed PCD append failed: %s", result.error.c_str());
    }
  }
  std::tie(timing.resident_memory_mib, timing.peak_resident_memory_mib) = processMemoryMib();
  timing.output_total_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - output_start)
          .count();
  return timing;
}

void LioOutputPublisher::noteLidarArrival(double timestamp) {
  colored_->noteLidarArrival(timestamp);
}
std::size_t LioOutputPublisher::globalMapSize() const { return preview_->globalMapSize(); }
bool LioOutputPublisher::getColoredTile(std::int64_t x, std::int64_t y, std::int64_t z,
                                        std::size_t max_points,
                                        std::vector<mapping::color::ColoredPoint> &points) {
  return preview_->getColoredTile(x, y, z, max_points, points);
}
bool LioOutputPublisher::saveSnapshot(std::string &message) {
  return exports_->saveSnapshot(message);
}
void LioOutputPublisher::publishRefinedMap(double timestamp) {
  common::FrameOutputTiming timing;
  state_->publishOptimizedPath(timestamp, processor_.backend().optimizedKeyframes(), timing);
  optimized_->requestOptimizedMap(timestamp);
}
bool LioOutputPublisher::savePcd() {
  stopWorkers();
  if (options_.save_pcd_on_shutdown && options_.livo_enabled) {
    colored_->logStats();
  }
  return exports_->savePcd(preview_->globalMapSize());
}

} // namespace itera_lio::ros
