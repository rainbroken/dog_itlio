// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/message_conversions.hpp"
#include "itera_lio/ros/publishers/map_preview_publisher.hpp"
#include <chrono>
#include <utility>

namespace itera_lio::ros {
MapPreviewPublisher::MapPreviewPublisher(rclcpp::Node &node, const OutputPublisherOptions &options)
    : node_(node), options_(options),
      global_preview_map_(options.global_voxel_size, options.global_map_max_points),
      colored_preview_map_(options.global_voxel_size, options.global_map_max_points) {
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  global_map_publisher_ =
      node_.create_publisher<sensor_msgs::msg::PointCloud2>(options_.global_map_topic, qos);
  if (options_.livo_enabled) {
    colored_updates_publisher_ =
        node_.create_publisher<sensor_msgs::msg::PointCloud2>("/lio/colored_map_updates", qos);
  }
  if (options_.publish_global_map) {
    global_snapshot_thread_ = std::thread(&MapPreviewPublisher::globalMapSnapshotLoop, this);
  }
}
MapPreviewPublisher::~MapPreviewPublisher() { stop(); }
void MapPreviewPublisher::stop() {
  {
    std::lock_guard<std::mutex> lock(global_snapshot_mutex_);
    stop_global_snapshot_thread_ = true;
  }
  global_snapshot_cv_.notify_one();
  if (global_snapshot_thread_.joinable()) {
    global_snapshot_thread_.join();
  }
}

void MapPreviewPublisher::requestGlobalMapSnapshot(const double timestamp,
                                                   const std::vector<common::MapPoint> &updates) {
  if (updates.empty()) {
    return;
  }
  GlobalPreviewTask task{output::preview::sampleMapPoints(updates, options_.global_voxel_size,
                                                          options_.max_preview_points),
                         timestamp};
  bool dropped = false;
  {
    std::lock_guard<std::mutex> lock(global_snapshot_mutex_);
    if (global_preview_tasks_.size() >= kGlobalPreviewTaskCapacity) {
      global_preview_tasks_.pop_front();
      dropped = true;
    }
    global_preview_tasks_.push_back(std::move(task));
  }
  if (dropped) {
    RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "LIO preview queue full: dropping preview delta; PCD archive is unaffected");
  }
  global_snapshot_cv_.notify_one();
}

void MapPreviewPublisher::globalMapSnapshotLoop() {
  std::uint64_t handled_generation = 0U;
  while (true) {
    double timestamp = -1.0;
    std::uint64_t generation = 0U;
    std::deque<GlobalPreviewTask> preview_tasks;
    {
      std::unique_lock<std::mutex> lock(global_snapshot_mutex_);
      global_snapshot_cv_.wait(lock, [this, handled_generation]() {
        return stop_global_snapshot_thread_ || !global_preview_tasks_.empty() ||
               global_snapshot_generation_ != handled_generation;
      });
      if (stop_global_snapshot_thread_) {
        return;
      }
      generation = global_snapshot_generation_;
      timestamp = latest_global_snapshot_timestamp_.load();
      handled_generation = generation;
      preview_tasks.swap(global_preview_tasks_);
    }

    // Only this worker touches the LIO preview. Map insertion and complete
    // snapshots must never hold up consumption of LiDAR or IMU measurements.
    if (!options_.livo_enabled) {
      for (const auto &task : preview_tasks) {
        global_preview_map_.addPoints(task.points);
        timestamp = task.timestamp;
      }
      if (preview_tasks.empty() || global_map_publisher_->get_subscription_count() == 0U ||
          (last_global_snapshot_timestamp_ >= 0.0 &&
           (options_.global_map_snapshot_period_sec <= 0.0 ||
            timestamp <
                last_global_snapshot_timestamp_ + options_.global_map_snapshot_period_sec))) {
        continue;
      }
      last_global_snapshot_timestamp_ = timestamp;
    }

    sensor_msgs::msg::PointCloud2 message;
    if (options_.livo_enabled) {
      std::vector<mapping::color::ColoredPoint> snapshot;
      {
        if (!colored_preview_map_.getAllPoints(snapshot)) {
          continue;
        }
      }
      message = toColoredPointCloud2(snapshot, timestamp, options_.world_frame);
    } else {
      std::vector<common::MapPoint> snapshot;
      if (!global_preview_map_.getAllPoints(snapshot)) {
        continue;
      }
      message = toMapPointCloud2(snapshot, timestamp, options_.world_frame);
    }
    {
      std::lock_guard<std::mutex> lock(global_snapshot_mutex_);
      if (stop_global_snapshot_thread_) {
        return;
      }
    }
    // Publish the completed snapshot even if newer updates arrived while it
    // was built. The next iteration will catch up; discarding here can starve
    // publication indefinitely for large maps.
    global_map_publisher_->publish(message);
  }
}

std::size_t MapPreviewPublisher::globalMapSize() const {
  return options_.livo_enabled ? colored_preview_map_.size() : global_preview_map_.size();
}

bool MapPreviewPublisher::getColoredTile(const std::int64_t x, const std::int64_t y,
                                         const std::int64_t z, const std::size_t max_points,
                                         std::vector<mapping::color::ColoredPoint> &points) {
  if (!options_.livo_enabled || max_points == 0U) {
    return false;
  }
  return colored_preview_map_.getChunkPoints(x, y, z, max_points, points);
}

void MapPreviewPublisher::requestColoredSnapshot(const double timestamp) {
  if (options_.publish_global_map && global_map_publisher_->get_subscription_count() > 0U &&
      (last_colored_global_snapshot_timestamp_ < 0.0 ||
       (options_.global_map_snapshot_period_sec > 0.0 &&
        timestamp >=
            last_colored_global_snapshot_timestamp_ + options_.global_map_snapshot_period_sec))) {
    last_colored_global_snapshot_timestamp_ = timestamp;
    latest_global_snapshot_timestamp_.store(timestamp);
    {
      std::lock_guard<std::mutex> lock(global_snapshot_mutex_);
      ++global_snapshot_generation_;
    }
    global_snapshot_cv_.notify_one();
  }
}
void MapPreviewPublisher::addColoredPoints(const std::vector<mapping::color::ColoredPoint> &points,
                                           const double timestamp) {
  std::vector<mapping::color::ColoredPoint> changed_points;
  if (colored_updates_publisher_->get_subscription_count() > 0U) {
    mapping::color::ColoredGlobalMapUpdate map_update;
    colored_preview_map_.addPoints(points, &map_update);
    changed_points = std::move(map_update.upserted_points);
  } else {
    colored_preview_map_.addPoints(points);
  }
  if (!changed_points.empty()) {
    colored_updates_publisher_->publish(
        toColoredPointCloud2(changed_points, timestamp, options_.world_frame));
  }
}

} // namespace itera_lio::ros
