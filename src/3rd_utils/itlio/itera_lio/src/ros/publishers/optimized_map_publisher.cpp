// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/message_conversions.hpp"
#include "itera_lio/ros/publishers/optimized_map_publisher.hpp"
#include <chrono>

namespace itera_lio::ros {
OptimizedMapPublisher::OptimizedMapPublisher(rclcpp::Node &node,
                                             const OutputPublisherOptions &options,
                                             const system::LioProcessor &processor,
                                             output::ScanArchive &archive)
    : node_(node), options_(options), processor_(processor), archive_(archive) {
  optimized_map_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
      options_.optimized_map_topic, rclcpp::QoS(1).transient_local().reliable());
  optimized_archive_publisher_ = node_.create_publisher<std_msgs::msg::String>(
      "/lio/optimized_colored_archive", rclcpp::QoS(1).transient_local().reliable());
  if (options_.publish_optimized_map) {
    optimized_map_thread_ = std::thread(&OptimizedMapPublisher::optimizedMapLoop, this);
  }
}
OptimizedMapPublisher::~OptimizedMapPublisher() { stopOptimizedMap(); }
void OptimizedMapPublisher::requestOptimizedMap(const double timestamp) {
  {
    std::lock_guard<std::mutex> lock(optimized_map_mutex_);
    optimized_map_timestamp_ = timestamp;
    optimized_map_pending_ = true; // Coalesce requests; never queue whole maps.
  }
  optimized_map_cv_.notify_one();
}

void OptimizedMapPublisher::stopOptimizedMap() {
  {
    std::lock_guard<std::mutex> lock(optimized_map_mutex_);
    stop_optimized_map_ = true;
  }
  optimized_map_cv_.notify_one();
  if (optimized_map_thread_.joinable()) {
    optimized_map_thread_.join();
  }
}

void OptimizedMapPublisher::optimizedMapLoop() {
  while (true) {
    double timestamp;
    {
      std::unique_lock<std::mutex> lock(optimized_map_mutex_);
      optimized_map_cv_.wait(lock,
                             [this]() { return stop_optimized_map_ || optimized_map_pending_; });
      if (stop_optimized_map_) {
        return;
      }
      timestamp = optimized_map_timestamp_;
      optimized_map_pending_ = false;
    }
    const auto start = std::chrono::steady_clock::now();
    try {
      common::FrameOutputTiming timing;
      publishOptimizedMap(timestamp, timing);
      if (optimized_archive_publisher_->get_subscription_count() > 0U) {
        const auto [raw, optimized] = processor_.backend().previewKeyframes();
        if (archive_.hasColoredScans()) {
          const auto result = archive_.optimizedPreview(raw, optimized);
          if (result.success) {
            std_msgs::msg::String notice;
            notice.data = std::to_string(timestamp) + "\n" + result.output_path;
            optimized_archive_publisher_->publish(notice);
            RCLCPP_INFO(node_.get_logger(),
                        "Optimized preview blocks ready: %zu points, manifest='%s'",
                        result.points_written, result.output_path.c_str());
          } else {
            RCLCPP_WARN(node_.get_logger(), "Optimized preview archive: %s", result.error.c_str());
          }
        }
      }
      RCLCPP_INFO(
          node_.get_logger(), "Optimized map background output: %.3f ms",
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
              .count());
    } catch (const std::exception &error) {
      RCLCPP_ERROR(node_.get_logger(), "Optimized map output failed: %s", error.what());
    }
  }
}

void OptimizedMapPublisher::publishOptimizedMap(const double timestamp,
                                                common::FrameOutputTiming &timing) {
  (void)timing;
  if (optimized_map_publisher_->get_subscription_count() == 0U) {
    return;
  }
  std::vector<common::MapPoint> points;
  if (processor_.getOptimizedMapPoints(points, options_.optimized_map_voxel_size)) {
    optimized_map_publisher_->publish(toMapPointCloud2(points, timestamp, options_.world_frame));
  }
}

} // namespace itera_lio::ros
