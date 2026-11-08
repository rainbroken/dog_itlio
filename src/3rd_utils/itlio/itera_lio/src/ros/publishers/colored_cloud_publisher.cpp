// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/message_conversions.hpp"
#include "itera_lio/ros/publishers/colored_cloud_publisher.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace itera_lio::ros {
namespace {
double imageTimestamp(const sensor_msgs::msg::Image &image) {
  return static_cast<double>(image.header.stamp.sec) +
         static_cast<double>(image.header.stamp.nanosec) * 1e-9;
}
} // namespace
ColoredCloudPublisher::ColoredCloudPublisher(rclcpp::Node &node,
                                             const OutputPublisherOptions &options,
                                             const common::FixedExtrinsics &extrinsics,
                                             ColoredCallback colored, CompletedCallback completed)
    : node_(node), options_(options), extrinsics_(extrinsics), colored_(std::move(colored)),
      completed_(std::move(completed)) {
  image_recorder_ = std::make_unique<output::ImageRecorder>(options_.image_recorder);
  const auto preview_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  if (options_.livo_enabled) {
    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(options_.image_qos_depth));
    if (options_.image_qos_reliability == "reliable") {
      image_qos.reliable();
    } else {
      image_qos.best_effort();
    }
    colored_publisher_ =
        node_.create_publisher<sensor_msgs::msg::PointCloud2>(options_.colored_topic, preview_qos);
    image_subscription_ = node_.create_subscription<sensor_msgs::msg::Image>(
        options_.image_topic, image_qos, [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          if (image_recorder_)
            image_recorder_->submit(msg);
          {
            std::lock_guard<std::mutex> lock(image_mutex_);
            ++image_frames_received_;
            auto best = pending_image_matches_.end();
            double best_distance = options_.image_sync_tolerance_sec;
            const double timestamp = imageTimestamp(*msg);
            if (std::isfinite(timestamp)) {
              latest_image_timestamp_ = std::max(latest_image_timestamp_, timestamp);
            }
            for (auto it = pending_image_matches_.begin(); it != pending_image_matches_.end();
                 ++it) {
              const double distance = std::abs(it->lidar_timestamp - timestamp);
              if (!it->image && std::isfinite(distance) && distance <= best_distance) {
                best = it;
                best_distance = distance;
              }
            }
            if (best != pending_image_matches_.end()) {
              best->image = std::move(msg);
            } else {
              image_queue_.push_back(std::move(msg));
              while (image_queue_.size() > options_.image_queue_capacity) {
                image_queue_.pop_front();
                ++image_queue_dropped_;
              }
            }
          }
          // Release image_mutex_ before taking the waiter's mutex. This
          // prevents both lock inversion and a lost readiness notification.
          std::lock_guard<std::mutex> task_lock(colored_task_mutex_);
          colored_task_cv_.notify_one();
        });
  }
  if (options_.livo_enabled) {
    colored_thread_ = std::thread(&ColoredCloudPublisher::coloredOutputLoop, this);
  }
}
ColoredCloudPublisher::~ColoredCloudPublisher() { stop(); }
void ColoredCloudPublisher::stop() {
  {
    std::lock_guard<std::mutex> lock(colored_task_mutex_);
    stop_colored_thread_ = true;
  }
  colored_task_cv_.notify_one();
  if (colored_thread_.joinable()) {
    colored_thread_.join();
  }
}

void ColoredCloudPublisher::noteLidarArrival(const double scan_end_time) {
  if (!options_.livo_enabled) {
    return;
  }
  std::lock_guard<std::mutex> lock(image_mutex_);
  sensor_msgs::msg::Image::ConstSharedPtr selected;
  auto best = image_queue_.end();
  double best_distance = options_.image_sync_tolerance_sec;
  for (auto it = image_queue_.begin(); it != image_queue_.end(); ++it) {
    const double distance = std::abs(imageTimestamp(**it) - scan_end_time);
    if (std::isfinite(distance) && distance <= best_distance) {
      best = it;
      best_distance = distance;
    }
  }
  if (best != image_queue_.end()) {
    selected = *best;
    image_queue_.erase(best);
  }
  pending_image_matches_.push_back({scan_end_time, std::move(selected)});
  while (pending_image_matches_.size() > kPendingImageMatchCapacity) {
    pending_image_matches_.pop_front();
    ++image_match_retention_dropped_;
  }
}

void ColoredCloudPublisher::enqueueColoredFrame(const sensor::LidarFrame &scan,
                                                const estimator::NominalState &state,
                                                const bool trusted, const double timestamp) {
  std::lock_guard<std::mutex> lock(colored_task_mutex_);
  if (stop_colored_thread_) {
    return;
  }
  if (colored_tasks_.size() >= kColoredTaskCapacity) {
    ++colored_task_dropped_;
    RCLCPP_ERROR_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                          "Colored scan queue full: %zu scans lost from the colored PCD archive",
                          colored_task_dropped_);
    return;
  }
  colored_tasks_.push_back(ColoredTask{scan, state, trusted, timestamp});
  colored_task_cv_.notify_one();
}

void ColoredCloudPublisher::coloredOutputLoop() {
  while (true) {
    ColoredTask task;
    {
      std::unique_lock<std::mutex> lock(colored_task_mutex_);
      colored_task_cv_.wait(lock, [this] {
        if (stop_colored_thread_) {return true;}
        if (colored_tasks_.empty()) {return false;}
        const double timestamp = colored_tasks_.front().timestamp;
        std::lock_guard<std::mutex> image_lock(image_mutex_);
        // Initialization/discarded scans have no output task. They must not
        // consume retention capacity forever ahead of the oldest real task.
        while (!pending_image_matches_.empty() &&
          pending_image_matches_.front().lidar_timestamp < timestamp - 1e-6)
        {
          pending_image_matches_.pop_front();
        }
        const auto matched = std::find_if(pending_image_matches_.begin(), pending_image_matches_.end(),
          [timestamp](const PendingImageMatch &entry) {
            return std::abs(entry.lidar_timestamp - timestamp) < 1e-6;
          });
        if (matched != pending_image_matches_.end() && matched->image) {return true;}
        // Sensor-time watermark, with a bounded reordering margin. Wall-clock
        // transport delays do not invalidate an otherwise legal association.
        return latest_image_timestamp_ > timestamp + options_.image_sync_tolerance_sec +
          options_.image_sync_wait_sec;
      });
      if (colored_tasks_.empty()) {
        return;
      }
      task = std::move(colored_tasks_.front());
      colored_tasks_.pop_front();
    }
    common::FrameOutputTiming timing;
    publishColoredCloud(task.scan, task.state, task.trusted, task.timestamp, timing);
    completed_(task.timestamp);
  }
}

void ColoredCloudPublisher::publishColoredCloud(const sensor::LidarFrame &scan,
                                                const estimator::NominalState &state,
                                                const bool trusted, const double timestamp,
                                                common::FrameOutputTiming &timing) {
  if (!colored_publisher_) {
    return;
  }
  ++colored_frames_attempted_;
  sensor_msgs::msg::Image::ConstSharedPtr image;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    const auto matched = std::find_if(pending_image_matches_.begin(), pending_image_matches_.end(),
                                      [timestamp](const PendingImageMatch &entry) {
                                        return std::abs(entry.lidar_timestamp - timestamp) < 1e-6;
                                      });
    if (matched != pending_image_matches_.end() && matched->image) {
      image = matched->image;
      pending_image_matches_.erase(matched);
    } else if (matched != pending_image_matches_.end()) {
      pending_image_matches_.erase(matched);
    }
  }
  if (!image) {
    ++colored_frames_sync_missed_;
    return;
  }
  ++colored_frames_matched_;
  std::vector<mapping::color::ColoredPoint> points;
  if (!mapping::color::colorize(scan, state, extrinsics_, *image, options_.camera, points) ||
      points.empty()) {
    ++colored_frames_empty_;
    return;
  }
  colored_points_projected_ += points.size();
  colored_(points, state, trusted, timestamp);
  auto message = toColoredPointCloud2(points, timestamp, options_.world_frame);
  const auto start = std::chrono::steady_clock::now();
  colored_publisher_->publish(message);
  timing.publish_call_time_ms +=
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void ColoredCloudPublisher::logStats() {
  std::size_t image_frames_received = 0U;
  std::size_t image_queue_dropped = 0U;
  std::size_t image_queue_pending = 0U;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    image_frames_received = image_frames_received_;
    image_queue_dropped = image_queue_dropped_;
    image_queue_pending = image_queue_.size();
  }
  RCLCPP_INFO(node_.get_logger(),
              "Colored sync: image_received=%zu attempted=%zu matched=%zu "
              "sync_missed=%zu projection_empty=%zu projected_points=%zu "
              "image_queue_dropped=%zu image_queue_pending=%zu "
              "image_match_retention_dropped=%zu colored_task_dropped=%zu",
              image_frames_received, colored_frames_attempted_, colored_frames_matched_,
              colored_frames_sync_missed_, colored_frames_empty_, colored_points_projected_,
              image_queue_dropped, image_queue_pending, image_match_retention_dropped_,
              colored_task_dropped_);
}

} // namespace itera_lio::ros
