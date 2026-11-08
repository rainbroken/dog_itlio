// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__PUBLISHERS__COLORED_CLOUD_PUBLISHER_HPP_
#define ITERA_LIO__ROS__PUBLISHERS__COLORED_CLOUD_PUBLISHER_HPP_

#include "itera_lio/common/frame_metrics.hpp"
#include "itera_lio/estimator/state.hpp"
#include "itera_lio/ros/publishers/output_options.hpp"
#include "itera_lio/sensor/lidar_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

namespace itera_lio::ros {

class ColoredCloudPublisher {
public:
  using ColoredCallback = std::function<void(const std::vector<mapping::color::ColoredPoint> &,
                                             const estimator::NominalState &, bool, double)>;
  using CompletedCallback = std::function<void(double)>;
  ColoredCloudPublisher(rclcpp::Node &node, const OutputPublisherOptions &options,
                        const common::FixedExtrinsics &extrinsics, ColoredCallback colored,
                        CompletedCallback completed);
  ~ColoredCloudPublisher();
  void stop();
  // Call logStats only after stop() has drained and joined the worker.
  void logStats();
  void noteLidarArrival(double scan_end_time);
  void enqueueColoredFrame(const sensor::LidarFrame &scan, const estimator::NominalState &state,
                           bool trusted, double timestamp);

private:
  void coloredOutputLoop();
  void publishColoredCloud(const sensor::LidarFrame &scan, const estimator::NominalState &state,
                           bool trusted, double timestamp, common::FrameOutputTiming &timing);
  rclcpp::Node &node_;
  OutputPublisherOptions options_;
  common::FixedExtrinsics extrinsics_;
  ColoredCallback colored_;
  CompletedCallback completed_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_publisher_;
  std::unique_ptr<output::ImageRecorder> image_recorder_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  std::mutex image_mutex_;
  double latest_image_timestamp_{-1.0};
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> image_queue_;
  struct PendingImageMatch {
    double lidar_timestamp{0.0};
    sensor_msgs::msg::Image::ConstSharedPtr image;
  };
  std::deque<PendingImageMatch> pending_image_matches_;
  std::size_t image_match_retention_dropped_{0U};
  static constexpr std::size_t kPendingImageMatchCapacity = 128U;
  std::size_t colored_frames_attempted_{0U};
  std::size_t colored_frames_matched_{0U};
  std::size_t colored_frames_sync_missed_{0U};
  std::size_t colored_frames_empty_{0U};
  std::size_t colored_points_projected_{0U};
  std::size_t image_frames_received_{0U};
  std::size_t image_queue_dropped_{0U};
  struct ColoredTask {
    sensor::LidarFrame scan;
    estimator::NominalState state;
    bool trusted{false};
    double timestamp{0.0};
  };
  std::mutex colored_task_mutex_;
  std::condition_variable colored_task_cv_;
  std::deque<ColoredTask> colored_tasks_;
  std::thread colored_thread_;
  bool stop_colored_thread_{false};
  std::size_t colored_task_dropped_{0U};
  // Synchronization and frame retention share one bounded budget. Frames
  // waiting for delayed images are not an output-computation backlog.
  static constexpr std::size_t kColoredTaskCapacity = kPendingImageMatchCapacity;
};

} // namespace itera_lio::ros

#endif // ITERA_LIO__ROS__PUBLISHERS__COLORED_CLOUD_PUBLISHER_HPP_
