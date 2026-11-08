// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__PUBLISHERS__OPTIMIZED_MAP_PUBLISHER_HPP_
#define ITERA_LIO__ROS__PUBLISHERS__OPTIMIZED_MAP_PUBLISHER_HPP_

#include "itera_lio/output/scan_archive.hpp"
#include "itera_lio/ros/publishers/output_options.hpp"
#include "itera_lio/system/lio_processor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include <condition_variable>
#include <mutex>
#include <thread>

namespace itera_lio::ros {

class OptimizedMapPublisher {
public:
  OptimizedMapPublisher(rclcpp::Node &node, const OutputPublisherOptions &options,
                        const system::LioProcessor &processor, output::ScanArchive &archive);
  ~OptimizedMapPublisher();
  void requestOptimizedMap(double timestamp);
  void stopOptimizedMap();

private:
  void optimizedMapLoop();
  void publishOptimizedMap(double timestamp, common::FrameOutputTiming &timing);
  rclcpp::Node &node_;
  OutputPublisherOptions options_;
  const system::LioProcessor &processor_;
  output::ScanArchive &archive_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr optimized_archive_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr optimized_map_publisher_;
  std::mutex optimized_map_mutex_;
  std::condition_variable optimized_map_cv_;
  std::thread optimized_map_thread_;
  bool stop_optimized_map_{false};
  bool optimized_map_pending_{false};
  double optimized_map_timestamp_{0.0};
};

} // namespace itera_lio::ros

#endif // ITERA_LIO__ROS__PUBLISHERS__OPTIMIZED_MAP_PUBLISHER_HPP_
