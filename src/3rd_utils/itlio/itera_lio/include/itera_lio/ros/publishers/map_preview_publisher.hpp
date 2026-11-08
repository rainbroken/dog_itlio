// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__PUBLISHERS__MAP_PREVIEW_PUBLISHER_HPP_
#define ITERA_LIO__ROS__PUBLISHERS__MAP_PREVIEW_PUBLISHER_HPP_

#include "itera_lio/mapping/color/colored_global_map.hpp"
#include "itera_lio/output/preview_cloud.hpp"
#include "itera_lio/ros/publishers/output_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

namespace itera_lio::ros {

class MapPreviewPublisher {
public:
  MapPreviewPublisher(rclcpp::Node &node, const OutputPublisherOptions &options);
  ~MapPreviewPublisher();
  void stop();
  void requestGlobalMapSnapshot(double timestamp, const std::vector<common::MapPoint> &updates);
  void requestColoredSnapshot(double timestamp);
  void addColoredPoints(const std::vector<mapping::color::ColoredPoint> &points, double timestamp);
  std::size_t globalMapSize() const;
  bool getColoredTile(std::int64_t x, std::int64_t y, std::int64_t z, std::size_t max_points,
                      std::vector<mapping::color::ColoredPoint> &points);

private:
  void globalMapSnapshotLoop();
  rclcpp::Node &node_;
  OutputPublisherOptions options_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_updates_publisher_;
  double last_global_snapshot_timestamp_{-1.0};
  double last_colored_global_snapshot_timestamp_{-1.0};
  output::preview::PreviewMap global_preview_map_;
  mapping::color::ColoredGlobalMap colored_preview_map_;
  std::atomic<double> latest_global_snapshot_timestamp_{-1.0};
  std::mutex global_snapshot_mutex_;
  struct GlobalPreviewTask {
    std::vector<common::MapPoint> points;
    double timestamp{0.0};
  };
  std::deque<GlobalPreviewTask> global_preview_tasks_;
  static constexpr std::size_t kGlobalPreviewTaskCapacity = 8U;
  std::condition_variable global_snapshot_cv_;
  std::uint64_t global_snapshot_generation_{0U};
  bool stop_global_snapshot_thread_{false};
  std::thread global_snapshot_thread_;
};

} // namespace itera_lio::ros

#endif // ITERA_LIO__ROS__PUBLISHERS__MAP_PREVIEW_PUBLISHER_HPP_
