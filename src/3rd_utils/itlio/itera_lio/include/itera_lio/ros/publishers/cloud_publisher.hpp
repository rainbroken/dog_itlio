// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__PUBLISHERS__CLOUD_PUBLISHER_HPP_
#define ITERA_LIO__ROS__PUBLISHERS__CLOUD_PUBLISHER_HPP_

#include "itera_lio/common/frame_metrics.hpp"
#include "itera_lio/common/observation_types.hpp"
#include "itera_lio/frontend/ikd_map.hpp"
#include "itera_lio/ros/publishers/output_options.hpp"
#include "itera_lio/sensor/lidar_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace itera_lio::ros {

class CloudPublisher {
public:
  CloudPublisher(rclcpp::Node &node, const OutputPublisherOptions &options,
                 const common::FixedExtrinsics &extrinsics);
  void publishDeskewedCloud(const sensor::LidarFrame &scan, common::FrameOutputTiming &timing);
  void publishRegisteredCloud(const sensor::LidarFrame &scan, const estimator::NominalState &state,
                              common::FrameOutputTiming &timing);
  void publishEffectiveCloud(const common::ObservationBatch &observations, double timestamp,
                             common::FrameOutputTiming &timing);
  void publishLocalMap(const frontend::IkdMap &map, double timestamp,
                       common::FrameOutputTiming &timing);
  void publishMapUpdates(double timestamp, const std::vector<common::MapPoint> &updates,
                         common::FrameOutputTiming &timing);

private:
  void
  publishMapPoints(const std::vector<common::MapPoint> &points,
                   const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
                   double timestamp, common::FrameOutputTiming &timing);
  template <typename MapType>
  void publishMap(const MapType &map,
                  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
                  double timestamp, common::FrameOutputTiming &timing);
  OutputPublisherOptions options_;
  common::FixedExtrinsics extrinsics_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_updates_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr effective_publisher_;
};

} // namespace itera_lio::ros

#endif // ITERA_LIO__ROS__PUBLISHERS__CLOUD_PUBLISHER_HPP_
