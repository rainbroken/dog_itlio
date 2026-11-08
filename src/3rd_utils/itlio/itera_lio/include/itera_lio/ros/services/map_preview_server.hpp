// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__SERVICES__MAP_PREVIEW_SERVER_HPP_
#define ITERA_LIO__ROS__SERVICES__MAP_PREVIEW_SERVER_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include <string>

namespace itera_lio::ros {
class LioOutputPublisher;

// ROS request/response adapter for the bounded map preview.
class MapPreviewServer {
public:
  MapPreviewServer(rclcpp::Node &node, LioOutputPublisher &output, const std::string &world_frame);

private:
  rclcpp::Node &node_;
  LioOutputPublisher &output_;
  std::string world_frame_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tile_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tile_request_subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr colored_count_publisher_;
  rclcpp::TimerBase::SharedPtr colored_count_timer_;
};
} // namespace itera_lio::ros
#endif // ITERA_LIO__ROS__SERVICES__MAP_PREVIEW_SERVER_HPP_
