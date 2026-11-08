// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/services/map_preview_server.hpp"
#include "itera_lio/ros/message_conversions.hpp"
#include "itera_lio/ros/publishers/output_publisher.hpp"

#include <chrono>
#include <cstdint>
#include <sstream>
#include <vector>

namespace itera_lio::ros {
MapPreviewServer::MapPreviewServer(rclcpp::Node &node, LioOutputPublisher &output,
                                   const std::string &world_frame)
    : node_(node), output_(output), world_frame_(world_frame) {
  tile_publisher_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>("/lio/colored_map_tile",
                                                                          rclcpp::SensorDataQoS());
  tile_request_subscription_ = node_.create_subscription<std_msgs::msg::String>(
      "/lio/colored_map_tile_request", 10, [this](const std_msgs::msg::String::SharedPtr request) {
        std::istringstream input(request->data);
        std::uint64_t id = 0;
        std::int64_t x = 0, y = 0, z = 0;
        std::size_t limit = 0;
        if (!(input >> id >> x >> y >> z >> limit) || limit == 0 || limit > 100000) {
          return;
        }
        std::vector<mapping::color::ColoredPoint> points;
        if (!output_.getColoredTile(x, y, z, limit, points)) {
          return;
        }
        auto response = ros::toColoredPointCloud2(points, node_.now().seconds(), world_frame_);
        response.header.frame_id = "tile:" + std::to_string(id);
        tile_publisher_->publish(response);
      });
  colored_count_publisher_ =
      node_.create_publisher<std_msgs::msg::UInt64>("/lio/colored_map_point_count", 10);
  colored_count_timer_ = node_.create_wall_timer(std::chrono::seconds(1), [this]() {
    std_msgs::msg::UInt64 count;
    count.data = output_.globalMapSize();
    colored_count_publisher_->publish(count);
  });
}
} // namespace itera_lio::ros
