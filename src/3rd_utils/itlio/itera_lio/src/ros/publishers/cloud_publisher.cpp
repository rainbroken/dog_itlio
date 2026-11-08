// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/output/preview_cloud.hpp"
#include "itera_lio/ros/message_conversions.hpp"
#include "itera_lio/ros/publishers/cloud_publisher.hpp"
#include <chrono>

namespace itera_lio::ros {
CloudPublisher::CloudPublisher(rclcpp::Node &node, const OutputPublisherOptions &options,
                               const common::FixedExtrinsics &extrinsics)
    : options_(options), extrinsics_(extrinsics) {
  const auto preview_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  registered_publisher_ =
      node.create_publisher<sensor_msgs::msg::PointCloud2>(options_.registered_topic, preview_qos);
  deskewed_publisher_ =
      node.create_publisher<sensor_msgs::msg::PointCloud2>(options_.deskewed_topic, preview_qos);
  local_map_publisher_ =
      node.create_publisher<sensor_msgs::msg::PointCloud2>(options_.local_map_topic, preview_qos);
  map_updates_publisher_ =
      node.create_publisher<sensor_msgs::msg::PointCloud2>(options_.map_updates_topic, preview_qos);
  effective_publisher_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
      options_.effective_points_topic, preview_qos);
}

void CloudPublisher::publishDeskewedCloud(const sensor::LidarFrame &scan,
                                          common::FrameOutputTiming &timing) {
  if (deskewed_publisher_->get_subscription_count() == 0U) {
    return;
  }
  const auto preparation_start = std::chrono::steady_clock::now();
  const auto sampled = options_.full_deskewed_enabled
                           ? scan
                           : output::preview::sampleLidarFrame(scan, options_.registered_voxel_size,
                                                               options_.max_preview_points);
  timing.preparation_time_ms += std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - preparation_start)
                                    .count();
  const auto serialization_start = std::chrono::steady_clock::now();
  const auto message = toLidarPointCloud2(sampled, options_.lidar_frame);
  timing.serialization_time_ms += std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - serialization_start)
                                      .count();
  const auto publish_start = std::chrono::steady_clock::now();
  deskewed_publisher_->publish(message);
  timing.publish_call_time_ms +=
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - publish_start)
          .count();
}

void CloudPublisher::publishEffectiveCloud(const common::ObservationBatch &observations,
                                           const double timestamp,
                                           common::FrameOutputTiming &timing) {
  if (effective_publisher_->get_subscription_count() == 0U) {
    return;
  }
  const auto preparation_start = std::chrono::steady_clock::now();
  std::vector<common::MapPoint> points;
  points.reserve(observations.size());
  for (const auto &observation : observations) {
    points.push_back({observation.point_W.cast<float>(), observation.intensity});
  }
  const auto sampled = output::preview::sampleMapPoints(points, options_.registered_voxel_size,
                                                        options_.max_preview_points);
  timing.preparation_time_ms += std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - preparation_start)
                                    .count();
  publishMapPoints(sampled, effective_publisher_, timestamp, timing);
}

void CloudPublisher::publishRegisteredCloud(const sensor::LidarFrame &scan,
                                            const estimator::NominalState &state,
                                            common::FrameOutputTiming &timing) {
  if (registered_publisher_->get_subscription_count() == 0U) {
    return;
  }
  const auto preparation_start = std::chrono::steady_clock::now();
  const auto sampled_scan =
      options_.full_deskewed_enabled
          ? scan
          : output::preview::sampleLidarFrame(scan, options_.registered_voxel_size,
                                              options_.max_preview_points);
  std::vector<common::MapPoint> points;
  points.reserve(sampled_scan.points.size());
  const auto &extrinsics = extrinsics_;
  for (const auto &point : sampled_scan.points) {
    const Eigen::Vector3d point_I = extrinsics.R_IL * point.position + extrinsics.p_IL;
    points.push_back({(state.R_WI * point_I + state.p_WI).cast<float>(), point.intensity});
  }
  timing.preparation_time_ms += std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - preparation_start)
                                    .count();
  publishMapPoints(points, registered_publisher_, scan.scan_end_time, timing);
}

template <typename MapType>
void CloudPublisher::publishMap(
    const MapType &map,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
    const double timestamp, common::FrameOutputTiming &timing) {
  const auto preparation_start = std::chrono::steady_clock::now();
  std::vector<common::MapPoint> points;
  if (!map.getAllPoints(points)) {
    return;
  }
  timing.preparation_time_ms += std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - preparation_start)
                                    .count();
  publishMapPoints(points, publisher, timestamp, timing);
}

void CloudPublisher::publishLocalMap(const frontend::IkdMap &map, const double timestamp,
                                     common::FrameOutputTiming &timing) {
  if (local_map_publisher_->get_subscription_count() == 0U) {
    return;
  }
  publishMap(map, local_map_publisher_, timestamp, timing);
}

void CloudPublisher::publishMapPoints(
    const std::vector<common::MapPoint> &points,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher,
    const double timestamp, common::FrameOutputTiming &timing) {
  const auto serialization_start = std::chrono::steady_clock::now();
  const auto message = toMapPointCloud2(points, timestamp, options_.world_frame);
  timing.serialization_time_ms += std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - serialization_start)
                                      .count();
  const auto publish_start = std::chrono::steady_clock::now();
  publisher->publish(message);
  timing.publish_call_time_ms +=
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - publish_start)
          .count();
}

void CloudPublisher::publishMapUpdates(const double timestamp,
                                       const std::vector<common::MapPoint> &updates,
                                       common::FrameOutputTiming &timing) {
  if (map_updates_publisher_->get_subscription_count() == 0U || updates.empty()) {
    return;
  }
  const auto sampled_updates = output::preview::sampleMapPoints(updates, options_.global_voxel_size,
                                                                options_.max_preview_points);
  publishMapPoints(sampled_updates, map_updates_publisher_, timestamp, timing);
}

} // namespace itera_lio::ros
