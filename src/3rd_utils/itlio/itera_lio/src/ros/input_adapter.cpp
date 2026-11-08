// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/input_adapter.hpp"

#include <functional>
#include <chrono>
#include <stdexcept>
#include <utility>

#include "itera_lio/sensor/lidar_parser.hpp"

namespace itera_lio::ros
{

namespace
{

rclcpp::QoS makeSensorQos(
  const std::size_t depth, const std::string & reliability)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(depth)).durability_volatile();
  if (reliability == "reliable") {
    return qos.reliable();
  }
  if (reliability == "best_effort") {
    return qos.best_effort();
  }
  throw std::invalid_argument(
          "sensor.qos.reliability must be reliable or best_effort");
}

}  // namespace

RosInputAdapter::RosInputAdapter(
  rclcpp::Node & node,
  const std::string & lidar_topic,
  const std::string & imu_topic,
  const double imu_time_offset,
  const std::string & qos_reliability,
  const std::size_t lidar_qos_depth,
  const std::size_t imu_qos_depth,
  sensor::MeasurementSynchronizer & synchronizer)
: node_(node), imu_time_offset_(imu_time_offset), synchronizer_(synchronizer)
{
  const auto lidar_qos = makeSensorQos(lidar_qos_depth, qos_reliability);
  const auto imu_qos = makeSensorQos(imu_qos_depth, qos_reliability);
  lidar_subscription_ = node_.create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic, lidar_qos,
    std::bind(&RosInputAdapter::onLidar, this, std::placeholders::_1));
  imu_subscription_ = node_.create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, imu_qos,
    std::bind(&RosInputAdapter::onImu, this, std::placeholders::_1));
  RCLCPP_INFO(
    node_.get_logger(), "Input QoS: %s (depth L/I=%zu/%zu)",
    qos_reliability.c_str(), lidar_qos_depth, imu_qos_depth);
}

void RosInputAdapter::setLidarReceivedCallback(
  std::function<void(double)> callback)
{
  lidar_received_callback_ = std::move(callback);
}

void RosInputAdapter::onLidar(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  const auto callback_start = std::chrono::steady_clock::now();
  sensor::LidarFrame lidar_frame;
  const auto report = sensor::parsePointCloud2(*message, lidar_frame);
  if (report.status != sensor::LidarParseStatus::kSuccess) {
    ++lidar_parse_failures_;
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "Rejected LiDAR frame during parsing: input=%zu output=%zu rejected=%zu",
      report.input_points, report.output_points, report.rejected_points);
    return;
  }
  if (lidar_received_callback_) {
    lidar_received_callback_(lidar_frame.scan_end_time);
  }
  lidar_frame.callback_start_steady_sec =
    std::chrono::duration<double>(callback_start.time_since_epoch()).count();
  lidar_frame.input_callback_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - callback_start).count();
  if (synchronizer_.pushLidar(std::move(lidar_frame)) ==
    sensor::PushResult::kRejectedTimestampRegression)
  {
    RCLCPP_ERROR_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "Rejected LiDAR message because its timestamp regressed");
  }
}

void RosInputAdapter::onImu(const sensor_msgs::msg::Imu::ConstSharedPtr message)
{
  sensor::ImuData imu_sample;
  imu_sample.timestamp =
    static_cast<double>(message->header.stamp.sec) +
    static_cast<double>(message->header.stamp.nanosec) * 1e-9;
  imu_sample.gyro = Eigen::Vector3d(
    message->angular_velocity.x,
    message->angular_velocity.y,
    message->angular_velocity.z);
  imu_sample.accel = Eigen::Vector3d(
    message->linear_acceleration.x,
    message->linear_acceleration.y,
    message->linear_acceleration.z);
  imu_sample = sensor::applyTimeOffset(imu_sample, imu_time_offset_);
  if (synchronizer_.pushImu(std::move(imu_sample)) ==
    sensor::PushResult::kRejectedTimestampRegression)
  {
    RCLCPP_ERROR_THROTTLE(
      node_.get_logger(), *node_.get_clock(), 5000,
      "Rejected IMU message because its timestamp regressed");
  }
}

}  // namespace itera_lio::ros
