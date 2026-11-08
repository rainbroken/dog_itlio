// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__INPUT_ADAPTER_HPP_
#define ITERA_LIO__ROS__INPUT_ADAPTER_HPP_

#include <atomic>
#include <cstdint>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include "itera_lio/sensor/measurement_synchronizer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace itera_lio::ros
{

class RosInputAdapter
{
public:
  RosInputAdapter(
    rclcpp::Node & node,
    const std::string & lidar_topic,
    const std::string & imu_topic,
    double imu_time_offset,
    const std::string & qos_reliability,
    std::size_t lidar_qos_depth,
    std::size_t imu_qos_depth,
    sensor::MeasurementSynchronizer & synchronizer);

  std::uint64_t lidarParseFailures() const {return lidar_parse_failures_.load();}
  void setLidarReceivedCallback(std::function<void(double)> callback);

private:
  void onLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr message);

  rclcpp::Node & node_;
  double imu_time_offset_{0.0};
  sensor::MeasurementSynchronizer & synchronizer_;
  std::atomic<std::uint64_t> lidar_parse_failures_{0U};
  std::function<void(double)> lidar_received_callback_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

}  // namespace itera_lio::ros

#endif  // ITERA_LIO__ROS__INPUT_ADAPTER_HPP_
