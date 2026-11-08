// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__CONFIG__NODE_CONFIG_HPP_
#define ITERA_LIO__ROS__CONFIG__NODE_CONFIG_HPP_

#include <cstddef>
#include <string>

#include "itera_lio/output/evaluation_logger.hpp"
#include "itera_lio/ros/publishers/output_options.hpp"
#include "itera_lio/system/lio_processor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace itera_lio::ros::config {

struct SensorConfig {
  std::string lidar_topic;
  std::string imu_topic;
  std::string qos_reliability;
  std::size_t lidar_qos_depth;
  std::size_t imu_qos_depth;
  std::size_t lidar_queue_capacity;
  std::size_t imu_queue_capacity;
  double imu_lookahead_sec;
  double lidar_to_imu_offset_sec;
};

struct EvaluationConfig {
  bool enabled;
  std::string output_directory;
  output::EvaluationOutputOptions outputs;
};

struct NodeConfig {
  SensorConfig sensor;
  system::LioProcessorOptions processor;
  OutputPublisherOptions output;
  EvaluationConfig evaluation;
  double diagnostic_period_sec;
};

// Parameter loading only: no subscriptions, worker threads or file creation.
NodeConfig loadNodeConfig(rclcpp::Node &node);

} // namespace itera_lio::ros::config

#endif // ITERA_LIO__ROS__CONFIG__NODE_CONFIG_HPP_
