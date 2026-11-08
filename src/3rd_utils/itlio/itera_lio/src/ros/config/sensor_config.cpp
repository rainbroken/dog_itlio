// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/config/module_loaders.hpp"
#include "itera_lio/ros/parameter_reader.hpp"

#include <cmath>
#include <stdexcept>

namespace itera_lio::ros::config {

SensorConfig loadSensorConfig(rclcpp::Node &node) {
  SensorConfig options;
  ParameterReader reader(node);
  options.lidar_topic = node.declare_parameter<std::string>("topics.lidar", "/interfaces/lidar");
  options.imu_topic = node.declare_parameter<std::string>("topics.imu", "/interfaces/imu");
  options.lidar_queue_capacity = reader.positiveSize("sensor.lidar_queue_capacity", 8);
  options.imu_queue_capacity = reader.positiveSize("sensor.imu_queue_capacity", 2000);
  options.imu_lookahead_sec = node.declare_parameter<double>("sync.imu_lookahead_sec", 0.0);
  options.lidar_to_imu_offset_sec = node.declare_parameter<double>("time_offset.lidar_to_imu", 0.0);
  if (!std::isfinite(options.lidar_to_imu_offset_sec)) {
    throw std::invalid_argument("time_offset.lidar_to_imu must be finite");
  }
  options.qos_reliability =
      node.declare_parameter<std::string>("sensor.qos.reliability", "best_effort");
  options.lidar_qos_depth = reader.positiveSize("sensor.qos.lidar_depth", 32);
  options.imu_qos_depth = reader.positiveSize("sensor.qos.imu_depth", 2000);
  return options;
}

} // namespace itera_lio::ros::config
