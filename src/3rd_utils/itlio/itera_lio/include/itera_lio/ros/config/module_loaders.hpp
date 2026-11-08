// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__CONFIG__MODULE_LOADERS_HPP_
#define ITERA_LIO__ROS__CONFIG__MODULE_LOADERS_HPP_

#include "itera_lio/ros/config/node_config.hpp"

namespace itera_lio::ros::config {
SensorConfig loadSensorConfig(rclcpp::Node &node);
system::LioProcessorOptions loadFrontendConfig(rclcpp::Node &node);
backend::BackendOptions loadBackendConfig(rclcpp::Node &node);
loop::LoopClosureOptions loadLoopConfig(rclcpp::Node &node);
OutputPublisherOptions loadOutputConfig(rclcpp::Node &node);
void loadCameraConfig(rclcpp::Node &node, OutputPublisherOptions &options);
EvaluationConfig loadEvaluationConfig(rclcpp::Node &node);
} // namespace itera_lio::ros::config

#endif // ITERA_LIO__ROS__CONFIG__MODULE_LOADERS_HPP_
