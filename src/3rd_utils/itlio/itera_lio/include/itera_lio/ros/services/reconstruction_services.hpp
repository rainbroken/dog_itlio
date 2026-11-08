// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__SERVICES__RECONSTRUCTION_SERVICES_HPP_
#define ITERA_LIO__ROS__SERVICES__RECONSTRUCTION_SERVICES_HPP_
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

namespace itera_lio::system {
class LioProcessor;
}
namespace itera_lio::ros {
class LioOutputPublisher;
class EvaluationReporter;
class DiagnosticsReporter;

// Services use the same processing mutex as online estimation. Referenced
// modules and mutex must outlive this adapter and its callbacks.
class ReconstructionServices {
public:
  ReconstructionServices(rclcpp::Node &node, system::LioProcessor &processor,
                         LioOutputPublisher &output, EvaluationReporter &evaluation,
                         DiagnosticsReporter &diagnostics, std::mutex &processing_mutex,
                         const std::string &pcd_directory);
  void noteLidarArrival();

private:
  rclcpp::Node &node_;
  system::LioProcessor &processor_;
  LioOutputPublisher &output_;
  EvaluationReporter &evaluation_;
  DiagnosticsReporter &diagnostics_;
  std::mutex &processing_mutex_;
  std::string pcd_directory_;
  std::atomic<std::chrono::steady_clock::duration::rep> last_lidar_arrival_ns_{0};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr refine_service_;
};
} // namespace itera_lio::ros
#endif // ITERA_LIO__ROS__SERVICES__RECONSTRUCTION_SERVICES_HPP_
