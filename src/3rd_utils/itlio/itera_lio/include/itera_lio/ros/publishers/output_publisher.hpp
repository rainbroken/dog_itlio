// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__PUBLISHERS__OUTPUT_PUBLISHER_HPP_
#define ITERA_LIO__ROS__PUBLISHERS__OUTPUT_PUBLISHER_HPP_

#include "itera_lio/common/frame_metrics.hpp"
#include "itera_lio/ros/publishers/output_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
namespace itera_lio::system {
class LioProcessor;
struct ProcessedFrame;
} // namespace itera_lio::system
namespace itera_lio::output {
class ScanArchive;
}

namespace itera_lio::ros {

class StatePublisher;
class CloudPublisher;
class MapPreviewPublisher;
class ColoredCloudPublisher;
class OptimizedMapPublisher;
class MapExportController;

class LioOutputPublisher {
public:
  LioOutputPublisher(rclcpp::Node &node, const OutputPublisherOptions &options,
                     system::LioProcessor &processor);
  ~LioOutputPublisher();
  common::FrameOutputTiming publishFrame(const system::ProcessedFrame &frame);
  void noteLidarArrival(double scan_end_time);
  std::size_t globalMapSize() const;
  bool savePcd();
  bool saveSnapshot(std::string &message);
  void publishRefinedMap(double timestamp);
  bool getColoredTile(std::int64_t x, std::int64_t y, std::int64_t z, std::size_t max_points,
                      std::vector<mapping::color::ColoredPoint> &points);

private:
  void stopWorkers();
  rclcpp::Node &node_;
  OutputPublisherOptions options_;
  system::LioProcessor &processor_;
  std::chrono::steady_clock::time_point last_preview_request_{};
  // Reverse destruction stops consumers before their archive/cache dependencies.
  std::unique_ptr<output::ScanArchive> archive_;
  std::unique_ptr<StatePublisher> state_;
  std::unique_ptr<CloudPublisher> clouds_;
  std::unique_ptr<MapPreviewPublisher> preview_;
  std::unique_ptr<MapExportController> exports_;
  std::unique_ptr<OptimizedMapPublisher> optimized_;
  std::unique_ptr<ColoredCloudPublisher> colored_;
};

} // namespace itera_lio::ros

#endif // ITERA_LIO__ROS__PUBLISHERS__OUTPUT_PUBLISHER_HPP_
