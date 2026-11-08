// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__MAP_EXPORT_CONTROLLER_HPP_
#define ITERA_LIO__ROS__MAP_EXPORT_CONTROLLER_HPP_

#include "itera_lio/output/scan_archive.hpp"
#include "itera_lio/ros/publishers/output_options.hpp"
#include "itera_lio/system/lio_processor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace itera_lio::ros {

// Detached, low-priority offline worker waits for this process to exit before
// reading the archives. It does not update the estimator or overwrite its PCD.
bool queueOfflineRefinement(const std::string &directory, const rclcpp::Logger &logger);

class MapExportController {
public:
  MapExportController(rclcpp::Node &node, const OutputPublisherOptions &options,
                      const system::LioProcessor &processor, output::ScanArchive &archive)
      : node_(node), options_(options), processor_(processor), archive_(archive) {}
  bool saveSnapshot(std::string &message);
  // The caller must stop processing and drain all scan workers before shutdown export.
  bool savePcd(std::size_t colored_preview_points);

private:
  bool saveCompactMap(bool finalized, std::string &message);
  rclcpp::Node &node_;
  OutputPublisherOptions options_;
  const system::LioProcessor &processor_;
  output::ScanArchive &archive_;
};

} // namespace itera_lio::ros

#endif // ITERA_LIO__ROS__MAP_EXPORT_CONTROLLER_HPP_
