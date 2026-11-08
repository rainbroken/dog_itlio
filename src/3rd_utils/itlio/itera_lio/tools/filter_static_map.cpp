// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0
#include "itera_lio/output/static_map_filter.hpp"
#include <fstream>
#include <iostream>
#include <sstream>

int main(int argc, char **argv) {
  const bool compact = argc == 3 && std::string(argv[2]) == "--compact";
  if (argc != 2 && !compact) {
    std::cerr << "Usage: ros2 run itera_lio filter_static_map RUN_DIRECTORY [--compact]\n"
                 "Requires evaluation/parameters.yaml; uses the recorded save_pcd.voxel_size.\n";
    return 1;
  }
  try {
    // No alternate default or user override: keep the actual run's PCD resolution.
    std::ifstream parameters(std::string(argv[1])+"/evaluation/parameters.yaml");
    float voxel = -1;
    itera_lio::output::StaticMapFilterOptions options;
    std::string line;
    while (std::getline(parameters, line)) {
      const auto position = line.find("save_pcd.voxel_size:");
      if (position != std::string::npos) {voxel = std::stof(line.substr(position+20));}
      const auto debug = line.find("save_pcd.keep_debug_artifacts:");
      if (debug != std::string::npos) {
        options.keep_debug_artifacts = line.substr(debug+29).find("true") != std::string::npos;
      }
      const auto prefix = line.find("save_pcd.dynamic_filter.");
      if (prefix != std::string::npos) {
        const auto colon = line.find(':', prefix);
        const auto key = line.substr(prefix+24, colon-prefix-24);
        const auto value = line.substr(colon+1);
        if (key == "evidence_voxel_size") {options.evidence_voxel_size = std::stof(value);}
        else if (key == "max_range") {options.max_range = std::stod(value);}
        else if (key == "endpoint_margin") {options.endpoint_margin = std::stod(value);}
        else if (key == "margin_per_meter") {options.margin_per_meter = std::stod(value);}
        else if (key == "max_endpoint_margin") {options.max_endpoint_margin = std::stod(value);}
        else if (key == "observer_voxel_size") {options.observer_voxel_size = std::stod(value);}
        else if (key == "min_free_observations") {options.min_free_observations = std::stoul(value);}
        else if (key == "min_scan_interval") {options.min_scan_interval = std::stod(value);}
      }
    }
    if (voxel < 0) {throw std::runtime_error("recorded save_pcd.voxel_size is required");}
    if (compact) {options.keep_debug_artifacts = false;}
    const auto result = itera_lio::output::exportStaticMap(argv[1], voxel, options);
    if (!result.success) {throw std::runtime_error(result.error);}
    if (compact) {itera_lio::output::compactStaticMapFiles(argv[1], true);}
    std::cout << result.output_path << " (" << result.points_written << " points)\n";
  } catch (const std::exception &error) {std::cerr << error.what() << '\n'; return 1;}
}
