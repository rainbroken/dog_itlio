// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/output/graph_export.hpp"
#include "itera_lio/output/map_pcd.hpp"
#include "itera_lio/ros/map_export_controller.hpp"
#include <cstdlib>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <spawn.h>
#include <unistd.h>
#include <signal.h>

extern char **environ;

namespace itera_lio::ros {
bool queueOfflineRefinement(const std::string &directory, const rclcpp::Logger &logger) {
  try {
    if (!std::filesystem::exists(std::filesystem::path(directory) / "colored_points.bin")) {
      RCLCPP_ERROR(logger, "Offline plane refinement requires a LIVO colored archive");
      return false;
    }
    const auto executable = ament_index_cpp::get_package_prefix("itera_lio") +
                            "/lib/itera_lio/refine_saved_run.py";
    std::vector<std::string> command{executable, directory, "--wait-for-pid",
                                    std::to_string(::getpid()), "--publish-latest"};
    std::vector<char *> argv;
    for (auto &part : command) {argv.push_back(part.data());}
    argv.push_back(nullptr);
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    const auto log = (std::filesystem::path(directory) / "refinement.log").string();
    posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, log.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
    posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);
    posix_spawnattr_t attributes;
    posix_spawnattr_init(&attributes);
    sigset_t empty, defaults;
    sigemptyset(&empty);
    sigemptyset(&defaults);
    sigaddset(&defaults, SIGINT);
    sigaddset(&defaults, SIGTERM);
    posix_spawnattr_setsigmask(&attributes, &empty);
    posix_spawnattr_setsigdefault(&attributes, &defaults);
    posix_spawnattr_setflags(&attributes,
                            POSIX_SPAWN_SETSID | POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF);
    pid_t worker = -1;
    const int error = posix_spawn(&worker, executable.c_str(), &actions, &attributes, argv.data(), environ);
    posix_spawnattr_destroy(&attributes);
    posix_spawn_file_actions_destroy(&actions);
    if (error) {
      RCLCPP_ERROR(logger, "Cannot launch offline refinement: error=%d", error);
      return false;
    }
    std::ofstream status(std::filesystem::path(directory) / "refinement_job.json");
    status << "{\"state\":\"waiting_for_exit\",\"worker_pid\":" << worker << "}\n";
    RCLCPP_INFO(logger, "Offline refinement queued: worker=%d, directory='%s'", worker, directory.c_str());
    return bool(status);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(logger, "Cannot queue offline refinement: %s", error.what());
    return false;
  }
}

bool MapExportController::saveCompactMap(bool finalized, std::string &message) {
  try {
    const auto [raw, optimized] = processor_.backend().previewKeyframes();
    if (raw.empty()) {message = "No localization keyframes available"; return false;}
    std::filesystem::remove(std::filesystem::path(options_.pcd_directory)/"localization_keyframes.csv");
    const auto result = finalized ? archive_.finalize() : archive_.snapshot(raw, optimized, false);
    if (!result.success) {message = result.error; return false;}
    const auto poses = output::writeTrajectoryCsv(raw, options_.pcd_directory, "localization_keyframes");
    if (!poses.success) {message = poses.error; return false;}
    output::compactStaticMapFiles(options_.pcd_directory, finalized);
    message = "Saved filtered and localization PCDs to " + options_.pcd_directory;
    RCLCPP_INFO(node_.get_logger(), "%s (%zu filtered points)", message.c_str(), result.points_written);
    return true;
  } catch (const std::exception &error) {message = error.what(); return false;}
}

bool MapExportController::saveSnapshot(std::string &message) {
  if (!options_.livo_enabled && options_.static_filter.enabled &&
      !options_.static_filter.keep_debug_artifacts) {
    return saveCompactMap(false, message);
  }
  // Called while LioNode holds its processing lock. ScanArchive serializes
  // snapshots with colored appends. Save into the same run directory.
  bool saved = false;
  std::string errors;
  // Never leave poses from an older cloud.pcd after a failed replacement.
  std::filesystem::remove(std::filesystem::path(options_.pcd_directory) / "localization_keyframes.csv");
  std::filesystem::remove(std::filesystem::path(options_.pcd_directory) / "localization_optimized_keyframes.csv");
  auto record = [&](const char *label, const output::PcdWriteResult &result) {
    if (result.success) {
      saved = true;
      RCLCPP_INFO(node_.get_logger(), "Saved %s: '%s' (%zu points)", label,
                  result.output_path.c_str(), result.points_written);
    } else {
      errors += std::string(label) + ": " + result.error + "; ";
    }
  };
  // A committed loop edge is only installed after successful optimization.
  const bool has_optimized_map = processor_.backend().graph().hasLoopConstraint();
  const auto [raw, optimized] = processor_.backend().previewKeyframes();
  if (options_.livo_enabled) {
    if (archive_.hasColoredScans()) {
      if (has_optimized_map) {
        record("optimized dense colored map", archive_.optimizedSnapshot(raw, optimized));
      } else {
        record("dense colored map", archive_.snapshot(raw, optimized, false));
      }
    } else {
      errors += "colored scan writer unavailable; ";
    }
  } else if (archive_.hasFullScans()) {
    if (has_optimized_map) {
      record("optimized full deskewed map", archive_.optimizedSnapshot(raw, optimized));
    } else {
      record("full deskewed map", archive_.snapshot(raw, optimized, false));
    }
  } else {
    errors += "full scan writer unavailable; ";
  }
  if(saved && !options_.livo_enabled && !raw.empty()) {
    const auto poses=output::writeTrajectoryCsv(has_optimized_map ? optimized : raw,
        options_.pcd_directory,has_optimized_map ? "localization_optimized_keyframes" : "localization_keyframes");
    if(!poses.success)errors+="localization keyframes: "+poses.error+"; ";
  }
  message = saved ? (errors.empty() ? std::string(has_optimized_map ? "Optimized PCD saved to "
                                                                    : "Original PCD saved to ") +
                                          options_.pcd_directory
                                    : "Partial save: " + errors)
                  : "Save failed: " + errors;
  return saved && errors.empty();
}

bool MapExportController::savePcd(std::size_t colored_preview_points) {
  if (!options_.save_pcd_on_shutdown) {
    if (const char* directory = std::getenv("ITERA_LIO_EVAL_GRAPH_DIR")) {
      // Evaluation-only CSV export, without constructing or writing map PCDs.
      output::writePoseGraphConstraintsCsv(processor_.backend().graph(), directory);
      const auto [raw, final] = processor_.backend().previewKeyframes();
      if (!raw.empty()) {
        output::writeTrajectoryCsv(raw, directory, "raw_keyframes");
        output::writeTrajectoryCsv(final, directory, "final_keyframes");
      }
    }
    return false;
  }
  if (!options_.livo_enabled && options_.static_filter.enabled &&
      !options_.static_filter.keep_debug_artifacts) {
    std::string message;
    const bool saved = saveCompactMap(true, message);
    if (!saved) {RCLCPP_ERROR(node_.get_logger(), "Compact map save failed: %s", message.c_str());}
    return saved;
  }
  const auto graph_audit =
      output::writePoseGraphConstraintsCsv(processor_.backend().graph(), options_.pcd_directory);
  std::filesystem::remove(std::filesystem::path(options_.pcd_directory) / "localization_keyframes.csv");
  std::filesystem::remove(std::filesystem::path(options_.pcd_directory) / "localization_optimized_keyframes.csv");
  if (!graph_audit.success) {
    RCLCPP_ERROR(node_.get_logger(), "Failed to save graph constraints: %s",
                 graph_audit.error.c_str());
  }
  // Evaluation export only: preserve both endpoints of the graph correction,
  // including the zero-loop case. This does not run or modify optimization.
  const auto [raw_keyframes, final_keyframes] = processor_.backend().previewKeyframes();
  bool complete = graph_audit.success && !raw_keyframes.empty();
  if (!raw_keyframes.empty()) {
    complete &= output::writeTrajectoryCsv(raw_keyframes, options_.pcd_directory, "raw_keyframes").success;
    complete &= output::writeTrajectoryCsv(final_keyframes, options_.pcd_directory, "final_keyframes").success;
  }
  std::vector<common::MapPoint> optimized_points;
  const bool has_optimized_map =
      processor_.getOptimizedMapPoints(optimized_points, options_.optimized_map_voxel_size);
  if (has_optimized_map) {
    const auto optimized_result =
        output::writeMapPcd(optimized_points, options_.pcd_directory, "optimized_map");
    complete &= optimized_result.success;
    if (!optimized_result.success) {
      RCLCPP_ERROR(node_.get_logger(), "Failed to save optimized map PCD: %s",
                   optimized_result.error.c_str());
    } else {
      RCLCPP_INFO(node_.get_logger(), "Saved optimized map PCD: '%s' (%zu points)",
                  optimized_result.output_path.c_str(), optimized_result.points_written);
    }
    const auto trajectory_result = output::writeTrajectoryCsv(
        processor_.backend().optimizedKeyframes(), options_.pcd_directory, "optimized_trajectory");
    complete &= trajectory_result.success;
    if (!trajectory_result.success) {
      RCLCPP_ERROR(node_.get_logger(), "Failed to save optimized trajectory: %s",
                   trajectory_result.error.c_str());
    } else {
      RCLCPP_INFO(node_.get_logger(), "Saved optimized trajectory: '%s' (%zu poses)",
                  trajectory_result.output_path.c_str(), trajectory_result.points_written);
    }
  }
  if (options_.livo_enabled) {
    if (archive_.hasColoredScans()) {
      if (has_optimized_map) {
        const auto graph = processor_.backend().graph();
        const auto optimized_result =
            archive_.optimizedSnapshot(graph.nodes(), processor_.backend().optimizedKeyframes());
        if (!optimized_result.success) {
          complete = false;
          RCLCPP_ERROR(node_.get_logger(), "Failed to save optimized colored PCD: %s",
                       optimized_result.error.c_str());
        } else {
          RCLCPP_INFO(node_.get_logger(), "Saved optimized colored PCD: '%s' (%zu points)",
                      optimized_result.output_path.c_str(), optimized_result.points_written);
        }
      }
      const auto result = archive_.finalize();
      if (!result.success) {
        RCLCPP_ERROR(node_.get_logger(), "Failed to save dense colored global PCD: %s",
                     result.error.c_str());
      } else {
        RCLCPP_INFO(node_.get_logger(),
                    "Saved dense colored global PCD: '%s' (%zu points); "
                    "voxel_preview_points=%zu",
                    result.output_path.c_str(), result.points_written, colored_preview_points);
      }
      return complete && result.success;
    }
    return false;
  }

  const bool save_full_deskewed =
      archive_.hasFullScans() &&
      (options_.pcd_source == "auto" || options_.pcd_source == "full_deskewed");
  if (save_full_deskewed) {
    const auto graph = processor_.backend().graph();
    if (graph.hasLoopConstraint()) {
      const auto optimized =
          archive_.optimizedSnapshot(graph.nodes(), processor_.backend().optimizedKeyframes());
      if (!optimized.success) {
        complete = false;
        RCLCPP_ERROR(node_.get_logger(), "Failed to save optimized full deskewed PCD: %s",
                     optimized.error.c_str());
      } else {
        complete &= output::writeTrajectoryCsv(final_keyframes,options_.pcd_directory,
                                               "localization_optimized_keyframes").success;
      }
    }
    const auto result = archive_.finalize();
    if (!result.success) {
      RCLCPP_ERROR(node_.get_logger(), "Failed to save full deskewed PCD: %s",
                   result.error.c_str());
    } else {
      RCLCPP_INFO(node_.get_logger(), "Saved full deskewed PCD: '%s' (%zu points)",
                  result.output_path.c_str(), result.points_written);
    }
    if(result.success) {
      const auto poses=output::writeTrajectoryCsv(raw_keyframes,
          options_.pcd_directory,"localization_keyframes");
      complete &= poses.success;
    }
    return complete && result.success;
  } else {
    RCLCPP_ERROR(node_.get_logger(), "Full scan writer unavailable; no PCD saved");
  }
  return false;
}

} // namespace itera_lio::ros
