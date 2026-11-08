// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/services/reconstruction_services.hpp"
#include "itera_lio/output/graph_export.hpp"
#include "itera_lio/system/lio_processor.hpp"
#include "itera_lio/ros/diagnostics_reporter.hpp"
#include "itera_lio/ros/evaluation_reporter.hpp"
#include "itera_lio/ros/publishers/output_publisher.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>

namespace itera_lio::ros {
ReconstructionServices::ReconstructionServices(rclcpp::Node &node, system::LioProcessor &processor,
                                               LioOutputPublisher &output,
                                               EvaluationReporter &evaluation,
                                               DiagnosticsReporter &diagnostics,
                                               std::mutex &processing_mutex,
                                               const std::string &pcd_directory)
    : node_(node), processor_(processor), output_(output), evaluation_(evaluation),
      diagnostics_(diagnostics), processing_mutex_(processing_mutex),
      pcd_directory_(pcd_directory) {
  save_service_ = node_.create_service<std_srvs::srv::Trigger>(
      "/lio/save_pcd", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(processing_mutex_);
        response->success = output_.saveSnapshot(response->message);
      });
  refine_service_ = node_.create_service<std_srvs::srv::Trigger>(
      "/lio/refine_revisits", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
        if (last_lidar_arrival_ns_.load() == 0 ||
            now_ns - last_lidar_arrival_ns_.load() <
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::seconds(3))
                    .count()) {
          response->message = "Stop acquisition/playback and wait at least 3 seconds before "
                              "offline revisit refinement";
          return;
        }
        std::lock_guard<std::mutex> lock(processing_mutex_);
        const auto keyframes = processor_.backend().optimizedKeyframes();
        if (!processor_.backend().graph().hasLoopConstraint() || keyframes.empty()) {
          response->message = "A verified global loop is required before local revisit refinement";
          return;
        }
        const auto audit_dir =
            std::filesystem::path(pcd_directory_) / ("revisit_" + std::to_string(now_ns));
        std::filesystem::create_directories(audit_dir);
        const auto before_graph =
            output::writePoseGraphConstraintsCsv(processor_.backend().graph(), audit_dir.string());
        const auto before_path =
            output::writeTrajectoryCsv(keyframes, audit_dir.string(), "before_refinement");
        if (!before_graph.success || !before_path.success) {
          response->message = "Cannot save refinement audit; graph unchanged";
          return;
        }
        RCLCPP_INFO(node_.get_logger(),
                    "Starting explicit local revisit refinement (%zu keyframes)", keyframes.size());
        const auto report =
            processor_.refineOptimizedLoops((audit_dir / "keyframe_clouds.bin").string());
        if (!report.error.empty()) {
          response->message = report.error;
          return;
        }
        evaluation_.recordRevisitRefinement(report.verifications, report.accepted,
                                            report.committed);
        diagnostics_.updateBackendMetrics(processor_.backend());
        std::ofstream audit(audit_dir / "verifications.csv");
        audit << "current_id,matched_id,accepted,reason,fitness,inliers,reciprocal_overlap,"
                 "condition,translation_correction,rotation_correction\n";
        audit << std::setprecision(17);
        for (const auto &result : report.results) {
          audit << result.current_id << ',' << result.matched_id << ',' << result.accepted << ','
                << result.rejection_reason << ',' << result.fitness << ',' << result.inliers << ','
                << result.reciprocal_inlier_ratio << ',' << result.geometry_condition_number << ','
                << result.translation_correction << ',' << result.rotation_correction << '\n';
        }
        audit.close();
        const auto after_dir = audit_dir / "after";
        std::filesystem::create_directories(after_dir);
        output::writePoseGraphConstraintsCsv(processor_.backend().graph(), after_dir.string());
        output::writeTrajectoryCsv(processor_.backend().optimizedKeyframes(), after_dir.string(),
                                   "after_refinement");
        output_.publishRefinedMap(keyframes.back().timestamp);
        std::string save_message;
        const bool saved = report.committed > 0U && output_.saveSnapshot(save_message);
        response->success = saved;
        response->message = "Revisit verifications=" + std::to_string(report.verifications) +
                            ", accepted=" + std::to_string(report.accepted) +
                            ", committed=" + std::to_string(report.committed) +
                            "; audit=" + audit_dir.string() + "; " + save_message;
        RCLCPP_INFO(node_.get_logger(), "%s", response->message.c_str());
      });
}

void ReconstructionServices::noteLidarArrival() {
  last_lidar_arrival_ns_.store(std::chrono::steady_clock::now().time_since_epoch().count());
}
} // namespace itera_lio::ros
