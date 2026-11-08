// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__DIAGNOSTICS_REPORTER_HPP_
#define ITERA_LIO__ROS__DIAGNOSTICS_REPORTER_HPP_

#include <cstdint>
#include <mutex>
#include <utility>

#include "itera_lio/common/evaluation_types.hpp"
#include "itera_lio/common/frame_metrics.hpp"
#include "itera_lio/sensor/measurement_synchronizer.hpp"
#include "itera_lio/system/lio_processor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace itera_lio::ros {

std::pair<double, double> processMemoryMib();

class RosInputAdapter;

class DiagnosticsReporter {
public:
  DiagnosticsReporter(rclcpp::Node &node, double period_sec,
                      const sensor::MeasurementSynchronizer &synchronizer);

  void setInputAdapter(const RosInputAdapter *input_adapter);
  void updateProcessorStats(const system::LioProcessorStats &stats);
  void updateBackendMetrics(const backend::BackendManager &backend);
  void updateFrameMetrics(const system::ProcessedFrame &frame,
                          const common::FrameOutputTiming &output_timing,
                          std::size_t global_map_size);
  common::RuntimeSnapshot snapshot() const;
  void log();

private:
  struct FrameSnapshot {
    common::FrameProcessingTiming processing;
    common::FrameOutputTiming output;
    double sensor_timestamp{0.0};
    double registration_time_ms{0.0};
    double correction_time_ms{0.0};
    double map_delete_time_ms{0.0};
    double archive_update_time_ms{0.0};
    double map_insert_time_ms{0.0};
    std::size_t map_candidate_points{0U};
    std::size_t map_inserted_points{0U};
    std::size_t map_replaced_points{0U};
    std::size_t map_rejected_points{0U};
    std::size_t map_size{0U};
    std::size_t global_map_size{0U};
    std::size_t pending_delete_boxes{0U};
    double rebuild_last_ms{0.0};
    double rebuild_max_ms{0.0};
    bool rebuild_active{false};
    std::uint64_t rebuild_total{0U};
    std::uint64_t rebuild_synchronous{0U};
    std::uint64_t rebuild_asynchronous{0U};
    std::uint8_t rebuild_last_reason{0U};
    std::size_t rebuild_last_points{0U};
    std::size_t rebuild_max_points{0U};
    std::size_t backend_keyframes{0U};
    std::size_t backend_odometry_constraints{0U};
    std::size_t backend_loop_constraints{0U};
    loop::LoopWorkerStats loop_worker;
    backend::BackendWorkerStats backend_worker;
  };

  rclcpp::Node &node_;
  double period_sec_{1.0};
  const sensor::MeasurementSynchronizer &synchronizer_;
  const RosInputAdapter *input_adapter_{nullptr};
  mutable std::mutex mutex_;
  system::LioProcessorStats processor_stats_;
  FrameSnapshot frame_snapshot_;
  sensor::SensorBufferStats previous_stats_;
};

} // namespace itera_lio::ros

#endif // ITERA_LIO__ROS__DIAGNOSTICS_REPORTER_HPP_
