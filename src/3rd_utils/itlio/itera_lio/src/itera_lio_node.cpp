// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <functional>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "itera_lio/ros/config/node_config.hpp"
#include "itera_lio/ros/diagnostics_reporter.hpp"
#include "itera_lio/ros/evaluation_reporter.hpp"
#include "itera_lio/ros/input_adapter.hpp"
#include "itera_lio/ros/map_export_controller.hpp"
#include "itera_lio/ros/publishers/output_publisher.hpp"
#include "itera_lio/ros/services/map_preview_server.hpp"
#include "itera_lio/ros/services/reconstruction_services.hpp"
#include "itera_lio/sensor/measurement_synchronizer.hpp"
#include "itera_lio/system/lio_processor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace itera_lio {

class LioNode final : public rclcpp::Node {
public:
  LioNode()
      : Node("itera_lio_node"), config_(ros::config::loadNodeConfig(*this)),
        sensor_buffer_(config_.sensor.lidar_queue_capacity, config_.sensor.imu_queue_capacity,
                       config_.sensor.imu_lookahead_sec) {
    const auto &processor_options = config_.processor;
    processor_ = std::make_unique<system::LioProcessor>(processor_options);
    input_adapter_ = std::make_unique<ros::RosInputAdapter>(
        *this, config_.sensor.lidar_topic, config_.sensor.imu_topic,
        config_.sensor.lidar_to_imu_offset_sec, config_.sensor.qos_reliability,
        config_.sensor.lidar_qos_depth, config_.sensor.imu_qos_depth, sensor_buffer_);
    diagnostics_reporter_ = std::make_unique<ros::DiagnosticsReporter>(
        *this, config_.diagnostic_period_sec, sensor_buffer_);
    diagnostics_reporter_->setInputAdapter(input_adapter_.get());
    output_publisher_ =
        std::make_unique<ros::LioOutputPublisher>(*this, config_.output, *processor_);
    output::EvaluationMetadata metadata{config_.sensor.lidar_topic, config_.sensor.imu_topic,
                                        config_.output.odom_frame, config_.output.imu_frame,
                                        config_.output.lidar_frame};
    evaluation_reporter_ = std::make_unique<ros::EvaluationReporter>(
        *this, config_.evaluation.enabled, config_.evaluation.output_directory, metadata,
        config_.evaluation.outputs);

    preview_server_ = std::make_unique<ros::MapPreviewServer>(*this, *output_publisher_,
                                                              config_.output.odom_frame);
    reconstruction_services_ = std::make_unique<ros::ReconstructionServices>(
        *this, *processor_, *output_publisher_, *evaluation_reporter_, *diagnostics_reporter_,
        processing_mutex_, config_.output.pcd_directory);
    input_adapter_->setLidarReceivedCallback([this](const double scan_end_time) {
      reconstruction_services_->noteLidarArrival();
      output_publisher_->noteLidarArrival(scan_end_time);
    });

    diagnostic_timer_ =
        create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::duration<double>(config_.diagnostic_period_sec)),
                          std::bind(&ros::DiagnosticsReporter::log, diagnostics_reporter_.get()));
    if (evaluation_reporter_->runtimeTickEnabled()) {
      evaluation_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(1.0 / evaluation_reporter_->runtimeFrequencyHz())),
          [this]() { evaluation_reporter_->writeRuntime(diagnostics_reporter_->snapshot()); });
    }
    if (config_.output.refinement_archive) {
      if (!config_.evaluation.enabled || !config_.evaluation.outputs.odometry ||
          !config_.evaluation.outputs.summary) {
        throw std::invalid_argument("refinement archive requires evaluation odometry and summary");
      }
      std::filesystem::create_directories(config_.output.pcd_directory);
      refinement_imu_.open(std::filesystem::path(config_.output.pcd_directory) /
                               "refinement_imu.bin", std::ios::binary);
      if (!refinement_imu_) {throw std::runtime_error("Cannot open refinement IMU archive");}
      std::ofstream metadata(std::filesystem::path(config_.output.pcd_directory) /
                                 "refinement_archive.json");
      metadata << "{\"pid\":" << ::getpid()
               << ",\"imu_columns\":7,\"imu_time_axis\":\"lidar_aligned\"}\n";
      if (!metadata) {throw std::runtime_error("Cannot write refinement archive metadata");}
    }
    processing_thread_ = std::thread(&LioNode::processingLoop, this);
    RCLCPP_INFO(
        get_logger(),
        "LIO started: lidar='%s', imu='%s', frames(world/odom/imu/lidar)='%s / %s / %s / %s', "
        "time_offset_lidar_to_imu=%.6f s, "
        "p_IL=(%.5f, %.5f, %.5f), "
        "R_IL=[%.5f %.5f %.5f; %.5f %.5f %.5f; %.5f %.5f %.5f]",
        config_.sensor.lidar_topic.c_str(), config_.sensor.imu_topic.c_str(),
        config_.output.world_frame.c_str(), config_.output.odom_frame.c_str(),
        config_.output.imu_frame.c_str(),
        config_.output.lidar_frame.c_str(), config_.sensor.lidar_to_imu_offset_sec,
        processor_options.lidar_extrinsics.p_IL.x(), processor_options.lidar_extrinsics.p_IL.y(),
        processor_options.lidar_extrinsics.p_IL.z(), processor_options.lidar_extrinsics.R_IL(0, 0),
        processor_options.lidar_extrinsics.R_IL(0, 1),
        processor_options.lidar_extrinsics.R_IL(0, 2),
        processor_options.lidar_extrinsics.R_IL(1, 0),
        processor_options.lidar_extrinsics.R_IL(1, 1),
        processor_options.lidar_extrinsics.R_IL(1, 2),
        processor_options.lidar_extrinsics.R_IL(2, 0),
        processor_options.lidar_extrinsics.R_IL(2, 1),
        processor_options.lidar_extrinsics.R_IL(2, 2));
    RCLCPP_INFO(get_logger(), "LIO TF publishers: map_to_odom=%s, odom_to_imu=%s, imu_to_lidar=%s",
                config_.output.publish_map_to_odom ? "true" : "false",
                config_.output.publish_odom_to_imu ? "true" : "false",
                config_.output.publish_imu_to_lidar ? "true" : "false");
    RCLCPP_INFO(get_logger(), "LiDAR near-field filter: min_range=%.3f m (%s)",
                processor_options.lidar_min_range,
                processor_options.lidar_min_range > 0.0 ? "enabled" : "disabled");
  }

  ~LioNode() override {
    running_.store(false);
    sensor_buffer_.stop();
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
    if (refinement_imu_.is_open()) {refinement_imu_.flush(); refinement_imu_.close();}
    // Save the online backend result. Shutdown-only loop refinement changed
    // the graph after the last published path and bypassed evaluation logs.
    // Refinement must be an explicit operation, not a destructor side effect.
    if (evaluation_reporter_ && diagnostics_reporter_) {
      evaluation_reporter_->finalize(diagnostics_reporter_->snapshot());
    }
    if (output_publisher_) {
      const bool saved = output_publisher_->savePcd();
      if (saved && config_.output.refine_after_save) {
        ros::queueOfflineRefinement(config_.output.pcd_directory, get_logger());
      } else if (config_.output.refine_after_save && config_.output.save_pcd_on_shutdown) {
        RCLCPP_ERROR(get_logger(), "Offline refinement not queued: PCD/graph export incomplete");
      }
    }
  }

private:
  void processingLoop() {
    sensor::BufferedMeasurement measurement;
    while (running_.load()) {
      if (!sensor_buffer_.waitAndPopNext(measurement, std::chrono::milliseconds(100))) {
        continue;
      }
      std::lock_guard<std::mutex> processing_lock(processing_mutex_);
      const auto measurement_type = measurement.type;
      if (config_.output.refinement_archive && measurement_type == sensor::MeasurementType::kImu) {
        // Sensor-buffer times are already aligned to the LiDAR clock. Store
        // these values before the processor consumes/calibrates the sample.
        const auto &s = measurement.imu;
        const double record[]{s.timestamp, s.gyro.x(), s.gyro.y(), s.gyro.z(),
                              s.accel.x(), s.accel.y(), s.accel.z()};
        refinement_imu_.write(reinterpret_cast<const char *>(record), sizeof(record));
        if (!refinement_imu_) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Refinement IMU archive write failed");
        }
      }
      const auto update = processor_->process(std::move(measurement));
      diagnostics_reporter_->updateProcessorStats(processor_->stats());
      const bool invalid_initialization_sample =
          measurement_type == sensor::MeasurementType::kImu &&
          update.initialization_status ==
              estimator::ImuInitializationStatus::kRejectedInvalidSample;
      const bool regressed_initialization_sample =
          measurement_type == sensor::MeasurementType::kImu &&
          update.initialization_status ==
              estimator::ImuInitializationStatus::kRejectedTimestampRegression;
      if (update.initialized_now) {
        const auto &stats = processor_->initializationStats();
        const auto &state = processor_->state();
        RCLCPP_INFO(get_logger(),
                    "IMU initialized: samples=%zu window=[%.6f, %.6f] duration=%.3f "
                    "gyro_bias=(%.5f, %.5f, %.5f) "
                    "accel_bias=(%.5f, %.5f, %.5f) gravity=(%.5f, %.5f, %.5f) "
                    "accel_mean_norm=%.5f gyro_std=%.5f accel_std=%.5f",
                    stats.sample_count, stats.first_timestamp, stats.last_timestamp,
                    stats.duration_sec, state.b_g.x(), state.b_g.y(), state.b_g.z(), state.b_a.x(),
                    state.b_a.y(), state.b_a.z(), state.g_W.x(), state.g_W.y(), state.g_W.z(),
                    stats.mean_accel.norm(), stats.gyro_std_norm(), stats.accel_std_norm());
      } else if (update.propagation_failed) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "IMU propagation rejected sample or produced invalid state");
      } else if (invalid_initialization_sample) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Ignoring non-finite IMU sample during initialization");
      } else if (regressed_initialization_sample) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Ignoring regressed IMU sample during initialization");
      }
      for (const auto &frame : update.frames) {
        handleProcessedFrame(frame);
      }
    }
  }

  void handleProcessedFrame(const system::ProcessedFrame &frame) {
    const auto &deskewed = frame.deskewed;
    const auto &result = frame.pipeline;
    if (result.backend.loop_constraint_added) {
      RCLCPP_INFO(get_logger(), "Loop committed: edges=%zu optimized=%d cost=%.6f -> %.6f",
                  result.backend.loop_constraint_count,
                  result.backend.optimization.optimized ? 1 : 0,
                  result.backend.optimization.initial_cost, result.backend.optimization.final_cost);
    }
    auto output_timing = output_publisher_->publishFrame(frame);
    if (frame.timing.callback_start_steady_sec > 0.0) {
      output_timing.callback_to_output_ms =
        (std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count() -
        frame.timing.callback_start_steady_sec) * 1000.0;
    }
    output_timing.end_to_end_time_ms =
        frame.timing.core_end_to_end_time_ms + output_timing.output_total_time_ms;
    const auto odometry_log_start = std::chrono::steady_clock::now();
    evaluation_reporter_->writeOdometry(
      deskewed.scan_end_time, frame.state_at_scan_end, frame.covariance_diagonal_at_scan_end);
    output_timing.odometry_log_time_ms = std::chrono::duration<double, std::milli>(
                                             std::chrono::steady_clock::now() - odometry_log_start)
                                             .count();
    output_timing.output_total_time_ms += output_timing.odometry_log_time_ms;
    output_timing.end_to_end_time_ms =
        frame.timing.core_end_to_end_time_ms + output_timing.output_total_time_ms;
    diagnostics_reporter_->updateFrameMetrics(frame, output_timing,
                                              output_publisher_->globalMapSize());
    evaluation_reporter_->writeFrame(frame.raw_points, deskewed, result, frame.timing,
                                     output_timing);
  }

  const ros::config::NodeConfig config_;
  std::ofstream refinement_imu_;
  sensor::MeasurementSynchronizer sensor_buffer_;
  std::unique_ptr<ros::RosInputAdapter> input_adapter_;
  std::unique_ptr<ros::DiagnosticsReporter> diagnostics_reporter_;
  // LioOutputPublisher keeps a non-owning reference to processor_. Declare
  // processor_ before the publisher so reverse destruction order tears down
  // the publisher before the object it references.
  std::unique_ptr<system::LioProcessor> processor_;
  std::unique_ptr<ros::LioOutputPublisher> output_publisher_;
  std::unique_ptr<ros::EvaluationReporter> evaluation_reporter_;
  std::mutex processing_mutex_;
  // Adapters are destroyed before their dependencies and processing mutex.
  std::unique_ptr<ros::MapPreviewServer> preview_server_;
  std::unique_ptr<ros::ReconstructionServices> reconstruction_services_;
  std::atomic_bool running_{true};
  std::thread processing_thread_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_;
};

} // namespace itera_lio

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<itera_lio::LioNode>());
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("itera_lio_node"), "Fatal error: %s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
