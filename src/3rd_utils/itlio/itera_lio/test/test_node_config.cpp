// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "itera_lio/ros/config/node_config.hpp"
#include "gtest/gtest.h"

namespace {

class NodeConfigTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> makeNode(const std::vector<rclcpp::Parameter> &overrides = {},
                                         const std::vector<std::string> &profiles = {}) {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).parameter_overrides(overrides);
    if (!profiles.empty()) {
      std::vector<std::string> arguments{"--ros-args"};
      for (const auto &profile : profiles) {
        arguments.push_back("--params-file");
        arguments.push_back(std::string(ITERA_LIO_CONFIG_DIR) + "/" + profile);
      }
      options.arguments(arguments);
    }
    return std::make_shared<rclcpp::Node>("itera_lio_node", options);
  }
};

TEST_F(NodeConfigTest, DefaultsPreserveProcessingAndOutputContracts) {
  auto node = makeNode();
  const auto config = itera_lio::ros::config::loadNodeConfig(*node);
  EXPECT_EQ(config.sensor.lidar_topic, "/interfaces/lidar");
  EXPECT_EQ(config.sensor.lidar_queue_capacity, 8U);
  EXPECT_DOUBLE_EQ(config.sensor.lidar_to_imu_offset_sec, 0.0);
  EXPECT_FALSE(config.output.livo_enabled);
  EXPECT_DOUBLE_EQ(config.output.camera.image_scale, 1.0);
  EXPECT_FALSE(config.processor.pipeline.backend.enabled);
  EXPECT_FALSE(config.processor.pipeline.loop_closure.enabled);
  EXPECT_FALSE(config.evaluation.outputs.detailed);
  EXPECT_EQ(config.processor.pipeline.loop_closure.verifier.registration_engine, "manual_gicp");
  EXPECT_TRUE(config.processor.full_deskewed_enabled);
  EXPECT_EQ(config.processor.point_filter_num, 1U);
  EXPECT_TRUE(config.output.full_deskewed_enabled);
  EXPECT_TRUE(config.processor.pipeline.input_prevoxelized);
  EXPECT_EQ(config.output.path_max_poses, 0U);
  EXPECT_DOUBLE_EQ(config.diagnostic_period_sec, 5.0);
  EXPECT_DOUBLE_EQ(config.processor.pipeline.correction.gravity_magnitude,
                   config.processor.imu_initialization.gravity_magnitude);
  const std::filesystem::path run(config.output.pcd_directory);
  EXPECT_TRUE(run.is_absolute());
  EXPECT_EQ(std::filesystem::path(config.evaluation.output_directory), run / "evaluation");
  EXPECT_EQ(std::filesystem::path(config.output.image_recorder.directory), run / "images");
  // Naming a run during config loading must not create output files.
  EXPECT_FALSE(std::filesystem::exists(run));
}

TEST_F(NodeConfigTest, OverridesReachModulesAndDerivedSettings) {
  auto node = makeNode({{"topics.lidar", "/test/lidar"},
                        {"frames.world", "map"},
                        {"time_offset.lidar_to_imu", 0.012},
                        {"sensor.lidar_queue_capacity", 17},
                        {"mode.type", "livo"},
                        {"backend.enabled", true},
                        {"loop_closure.enabled", true},
                        {"loop_closure.verifier.registration_engine", "manual_gicp"},
                        {"voxel.scan_size", 0.1},
                        {"preprocess.point_filter_num", 3},
                        {"voxel.deskew_size", 0.2},
                        {"imu.gravity_magnitude", 9.8},
                        {"imu.accel_norm_threshold", 0.0},
                        {"imu.initialize_accel_bias_from_static_mean", false},
                        {"imu.update_gravity_after_initialization", true},
                        {"imu.gravity_magnitude_stddev", 0.0},
                        {"preview.global_map_snapshot_frequency_hz", 2.0},
                        {"diagnostics.log_frequency_hz", 4.0},
                        {"camera.intrinsics.fx", 800.0},
                        {"camera.image_scale", 0.5},
                        {"camera.width", 1920},
                        {"camera.height", 1080},
                        {"camera.image_queue_capacity", 7},
                        {"image_record.directory", "photos"},
                        {"save_pcd.pcd_directory", "/tmp/itera_lio_config_test"},
                        {"evaluation.output_dir", "/tmp/itera_lio_config_test"},
                        {"evaluation.outputs.frames", false},
                        {"evaluation.detailed", true},
                        {"evaluation.slow_rebuild_ms", 75.0}});
  const auto config = itera_lio::ros::config::loadNodeConfig(*node);
  EXPECT_EQ(config.sensor.lidar_topic, "/test/lidar");
  EXPECT_EQ(config.sensor.lidar_queue_capacity, 17U);
  EXPECT_DOUBLE_EQ(config.sensor.lidar_to_imu_offset_sec, 0.012);
  EXPECT_EQ(config.output.world_frame, "map");
  EXPECT_TRUE(config.output.livo_enabled);
  EXPECT_TRUE(config.processor.pipeline.backend.enabled);
  EXPECT_TRUE(config.processor.pipeline.loop_closure.enabled);
  EXPECT_EQ(config.output.world_frame, "map");
  EXPECT_EQ(config.output.odom_frame, "odom");
  EXPECT_EQ(config.processor.pipeline.loop_closure.verifier.registration_engine, "manual_gicp");
  EXPECT_EQ(config.processor.point_filter_num, 3U);
  EXPECT_FALSE(config.processor.pipeline.input_prevoxelized);
  EXPECT_DOUBLE_EQ(config.processor.pipeline.correction.gravity_magnitude, 9.8);
  EXPECT_FALSE(config.processor.imu_initialization.initialize_accel_bias_from_static_mean);
  EXPECT_DOUBLE_EQ(config.processor.imu_initialization.accel_norm_threshold, 0.0);
  EXPECT_TRUE(config.processor.pipeline.correction.update_gravity);
  EXPECT_DOUBLE_EQ(config.processor.pipeline.correction.gravity_magnitude_stddev, 0.0);
  EXPECT_DOUBLE_EQ(config.output.global_map_snapshot_period_sec, 0.5);
  EXPECT_DOUBLE_EQ(config.diagnostic_period_sec, 0.25);
  EXPECT_DOUBLE_EQ(config.output.camera.fx, 800.0);
  EXPECT_DOUBLE_EQ(config.output.camera.image_scale, 0.5);
  EXPECT_EQ(config.output.camera.width, 1920);
  EXPECT_EQ(config.output.camera.height, 1080);
  EXPECT_EQ(config.output.image_queue_capacity, 7U);
  EXPECT_FALSE(config.evaluation.outputs.frames);
  EXPECT_TRUE(config.evaluation.outputs.detailed);
  EXPECT_DOUBLE_EQ(config.evaluation.outputs.slow_rebuild_ms, 75.0);
  EXPECT_EQ(std::filesystem::path(config.output.image_recorder.directory),
            std::filesystem::path(config.output.pcd_directory) / "photos");
}

TEST_F(NodeConfigTest, LaunchUsesUnifiedAlgorithmConfiguration) {
  const std::vector<std::string> base{"mid360.yaml", "frontend.yaml", "backend.yaml",   "loop.yaml",
                                      "output.yaml", "camera.yaml",   "evaluation.yaml"};
  auto node = makeNode({}, base);
  const auto config = itera_lio::ros::config::loadNodeConfig(*node);
  EXPECT_FALSE(node->has_parameter("backend.optimizer.gravity_aligned"));
  EXPECT_FALSE(node->has_parameter("backend.optimizer.gravity_tilt_stddev_rad"));
  EXPECT_TRUE(config.processor.pipeline.backend.enabled);
  EXPECT_TRUE(config.processor.pipeline.loop_closure.enabled);
  EXPECT_TRUE(config.processor.pipeline.correction.update_gravity);
  EXPECT_FALSE(config.processor.imu_initialization.initialize_accel_bias_from_static_mean);
  EXPECT_DOUBLE_EQ(config.processor.imu_initialization.accel_norm_threshold, 0.0);
  EXPECT_DOUBLE_EQ(config.processor.pipeline.correction.gravity_magnitude_stddev, 0.0);
  EXPECT_EQ(config.processor.pipeline.map_rebuild_policy,
            itera_lio::frontend::IkdRebuildPolicy::kHybrid);
}

TEST_F(NodeConfigTest, RejectsInvalidModuleAndCrossModuleSettings) {
  for (const auto &parameter :
       std::vector<rclcpp::Parameter>{{"sensor.lidar_queue_capacity", 0},
                                      {"frames.odom", "map"},
                                      {"frames.world", ""},
                                      {"loop_closure.verifier.registration_engine", "unknown"},
                                      {"mode.type", "unknown"},
                                      {"camera.image_scale", 0.0},
                                      {"camera.image_scale", -0.5},
                                      {"camera.width", 0},
                                      {"camera.height", -1},
                                      {"diagnostics.log_frequency_hz", 0.0},
                                      {"preview.global_map_snapshot_frequency_hz", -1.0},
                                      {"evaluation.comparison.percentile", 1.0},
                                      {"evaluation.slow_rebuild_ms", 0.0},
                                      {"save_pcd.pcd_directory", "/tmp/different_output_root"}}) {
    SCOPED_TRACE(parameter.get_name());
    auto node = makeNode({parameter});
    EXPECT_THROW(itera_lio::ros::config::loadNodeConfig(*node), std::invalid_argument);
  }
}

} // namespace
