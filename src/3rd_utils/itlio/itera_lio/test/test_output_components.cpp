// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
#include <thread>

#include "itera_lio/output/scan_archive.hpp"
#include "itera_lio/ros/publishers/colored_cloud_publisher.hpp"
#include "itera_lio/ros/publishers/output_publisher.hpp"
#include "itera_lio/ros/publishers/state_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "itera_lio/system/lio_processor.hpp"
#include "gtest/gtest.h"

namespace {
using itera_lio::common::FixedExtrinsics;
using itera_lio::common::Pose;
using itera_lio::mapping::color::ColoredPoint;
using itera_lio::output::ScanArchive;
using itera_lio::ros::OutputPublisherOptions;
using namespace std::chrono_literals;

struct PcdContents {
  std::size_t points{0};
  std::vector<float> records;
};
PcdContents readPcd(const std::filesystem::path &path, std::size_t fields) {
  std::ifstream input(path, std::ios::binary);
  PcdContents result;
  std::string line;
  while (std::getline(input, line) && line != "DATA binary") {
    if (line.rfind("POINTS ", 0) == 0) {
      result.points = std::stoull(line.substr(7));
    }
  }
  if (!input) {
    return {};
  }
  result.records.resize(result.points * fields);
  input.read(reinterpret_cast<char *>(result.records.data()),
             result.records.size() * sizeof(float));
  if (!input) {
    return {};
  }
  return result;
}

class OutputComponentsTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    directory_ = std::filesystem::temp_directory_path() /
                 ("itera_lio_output_components_" +
                  std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  }
  void TearDown() override { std::filesystem::remove_all(directory_); }
  OutputPublisherOptions options(bool colored = false) {
    OutputPublisherOptions result;
    result.pcd_directory = directory_.string();
    result.livo_enabled = colored;
    result.full_deskewed_enabled = true;
    result.save_pcd_on_shutdown = true;
    result.publish_deskewed = false;
    result.publish_local_map = false;
    result.publish_map_updates = false;
    result.publish_optimized_map = false;
    result.global_map_max_points = 2;
    return result;
  }
  bool spinUntil(rclcpp::executors::SingleThreadedExecutor &executor,
                 const std::function<bool()> &predicate) {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(2ms);
    }
    return predicate();
  }
  itera_lio::sensor::LidarFrame scan(double timestamp, std::size_t count = 2) {
    itera_lio::sensor::LidarFrame result;
    result.scan_begin_time = timestamp;
    result.scan_end_time = timestamp;
    for (std::size_t i = 0; i < count; ++i) {
      itera_lio::sensor::LidarPoint point;
      point.position = Eigen::Vector3d(0, 0, 1 + i);
      point.intensity = float(i + 7);
      result.points.push_back(point);
    }
    return result;
  }
  std::filesystem::path directory_;
};

TEST_F(OutputComponentsTest, RefinementKeepsFullLocalScansAlongsideColoredArchive) {
  {
    ScanArchive archive({true, true, directory_.string()});
    ASSERT_TRUE(archive.initializationError().empty());
    ASSERT_TRUE(archive.hasFullScans());
    ASSERT_TRUE(archive.hasColoredScans());
    Pose pose;
    pose.p_WI.x() = 10;
    FixedExtrinsics extrinsics;
    ASSERT_TRUE(archive.appendFull(scan(1), pose, extrinsics).success);
    ColoredPoint point;
    point.position = Eigen::Vector3f(10, 0, 1);
    point.intensity = 7;
    ASSERT_TRUE(archive.appendColored({point}, 1, pose, extrinsics).success);
    const auto saved = archive.finalize();
    ASSERT_TRUE(saved.success) << saved.error;
    EXPECT_EQ(readPcd(saved.output_path, 5).points, 1U);
  }
  ASSERT_EQ(std::filesystem::file_size(directory_ / "full_deskewed_points.bin"), 32U);
  std::ifstream file(directory_ / "full_deskewed_points.bin", std::ios::binary);
  float point[4]{};
  file.read(reinterpret_cast<char *>(point), sizeof(point));
  EXPECT_FLOAT_EQ(point[0], 0);  // Local LiDAR geometry, not world X=10.
  EXPECT_FLOAT_EQ(point[2], 1);
  EXPECT_FLOAT_EQ(point[3], 7);
}

TEST_F(OutputComponentsTest, FullArchiveSnapshotAllowsFurtherAppendAndPreservesAllPoints) {
  ScanArchive archive({false, true, directory_.string()});
  ASSERT_TRUE(archive.initializationError().empty());
  Pose pose;
  pose.p_WI.x() = 10;
  FixedExtrinsics extrinsics;
  extrinsics.p_IL.y() = 2;
  ASSERT_TRUE(archive.appendFull(scan(1), pose, extrinsics).success);
  auto snapshot = archive.snapshot({}, {}, false);
  ASSERT_TRUE(snapshot.success) << snapshot.error;
  auto pcd = readPcd(snapshot.output_path, 4);
  ASSERT_EQ(pcd.points, 2U);
  EXPECT_FLOAT_EQ(pcd.records[0], 10);
  EXPECT_FLOAT_EQ(pcd.records[1], 2);
  EXPECT_FLOAT_EQ(pcd.records[2], 1);
  ASSERT_TRUE(archive.appendFull(scan(2), pose, extrinsics).success);
  auto final = archive.finalize();
  ASSERT_TRUE(final.success) << final.error;
  EXPECT_EQ(readPcd(final.output_path, 4).points, 4U);
  EXPECT_FALSE(archive.appendFull(scan(3), pose, extrinsics).success);
}

TEST_F(OutputComponentsTest, PcdVoxelSamplingDeduplicatesAcrossScansInMapCoordinates) {
  ScanArchive archive({false, true, directory_.string(), 0.05F});
  auto frame = scan(1, 1);
  frame.points[0].position = Eigen::Vector3d(0.01, 0.01, 0.01);
  Pose pose;
  FixedExtrinsics extrinsics;
  extrinsics.p_IL.x() = 0.01;
  ASSERT_TRUE(archive.appendFull(frame, pose, extrinsics).success);
  // Different local coordinates, same world position and voxel.
  pose.p_WI.x() = 1;
  frame.scan_end_time = 2;
  frame.points[0].position.x() = -0.99;
  ASSERT_TRUE(archive.appendFull(frame, pose, extrinsics).success);
  auto saved = archive.snapshot({}, {}, false);
  ASSERT_TRUE(saved.success) << saved.error;
  EXPECT_EQ(saved.points_written, 1U);
  auto pcd = readPcd(saved.output_path, 4);
  ASSERT_EQ(pcd.points, 1U);
  EXPECT_NEAR(pcd.records[0], 0.02F, 1e-6F);
  EXPECT_FLOAT_EQ(pcd.records[3], 7);
  // A negative world coordinate occupies a separate voxel (floor, not truncation).
  frame.scan_end_time = 3;
  frame.points[0].position.x() = -1.03;
  ASSERT_TRUE(archive.appendFull(frame, pose, extrinsics).success);
  saved = archive.finalize();
  ASSERT_TRUE(saved.success) << saved.error;
  EXPECT_EQ(saved.points_written, 2U);
  EXPECT_EQ(readPcd(saved.output_path, 4).points, 2U);
  EXPECT_EQ(std::filesystem::file_size(directory_ / "full_deskewed_points.bin"), 48U);
}

TEST_F(OutputComponentsTest, PcdVoxelSamplingRunsAfterGraphCorrection) {
  itera_lio::output::FullCloudPcdWriter writer(directory_.string(), 0.05F);
  Pose pose;
  FixedExtrinsics extrinsics;
  const itera_lio::common::MapPoint point{{0.01F, 0.01F, 0.01F}, 7};
  ASSERT_TRUE(writer.append({point}, 0, pose, extrinsics));
  pose.p_WI.x() = 1;
  ASSERT_TRUE(writer.append({point}, 1, pose, extrinsics));
  std::vector<itera_lio::backend::Keyframe> raw(2), optimized(2);
  for (std::size_t i = 0; i < 2; ++i) {
    raw[i].id = i;
    raw[i].timestamp = i;
    raw[i].pose_WI.p_WI.x() = i;
    optimized[i] = raw[i];
    optimized[i].pose_WI.p_WI.x() = 0;
  }
  auto saved = writer.optimizedSnapshot(raw, optimized);
  ASSERT_TRUE(saved.success) << saved.error;
  EXPECT_EQ(saved.points_written, 1U);
  EXPECT_EQ(readPcd(saved.output_path, 4).points, 1U);
  saved = writer.finalize();
  ASSERT_TRUE(saved.success) << saved.error;
  EXPECT_EQ(saved.points_written, 2U);
}

TEST_F(OutputComponentsTest, StateOutputSeparatesContinuousOdometryFromGlobalCorrection) {
  auto node = std::make_shared<rclcpp::Node>("state_frames_test");
  auto config = options();
  config.world_frame = "map";
  config.odom_frame = "odom";
  config.odometry_topic = "/state_test/odom";
  config.path_topic = "/state_test/path";
  config.optimized_path_topic = "/state_test/optimized_path";
  itera_lio::ros::StatePublisher publisher(*node, config);
  nav_msgs::msg::Odometry::ConstSharedPtr odometry;
  nav_msgs::msg::Path::ConstSharedPtr path, optimized;
  bool got_correction = false, got_local_pose = false;
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(config.odometry_topic, 10,
      [&](nav_msgs::msg::Odometry::ConstSharedPtr message) {odometry = message;});
  auto path_sub = node->create_subscription<nav_msgs::msg::Path>(config.path_topic, 10,
      [&](nav_msgs::msg::Path::ConstSharedPtr message) {path = message;});
  auto optimized_sub = node->create_subscription<nav_msgs::msg::Path>(config.optimized_path_topic,
      rclcpp::QoS(1).transient_local().reliable(),
      [&](nav_msgs::msg::Path::ConstSharedPtr message) {optimized = message;});
  auto tf_sub = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 100,
      [&](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
        for (const auto &tf : message->transforms) {
          if (tf.header.frame_id == "map" && tf.child_frame_id == "odom") {
            got_correction = std::abs(tf.transform.translation.x - 10.0) < 1e-9;
          }
          if (tf.header.frame_id == "odom" && tf.child_frame_id == config.imu_frame) {
            got_local_pose = std::abs(tf.transform.translation.x - 2.0) < 1e-9;
          }
        }
      });
  itera_lio::estimator::NominalState state;
  state.p_WI.x() = 2.0;
  state.R_WI = Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  state.v_WI.y() = 1.0;
  Pose correction;
  correction.p_WI.x() = 10.0;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  double timestamp = 0.0;
  ASSERT_TRUE(spinUntil(executor, [&] {
    timestamp += 0.25;
    itera_lio::common::FrameOutputTiming timing;
    publisher.publishState(timestamp, state, timing, correction);
    publisher.publishOptimizedPath(timestamp, {{0, timestamp, {state.R_WI, {12, 0, 0}}}}, timing);
    return odometry && path && optimized && got_correction && got_local_pose;
  }));
  EXPECT_EQ(odometry->header.frame_id, "odom");
  EXPECT_EQ(odometry->child_frame_id, config.imu_frame);
  EXPECT_DOUBLE_EQ(odometry->pose.pose.position.x, 2.0);
  EXPECT_NEAR(odometry->twist.twist.linear.x, 1.0, 1e-9);
  EXPECT_NEAR(odometry->twist.twist.linear.y, 0.0, 1e-9);
  EXPECT_EQ(path->header.frame_id, "odom");
  EXPECT_EQ(optimized->header.frame_id, "map");
  EXPECT_DOUBLE_EQ(optimized->poses.back().pose.position.x, 12.0);
}

TEST_F(OutputComponentsTest, RobotIntegrationRetainsMapTfWithoutPublishingSensorParents) {
  rclcpp::NodeOptions node_options;
  node_options.arguments({"--ros-args", "-r", "/tf:=/robot_tf_test/tf",
                         "-r", "/tf_static:=/robot_tf_test/tf_static"});
  auto node = std::make_shared<rclcpp::Node>("robot_tf_ownership_test", node_options);
  auto config = options();
  config.publish_odom_to_imu = false;
  config.publish_imu_to_lidar = false;
  config.odometry_topic = "/robot_tf_test/odom";
  itera_lio::ros::StatePublisher publisher(*node, config);
  bool got_map = false, forbidden_sensor_parent = false, got_odometry = false;
  auto callback = [&](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
    for (const auto &tf : message->transforms) {
      got_map |= tf.header.frame_id == config.world_frame && tf.child_frame_id == config.odom_frame;
      forbidden_sensor_parent |= tf.child_frame_id == config.imu_frame ||
                                 tf.child_frame_id == config.lidar_frame;
    }
  };
  auto dynamic_sub = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, callback);
  auto static_sub = node->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(1).transient_local().reliable(), callback);
  auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(config.odometry_topic, 10,
    [&](nav_msgs::msg::Odometry::ConstSharedPtr) {got_odometry = true;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(spinUntil(executor, [&] {
    itera_lio::common::FrameOutputTiming timing;
    publisher.publishState(1.0, {}, timing);
    return got_map && got_odometry;
  }));
  EXPECT_FALSE(forbidden_sensor_parent);
  EXPECT_EQ(node->count_publishers("/tf_static"), 0U);
}

TEST_F(OutputComponentsTest, ColoredArchiveRoundTripsWorldCoordinatesAndKeepsRepeatedObservations) {
  ScanArchive archive({true, false, directory_.string()});
  Pose pose;
  pose.R_WI = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  pose.p_WI = Eigen::Vector3d(10, 2, 1);
  FixedExtrinsics extrinsics;
  extrinsics.R_IL = Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  extrinsics.p_IL = Eigen::Vector3d(0.1, -0.2, 0.3);
  ColoredPoint point;
  point.position = Eigen::Vector3f(12, 4, 3);
  point.intensity = 7;
  point.r = 25;
  point.g = 50;
  point.b = 75;
  ASSERT_TRUE(archive.appendColored({point, point}, 1.0, pose, extrinsics).success);
  std::vector<itera_lio::backend::Keyframe> raw(2);
  raw[0].id = 0;
  raw[0].timestamp = 0;
  raw[1].id = 1;
  raw[1].timestamp = 2;
  auto optimized = raw;
  optimized[1].pose_WI.p_WI.x() = 2;
  auto corrected = archive.optimizedSnapshot(raw, optimized);
  ASSERT_TRUE(corrected.success) << corrected.error;
  auto pcd = readPcd(corrected.output_path, 5);
  ASSERT_EQ(pcd.points, 2U);
  EXPECT_NEAR(pcd.records[0], 13, 1e-5);
  EXPECT_NEAR(pcd.records[1], 4, 1e-5);
  std::uint32_t rgb;
  std::memcpy(&rgb, &pcd.records[4], sizeof(rgb));
  EXPECT_EQ(rgb, (25U << 16U) | (50U << 8U) | 75U);
  auto original = archive.finalize();
  ASSERT_TRUE(original.success) << original.error;
  auto original_pcd = readPcd(original.output_path, 5);
  ASSERT_EQ(original_pcd.points, 2U);
  EXPECT_NEAR(original_pcd.records[0], 12, 1e-5);
}

TEST_F(OutputComponentsTest, OptimizedPreviewCapturesValidPrefixDuringConcurrentAppend) {
  ScanArchive archive({true, false, directory_.string()});
  std::vector<itera_lio::backend::Keyframe> frames(2);
  frames[0].id = 0;
  frames[0].timestamp = 0;
  frames[1].id = 1;
  frames[1].timestamp = 100;
  ColoredPoint point;
  point.position = Eigen::Vector3f(1, 2, 3);
  ASSERT_TRUE(archive.appendColored({point}, 0, {}, {}).success);
  std::atomic<bool> append_ok{true};
  std::thread producer([&] {
    for (int i = 1; i <= 64; ++i) {
      if (!archive.appendColored({point}, i, {}, {}).success) {
        append_ok = false;
      }
    }
  });
  bool previews_ok = true;
  for (int i = 0; i < 8; ++i) {
    const auto preview = archive.optimizedPreview(frames, frames);
    previews_ok = previews_ok && preview.success && preview.points_written > 0;
    if (preview.success) {
      std::ifstream manifest(preview.output_path);
      const std::string json((std::istreambuf_iterator<char>(manifest)), {});
      previews_ok =
          previews_ok && json.find("scanforge-blocks-v1") != std::string::npos && !json.empty() &&
          json.back() == '}' &&
          json.find("\"points\":" + std::to_string(preview.points_written)) != std::string::npos;
      previews_ok = previews_ok && std::filesystem::file_size(directory_ / "colored_points.bin") >=
                                       preview.points_written * 5 * sizeof(float);
    }
  }
  producer.join();
  EXPECT_TRUE(append_ok.load());
  EXPECT_TRUE(previews_ok);
  const auto final = archive.finalize();
  ASSERT_TRUE(final.success) << final.error;
  EXPECT_EQ(final.points_written, 65U);
}

TEST_F(OutputComponentsTest, FacadeKeepsCompleteLioArchiveIndependentOfPreviewLimit) {
  auto node = std::make_shared<rclcpp::Node>("output_lio_test");
  itera_lio::system::LioProcessor processor(itera_lio::system::LioProcessorOptions{});
  itera_lio::ros::LioOutputPublisher output(*node, options(), processor);
  itera_lio::system::ProcessedFrame frame;
  frame.full_deskewed = scan(1, 40);
  frame.deskewed = frame.full_deskewed;
  for (const auto &point : frame.full_deskewed.points) {
    frame.pipeline.map_updates.push_back({point.position.cast<float>(), point.intensity});
  }
  output.publishFrame(frame);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (output.globalMapSize() < 2 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_EQ(output.globalMapSize(), 2U);
  std::string message;
  ASSERT_TRUE(output.saveSnapshot(message)) << message;
  EXPECT_EQ(readPcd(directory_ / "cloud.pcd", 4).points, 40U);
  output.savePcd();
  EXPECT_EQ(readPcd(directory_ / "cloud.pcd", 4).points, 40U);
}

TEST_F(OutputComponentsTest, ColoredWorkerDrainsPendingScansAndStopIsIdempotent) {
  auto node = std::make_shared<rclcpp::Node>("color_stop_test");
  auto config = options(true);
  config.image_sync_wait_sec = 0;
  std::atomic<int> completed{0};
  itera_lio::ros::ColoredCloudPublisher worker(
      *node, config, {}, [](const auto &, const auto &, bool, double) {},
      [&](double) { ++completed; });
  for (int i = 0; i < 3; ++i) {
    worker.noteLidarArrival(i + 1);
    worker.enqueueColoredFrame(scan(i + 1), {}, true, i + 1);
  }
  worker.stop();
  worker.stop();
  EXPECT_EQ(completed.load(), 3);
  worker.enqueueColoredFrame(scan(4), {}, true, 4);
  EXPECT_EQ(completed.load(), 3);
}

TEST_F(OutputComponentsTest, SensorWatermarkKeepsDelayedImagesAndSkipsMissingScansWithoutSerialWaits) {
  auto node = std::make_shared<rclcpp::Node>("color_watermark_test");
  auto config = options(true);
  config.image_sync_wait_sec = 0.02;
  config.camera.width = config.camera.height = 3;
  config.camera.fx = config.camera.fy = config.camera.cx = config.camera.cy = 1;
  config.camera.R_CL.setIdentity();
  config.camera.p_CL.setZero();
  std::atomic<int> completed{0}, colored{0};
  itera_lio::ros::ColoredCloudPublisher worker(
      *node, config, {}, [&](const auto &, const auto &, bool, double) {++colored;},
      [&](double) { ++completed; });
  auto images = node->create_publisher<sensor_msgs::msg::Image>(config.image_topic, 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(spinUntil(executor, [&] {return images->get_subscription_count() > 0;}));
  for (int i = 1; i <= 8; ++i) {
    worker.noteLidarArrival(i);
    worker.enqueueColoredFrame(scan(i), {}, true, i);
  }
  std::this_thread::sleep_for(100ms); // Deliberately longer than the reorder margin.
  EXPECT_EQ(completed.load(), 0);
  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 1;
  image.width = image.height = 3;
  image.step = 9;
  image.encoding = "rgb8";
  image.data.resize(27, 100);
  images->publish(image);
  ASSERT_TRUE(spinUntil(executor, [&] {return colored.load() == 1;}));
  image.header.stamp.sec = 8;
  images->publish(image);
  ASSERT_TRUE(spinUntil(executor, [&] {return completed.load() == 8;}));
  worker.stop();
  EXPECT_EQ(colored.load(), 2);
}

TEST_F(OutputComponentsTest, FacadeMatchesImageColorsPublishesAndSavesTrustedScans) {
  auto node = std::make_shared<rclcpp::Node>("output_livo_test");
  auto config = options(true);
  config.camera.width = 3;
  config.camera.height = 3;
  config.camera.fx = 1;
  config.camera.fy = 1;
  config.camera.cx = 1;
  config.camera.cy = 1;
  config.camera.k1 = 0;
  config.camera.k2 = 0;
  config.camera.p1 = 0;
  config.camera.p2 = 0;
  config.camera.R_CL.setIdentity();
  config.camera.p_CL.setZero();
  itera_lio::system::LioProcessor processor(itera_lio::system::LioProcessorOptions{});
  itera_lio::ros::LioOutputPublisher output(*node, config, processor);
  auto images = node->create_publisher<sensor_msgs::msg::Image>(config.image_topic, 10);
  std::size_t received = 0;
  auto colored = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      config.colored_topic, rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        received += message->width * message->height;
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(spinUntil(executor, [&] { return images->get_subscription_count() > 0; }));
  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 1;
  image.width = 3;
  image.height = 3;
  image.step = 9;
  image.encoding = "rgb8";
  image.data.resize(27, 0);
  image.data[12] = 25;
  image.data[13] = 50;
  image.data[14] = 75;
  output.noteLidarArrival(1);
  images->publish(image);
  itera_lio::system::ProcessedFrame frame;
  frame.full_deskewed = scan(1);
  frame.deskewed = frame.full_deskewed;
  frame.pipeline.initialized_map = true;
  output.publishFrame(frame);
  ASSERT_TRUE(spinUntil(executor, [&] { return received == 2; }));
  // Untrusted scans remain visible but must not enter the persistent archive.
  image.header.stamp.sec = 2;
  output.noteLidarArrival(2);
  images->publish(image);
  frame.full_deskewed = scan(2);
  frame.deskewed = frame.full_deskewed;
  frame.pipeline.initialized_map = false;
  output.publishFrame(frame);
  ASSERT_TRUE(spinUntil(executor, [&] { return received == 4; }));
  output.savePcd();
  EXPECT_EQ(output.globalMapSize(), 2U);
  const auto pcd = readPcd(directory_ / "cloud_color.pcd", 5);
  ASSERT_EQ(pcd.points, 2U);
  std::uint32_t rgb;
  std::memcpy(&rgb, &pcd.records[4], sizeof(rgb));
  EXPECT_EQ(rgb, (25U << 16U) | (50U << 8U) | 75U);
  EXPECT_FLOAT_EQ(pcd.records[2], 1);
}

} // namespace


TEST_F(OutputComponentsTest, FullWriterBatchesPreserveBinaryLayoutAndInvalidPointFiltering) {
  itera_lio::output::FullCloudPcdWriter writer(directory_.string());
  std::vector<itera_lio::common::MapPoint> points;
  std::vector<float> expected;
  for (int i = 0; i < 10003; ++i) {
    const float x = i == 0 ? -0.0F : i * .01F;
    points.push_back({{x, i * -.02F, i * .03F}, float(i)});
    if (i % 101 == 0 && i != 0) {
      points.back().intensity = std::numeric_limits<float>::quiet_NaN();
    } else if (i % 137 == 0 && i != 0) {
      points.back().position.y() = std::numeric_limits<float>::infinity();
    } else {
      expected.insert(expected.end(), {x, i * -.02F, i * .03F, float(i)});
    }
  }
  ASSERT_TRUE(writer.append(points, 1.0));
  auto snapshot = writer.snapshot();
  ASSERT_TRUE(snapshot.success) << snapshot.error;
  // Compare the stored LiDAR records before world-frame export, including -0.
  std::ifstream raw(directory_ / "full_deskewed_points.bin", std::ios::binary);
  std::vector<float> archived(expected.size());
  raw.read(reinterpret_cast<char *>(archived.data()), archived.size() * sizeof(float));
  ASSERT_TRUE(raw);
  EXPECT_EQ(std::memcmp(archived.data(), expected.data(), expected.size() * sizeof(float)), 0);
  // The unchanged world-frame transform adds +0 even for an identity pose.
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (i % 4U != 3U && expected[i] == 0.0F) {expected[i] = 0.0F;}
  }
  auto pcd = readPcd(snapshot.output_path, 4);
  ASSERT_EQ(pcd.records.size(), expected.size());
  EXPECT_EQ(std::memcmp(pcd.records.data(), expected.data(), expected.size() * sizeof(float)), 0);
  ASSERT_TRUE(writer.append({}, 2.0));
  ASSERT_TRUE(writer.append(points, 3.0));
  auto final = writer.finalize();
  ASSERT_TRUE(final.success) << final.error;
  auto doubled = expected;
  doubled.insert(doubled.end(), expected.begin(), expected.end());
  pcd = readPcd(final.output_path, 4);
  ASSERT_EQ(pcd.records.size(), doubled.size());
  EXPECT_EQ(std::memcmp(pcd.records.data(), doubled.data(), doubled.size() * sizeof(float)), 0);
  EXPECT_FALSE(writer.append(points, 4.0));
}

TEST_F(OutputComponentsTest, FullWriterReportsBatchWriteFailureAndRejectsFinalization) {
  if (!std::filesystem::exists("/dev/full")) {GTEST_SKIP();}
  std::filesystem::create_directories(directory_);
  std::filesystem::create_symlink("/dev/full", directory_ / "full_deskewed_points.bin");
  itera_lio::output::FullCloudPcdWriter writer(directory_.string());
  std::vector<itera_lio::common::MapPoint> points(10000);
  ASSERT_TRUE(writer.error().empty());
  EXPECT_FALSE(writer.append(points, 1.0));
  EXPECT_FALSE(writer.error().empty());
  EXPECT_FALSE(writer.finalize().success);
  EXPECT_FALSE(writer.append(points, 2.0));
}
