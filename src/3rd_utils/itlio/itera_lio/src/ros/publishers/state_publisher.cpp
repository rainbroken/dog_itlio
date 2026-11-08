// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "itera_lio/ros/message_conversions.hpp"
#include "itera_lio/ros/publishers/state_publisher.hpp"
#include <Eigen/Geometry>
#include <chrono>

namespace itera_lio::ros {
namespace {
geometry_msgs::msg::TransformStamped poseTransform(
    const common::Pose &pose, const builtin_interfaces::msg::Time &stamp,
    const std::string &parent, const std::string &child) {
  geometry_msgs::msg::TransformStamped result;
  result.header.stamp = stamp;
  result.header.frame_id = parent;
  result.child_frame_id = child;
  const Eigen::Quaterniond q(pose.R_WI);
  result.transform.rotation.x = q.x();
  result.transform.rotation.y = q.y();
  result.transform.rotation.z = q.z();
  result.transform.rotation.w = q.w();
  result.transform.translation.x = pose.p_WI.x();
  result.transform.translation.y = pose.p_WI.y();
  result.transform.translation.z = pose.p_WI.z();
  return result;
}
} // namespace
StatePublisher::StatePublisher(rclcpp::Node &node, const OutputPublisherOptions &options,
                               const common::FixedExtrinsics &extrinsics)
    : options_(options), path_publish_period_sec_(1.0 / options.path_publish_frequency_hz) {
  odometry_publisher_ = node.create_publisher<nav_msgs::msg::Odometry>(options_.odometry_topic, 10);
  path_publisher_ = node.create_publisher<nav_msgs::msg::Path>(options_.path_topic, 10);
  optimized_path_publisher_ = node.create_publisher<nav_msgs::msg::Path>(
      options_.optimized_path_topic, rclcpp::QoS(1).transient_local().reliable());
  transforms_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  if (options_.publish_imu_to_lidar) {
    static_transforms_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
    static_transforms_->sendTransform(poseTransform(
        {extrinsics.R_IL, extrinsics.p_IL}, node.now(), options_.imu_frame, options_.lidar_frame));
  }
}

void StatePublisher::publishState(const double timestamp, const estimator::NominalState &state,
                                  common::FrameOutputTiming &timing,
                                  const common::Pose &map_to_odom) {
  const auto serialization_start = std::chrono::steady_clock::now();
  const auto odometry = toOdometry(state, timestamp, options_.odom_frame, options_.imu_frame);
  timing.serialization_time_ms += std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - serialization_start)
                                      .count();
  const auto odometry_publish_start = std::chrono::steady_clock::now();
  odometry_publisher_->publish(odometry);
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  if (options_.publish_map_to_odom) {
    transforms.push_back(poseTransform(map_to_odom, odometry.header.stamp,
      options_.world_frame, options_.odom_frame));
  }
  if (options_.publish_odom_to_imu) {
    transforms.push_back(poseTransform({state.R_WI, state.p_WI}, odometry.header.stamp,
      options_.odom_frame, options_.imu_frame));
  }
  if (!transforms.empty()) {transforms_->sendTransform(transforms);}
  timing.publish_call_time_ms += std::chrono::duration<double, std::milli>(
                                     std::chrono::steady_clock::now() - odometry_publish_start)
                                     .count();

  const auto path_preparation_start = std::chrono::steady_clock::now();
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odometry.header;
  pose.pose = odometry.pose.pose;
  path_history_.push_back(pose);
  if (options_.path_max_poses > 0U && path_history_.size() > options_.path_max_poses) {
    path_history_.pop_front();
  }
  timing.preparation_time_ms += std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - path_preparation_start)
                                    .count();
  if (path_publisher_->get_subscription_count() > 0U) {
    const bool period_elapsed =
        last_path_publish_timestamp_ < 0.0 ||
        timestamp >= last_path_publish_timestamp_ + path_publish_period_sec_;
    if (period_elapsed) {
      const auto path_serialization_start = std::chrono::steady_clock::now();
      nav_msgs::msg::Path path;
      path.header.stamp = odometry.header.stamp;
      path.header.frame_id = options_.odom_frame;
      path.poses.assign(path_history_.begin(), path_history_.end());
      timing.serialization_time_ms +=
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                    path_serialization_start)
              .count();
      const auto path_publish_start = std::chrono::steady_clock::now();
      path_publisher_->publish(path);
      timing.publish_call_time_ms += std::chrono::duration<double, std::milli>(
                                         std::chrono::steady_clock::now() - path_publish_start)
                                         .count();
      last_path_publish_timestamp_ = timestamp;
    }
  }
}

void StatePublisher::publishOptimizedPath(const double timestamp,
                                          const std::vector<backend::Keyframe> &optimized_keyframes,
                                          common::FrameOutputTiming &timing) {
  const auto start = std::chrono::steady_clock::now();
  nav_msgs::msg::Path path;
  path.header.frame_id = options_.world_frame;
  path.header.stamp = toRosTime(timestamp);
  for (const auto &keyframe : optimized_keyframes) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = options_.world_frame;
    pose.header.stamp = toRosTime(keyframe.timestamp);
    const Eigen::Quaterniond rotation(keyframe.pose_WI.R_WI);
    pose.pose.orientation.x = rotation.x();
    pose.pose.orientation.y = rotation.y();
    pose.pose.orientation.z = rotation.z();
    pose.pose.orientation.w = rotation.w();
    pose.pose.position.x = keyframe.pose_WI.p_WI.x();
    pose.pose.position.y = keyframe.pose_WI.p_WI.y();
    pose.pose.position.z = keyframe.pose_WI.p_WI.z();
    path.poses.push_back(pose);
  }
  timing.serialization_time_ms +=
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  const auto publish_start = std::chrono::steady_clock::now();
  optimized_path_publisher_->publish(path);
  timing.publish_call_time_ms +=
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - publish_start)
          .count();
}

} // namespace itera_lio::ros
