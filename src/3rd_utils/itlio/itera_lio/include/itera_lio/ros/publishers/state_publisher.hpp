// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__PUBLISHERS__STATE_PUBLISHER_HPP_
#define ITERA_LIO__ROS__PUBLISHERS__STATE_PUBLISHER_HPP_

#include "itera_lio/backend/pose_graph.hpp"
#include "itera_lio/common/frame_metrics.hpp"
#include "itera_lio/estimator/state.hpp"
#include "itera_lio/ros/publishers/output_options.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include <deque>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace itera_lio::ros {

class StatePublisher {
public:
  StatePublisher(rclcpp::Node &node, const OutputPublisherOptions &options,
                 const common::FixedExtrinsics &extrinsics = {});
  void publishState(double timestamp, const estimator::NominalState &state,
                    common::FrameOutputTiming &timing, const common::Pose &map_to_odom = {});
  void publishOptimizedPath(double timestamp, const std::vector<backend::Keyframe> &keyframes,
                            common::FrameOutputTiming &timing);

private:
  OutputPublisherOptions options_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transforms_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_transforms_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_publisher_;
  std::deque<geometry_msgs::msg::PoseStamped> path_history_;
  double path_publish_period_sec_;
  double last_path_publish_timestamp_{-1.0};
};

} // namespace itera_lio::ros

#endif // ITERA_LIO__ROS__PUBLISHERS__STATE_PUBLISHER_HPP_
