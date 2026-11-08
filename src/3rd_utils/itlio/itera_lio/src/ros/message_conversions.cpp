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

#include "itera_lio/ros/message_conversions.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <cstring>

#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace itera_lio::ros {

builtin_interfaces::msg::Time toRosTime(const double timestamp) {
  builtin_interfaces::msg::Time result;
  if (!std::isfinite(timestamp) || timestamp < 0.0) {
    return result;
  }
  auto seconds = static_cast<std::int64_t>(timestamp);
  auto nanoseconds = static_cast<std::int64_t>((timestamp - seconds) * 1e9);
  if (nanoseconds >= 1000000000LL) {
    nanoseconds -= 1000000000LL;
    ++seconds;
  }
  result.sec = static_cast<std::int32_t>(seconds);
  result.nanosec = static_cast<std::uint32_t>(nanoseconds);
  return result;
}

sensor_msgs::msg::PointCloud2 toLidarPointCloud2(const sensor::LidarFrame &scan,
                                                 const std::string &frame_id) {
  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp = toRosTime(scan.scan_end_time);
  message.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(scan.points.size());
  sensor_msgs::PointCloud2Iterator<float> x(message, "x");
  sensor_msgs::PointCloud2Iterator<float> y(message, "y");
  sensor_msgs::PointCloud2Iterator<float> z(message, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(message, "intensity");
  for (const auto &point : scan.points) {
    *x = static_cast<float>(point.position.x());
    *y = static_cast<float>(point.position.y());
    *z = static_cast<float>(point.position.z());
    *intensity = point.intensity;
    ++x;
    ++y;
    ++z;
    ++intensity;
  }
  return message;
}

sensor_msgs::msg::PointCloud2 toRegisteredPointCloud2(
    const sensor::LidarFrame &scan, const estimator::NominalState &state,
    const common::FixedExtrinsics &extrinsics, const std::string &frame_id) {
  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp = toRosTime(scan.scan_end_time);
  message.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(scan.points.size());
  sensor_msgs::PointCloud2Iterator<float> x(message, "x");
  sensor_msgs::PointCloud2Iterator<float> y(message, "y");
  sensor_msgs::PointCloud2Iterator<float> z(message, "z");
  for (const auto &point : scan.points) {
    const Eigen::Vector3d point_I =
        extrinsics.R_IL * point.position + extrinsics.p_IL;
    const Eigen::Vector3d point_W = state.R_WI * point_I + state.p_WI;
    *x = static_cast<float>(point_W.x());
    *y = static_cast<float>(point_W.y());
    *z = static_cast<float>(point_W.z());
    ++x;
    ++y;
    ++z;
  }
  return message;
}

sensor_msgs::msg::PointCloud2 toObservationPointCloud2(
    const std::vector<common::PlaneObservation> &observations,
    const double timestamp, const std::string &frame_id) {
  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp = toRosTime(timestamp);
  message.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(observations.size());
  sensor_msgs::PointCloud2Iterator<float> x(message, "x");
  sensor_msgs::PointCloud2Iterator<float> y(message, "y");
  sensor_msgs::PointCloud2Iterator<float> z(message, "z");
  for (const auto &observation : observations) {
    *x = static_cast<float>(observation.point_W.x());
    *y = static_cast<float>(observation.point_W.y());
    *z = static_cast<float>(observation.point_W.z());
    ++x;
    ++y;
    ++z;
  }
  return message;
}

sensor_msgs::msg::PointCloud2 toColoredPointCloud2(
    const std::vector<mapping::color::ColoredPoint> &points,
    const double timestamp, const std::string &frame_id) {
  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp = toRosTime(timestamp);
  message.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2Fields(
      5, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
      sensor_msgs::msg::PointField::FLOAT32, "rgb", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(message, "x"), y(message, "y"),
      z(message, "z"), i(message, "intensity");
  sensor_msgs::PointCloud2Iterator<float> rgb(message, "rgb");
  for (const auto &p : points) {
    std::uint32_t packed = (static_cast<std::uint32_t>(p.r) << 16) |
                           (static_cast<std::uint32_t>(p.g) << 8) | p.b;
    float value;
    std::memcpy(&value, &packed, sizeof(value));
    *x = p.position.x();
    *y = p.position.y();
    *z = p.position.z();
    *i = p.intensity;
    *rgb = value;
    ++x;
    ++y;
    ++z;
    ++i;
    ++rgb;
  }
  return message;
}

sensor_msgs::msg::PointCloud2
toMapPointCloud2(const std::vector<common::MapPoint> &points,
                 const double timestamp, const std::string &frame_id) {
  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp = toRosTime(timestamp);
  message.header.frame_id = frame_id;
  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
      sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(message, "x");
  sensor_msgs::PointCloud2Iterator<float> y(message, "y");
  sensor_msgs::PointCloud2Iterator<float> z(message, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(message, "intensity");
  for (const auto &point : points) {
    *x = point.position.x();
    *y = point.position.y();
    *z = point.position.z();
    *intensity = point.intensity;
    ++x;
    ++y;
    ++z;
    ++intensity;
  }
  return message;
}

nav_msgs::msg::Odometry toOdometry(const estimator::NominalState &state,
                                   const double timestamp,
                                   const std::string &world_frame,
                                   const std::string &imu_frame) {
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = toRosTime(timestamp);
  odometry.header.frame_id = world_frame;
  odometry.child_frame_id = imu_frame;
  const Eigen::Quaterniond quaternion(state.R_WI);
  odometry.pose.pose.position.x = state.p_WI.x();
  odometry.pose.pose.position.y = state.p_WI.y();
  odometry.pose.pose.position.z = state.p_WI.z();
  odometry.pose.pose.orientation.w = quaternion.w();
  odometry.pose.pose.orientation.x = quaternion.x();
  odometry.pose.pose.orientation.y = quaternion.y();
  odometry.pose.pose.orientation.z = quaternion.z();
  const Eigen::Vector3d velocity_I = state.R_WI.transpose() * state.v_WI;
  odometry.twist.twist.linear.x = velocity_I.x();
  odometry.twist.twist.linear.y = velocity_I.y();
  odometry.twist.twist.linear.z = velocity_I.z();
  return odometry;
}

} // namespace itera_lio::ros
