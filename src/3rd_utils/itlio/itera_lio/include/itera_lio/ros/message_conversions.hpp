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

#ifndef ITERA_LIO__ROS__MESSAGE_CONVERSIONS_HPP_
#define ITERA_LIO__ROS__MESSAGE_CONVERSIONS_HPP_

#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "itera_lio/common/geometry.hpp"
#include "itera_lio/common/map_types.hpp"
#include "itera_lio/common/observation_types.hpp"
#include "itera_lio/estimator/state.hpp"
#include "itera_lio/mapping/color/colorizer.hpp"
#include "itera_lio/sensor/lidar_frame.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace itera_lio::ros {

builtin_interfaces::msg::Time toRosTime(double timestamp);

sensor_msgs::msg::PointCloud2 toLidarPointCloud2(const sensor::LidarFrame &scan,
                                                 const std::string &frame_id);

sensor_msgs::msg::PointCloud2 toRegisteredPointCloud2(
    const sensor::LidarFrame &scan, const estimator::NominalState &state,
    const common::FixedExtrinsics &extrinsics, const std::string &frame_id);

sensor_msgs::msg::PointCloud2 toObservationPointCloud2(
    const std::vector<common::PlaneObservation> &observations, double timestamp,
    const std::string &frame_id);
sensor_msgs::msg::PointCloud2
toColoredPointCloud2(const std::vector<mapping::color::ColoredPoint> &,
                     double timestamp, const std::string &frame_id);

sensor_msgs::msg::PointCloud2
toMapPointCloud2(const std::vector<common::MapPoint> &points, double timestamp,
                 const std::string &frame_id);

nav_msgs::msg::Odometry toOdometry(const estimator::NominalState &state,
                                   double timestamp,
                                   const std::string &world_frame,
                                   const std::string &imu_frame);

} // namespace itera_lio::ros

#endif // ITERA_LIO__ROS__MESSAGE_CONVERSIONS_HPP_
