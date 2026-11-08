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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace interfaces
{

class LivoxToPointCloud2Node final : public rclcpp::Node
{
public:
  LivoxToPointCloud2Node()
  : Node("livox_to_pointcloud2")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/interfaces/lidar");
    imu_input_topic_ = declare_parameter<std::string>("imu_input_topic", "/livox/imu");
    imu_output_topic_ =
      declare_parameter<std::string>("imu_output_topic", "/interfaces/imu");
    imu_acceleration_scale_ = declare_parameter<double>("imu_acceleration_scale", 9.80665);
    if (!std::isfinite(imu_acceleration_scale_) || imu_acceleration_scale_ <= 0.0) {
      throw std::invalid_argument("imu_acceleration_scale must be finite and positive");
    }
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "");

    const auto reliability =
      declare_parameter<std::string>("qos.reliability", "best_effort");
    const auto lidar_depth = declarePositiveDepth("qos.lidar_depth", 32);
    const auto imu_depth = declarePositiveDepth("qos.imu_depth", 2000);
    const auto lidar_qos = makeQos(lidar_depth, reliability);
    const auto imu_qos = makeQos(imu_depth, reliability);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, lidar_qos);
    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_, lidar_qos,
      std::bind(&LivoxToPointCloud2Node::convertAndPublish, this, std::placeholders::_1));
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_output_topic_, imu_qos);
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input_topic_, imu_qos,
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        ++imu_received_;
        auto converted = *msg;
        if (!imu_frame_id_.empty()) {converted.header.frame_id = imu_frame_id_;}
        converted.linear_acceleration.x *= imu_acceleration_scale_;
        converted.linear_acceleration.y *= imu_acceleration_scale_;
        converted.linear_acceleration.z *= imu_acceleration_scale_;
        imu_publisher_->publish(converted);
        ++imu_published_;
      });

    RCLCPP_INFO(
      get_logger(),
      "Bridging lidar '%s' -> '%s', IMU '%s' -> '%s' with %s QoS "
      "(depth L/I=%zu/%zu)",
      input_topic_.c_str(), output_topic_.c_str(),
      imu_input_topic_.c_str(), imu_output_topic_.c_str(), reliability.c_str(),
      lidar_depth, imu_depth);
  }

  ~LivoxToPointCloud2Node() override
  {
    RCLCPP_INFO(
      get_logger(),
      "Bridge totals: lidar received/published=%zu/%zu, IMU received/published=%zu/%zu",
      lidar_received_, lidar_published_, imu_received_, imu_published_);
  }

private:
  static constexpr std::uint32_t kPointStep = 20U;

  std::size_t declarePositiveDepth(const std::string & name, const int default_value)
  {
    const auto value = declare_parameter<int>(name, default_value);
    if (value <= 0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return static_cast<std::size_t>(value);
  }

  static rclcpp::QoS makeQos(const std::size_t depth, const std::string & reliability)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(depth)).durability_volatile();
    if (reliability == "reliable") {
      return qos.reliable();
    }
    if (reliability == "best_effort") {
      return qos.best_effort();
    }
    throw std::invalid_argument("qos.reliability must be reliable or best_effort");
  }

  static sensor_msgs::msg::PointField makeField(
    const std::string & name, const std::uint32_t offset, const std::uint8_t datatype)
  {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1U;
    return field;
  }

  template<typename T>
  static void writeValue(
    std::vector<std::uint8_t> & data, const std::size_t offset, const T & value)
  {
    std::memcpy(data.data() + offset, &value, sizeof(T));
  }

  void convertAndPublish(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
  {
    ++lidar_received_;
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;
    if (!frame_id_.empty()) {
      cloud.header.frame_id = frame_id_;
    }

    // Keep every field carried by Livox CustomPoint. Padding makes each point
    // 4-byte aligned while preserving the values byte-for-byte.
    cloud.fields = {
      makeField("x", 0U, sensor_msgs::msg::PointField::FLOAT32),
      makeField("y", 4U, sensor_msgs::msg::PointField::FLOAT32),
      makeField("z", 8U, sensor_msgs::msg::PointField::FLOAT32),
      makeField("reflectivity", 12U, sensor_msgs::msg::PointField::UINT8),
      makeField("tag", 13U, sensor_msgs::msg::PointField::UINT8),
      makeField("line", 14U, sensor_msgs::msg::PointField::UINT8),
      makeField("offset_time", 16U, sensor_msgs::msg::PointField::UINT32),
    };

    const std::size_t point_count =
      std::min<std::size_t>(msg->point_num, msg->points.size());
    if (static_cast<std::size_t>(msg->point_num) != msg->points.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "CustomMsg point_num (%u) differs from points.size() (%zu); converting %zu points",
        msg->point_num, msg->points.size(), point_count);
    }

    cloud.height = 1U;
    cloud.width = static_cast<std::uint32_t>(point_count);
    cloud.is_bigendian = false;
    cloud.point_step = kPointStep;
    cloud.row_step = cloud.point_step * cloud.width;
    // Livox does not provide an explicit validity flag for all coordinates.
    cloud.is_dense = false;
    cloud.data.assign(static_cast<std::size_t>(cloud.row_step), 0U);

    for (std::size_t i = 0; i < point_count; ++i) {
      const auto & point = msg->points[i];
      const std::size_t base = i * kPointStep;
      writeValue(cloud.data, base + 0U, point.x);
      writeValue(cloud.data, base + 4U, point.y);
      writeValue(cloud.data, base + 8U, point.z);
      writeValue(cloud.data, base + 12U, point.reflectivity);
      writeValue(cloud.data, base + 13U, point.tag);
      writeValue(cloud.data, base + 14U, point.line);
      writeValue(cloud.data, base + 16U, point.offset_time);
    }

    publisher_->publish(std::move(cloud));
    ++lidar_published_;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string imu_input_topic_;
  std::string imu_output_topic_;
  double imu_acceleration_scale_{9.80665};
  std::string frame_id_;
  std::string imu_frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  std::size_t lidar_received_{0U};
  std::size_t lidar_published_{0U};
  std::size_t imu_received_{0U};
  std::size_t imu_published_{0U};
};

}  // namespace interfaces

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<interfaces::LivoxToPointCloud2Node>());
  rclcpp::shutdown();
  return 0;
}
