#include "dog_slam/pose_conversion.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace dog_slam {
class LioTfAdapter : public rclcpp::Node {
public:
  LioTfAdapter() : Node("lio_tf_adapter")
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "livox_imu");
    const auto topic = declare_parameter<std::string>("odometry_topic", "/lio/odom");
    // Required: never silently assume that the robot body coincides with the IMU.
    const auto translation = declare_parameter<std::vector<double>>(
      "base_to_imu_translation", std::vector<double>{});
    const auto rotation = declare_parameter<std::vector<double>>(
      "base_to_imu_rotation", std::vector<double>{});
    if (translation.size() != 3 || rotation.size() != 9 || topic.empty() ||
        odom_frame_.empty() || base_frame_.empty() || imu_frame_.empty() ||
        odom_frame_ == base_frame_ || odom_frame_ == imu_frame_ || base_frame_ == imu_frame_) {
      throw std::invalid_argument("valid distinct frames, topic and URDF mounting transform required");
    }
    for (int i = 0; i < 3; ++i) {
      base_to_imu_.translation()[i] = translation[i];
      for (int j = 0; j < 3; ++j) {base_to_imu_.linear()(i, j) = rotation[3 * i + j];}
    }
    const auto & r = base_to_imu_.linear();
    if (!base_to_imu_.matrix().allFinite() ||
        !(r.transpose() * r).isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
        std::abs(r.determinant() - 1.0) > 1e-6) {
      throw std::invalid_argument("mounting transform must contain a finite SO(3) rotation");
    }
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      topic, rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {publish(*msg);});
    RCLCPP_INFO(get_logger(), "LIO body TF: %s -> %s, input %s (%s)",
      odom_frame_.c_str(), base_frame_.c_str(), topic.c_str(), imu_frame_.c_str());
  }

private:
  void publish(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
    if (msg.header.frame_id != odom_frame_ || msg.child_frame_id != imu_frame_ ||
        !Eigen::Vector3d(p.x, p.y, p.z).allFinite() || !orientation.coeffs().allFinite() ||
        std::abs(orientation.norm() - 1.0) > 1e-3 || msg.header.stamp.sec < 0 ||
        msg.header.stamp.nanosec >= 1000000000U ||
        (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Reject LIO odometry: invalid frames, pose or timestamp");
      return;
    }
    Eigen::Isometry3d odom_to_imu = Eigen::Isometry3d::Identity();
    odom_to_imu.linear() = orientation.normalized().toRotationMatrix();
    odom_to_imu.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    const auto odom_to_base = basePose(odom_to_imu, base_to_imu_);
    const Eigen::Quaterniond body_rotation(odom_to_base.linear());
    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg.header;  // Preserve scan-end time; never stamp with wall time.
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = odom_to_base.translation().x();
    transform.transform.translation.y = odom_to_base.translation().y();
    transform.transform.translation.z = odom_to_base.translation().z();
    transform.transform.rotation.x = body_rotation.x();
    transform.transform.rotation.y = body_rotation.y();
    transform.transform.rotation.z = body_rotation.z();
    transform.transform.rotation.w = body_rotation.w();
    broadcaster_->sendTransform(transform);
  }

  std::string odom_frame_, base_frame_, imu_frame_;
  Eigen::Isometry3d base_to_imu_{Eigen::Isometry3d::Identity()};
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};
}  // namespace dog_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dog_slam::LioTfAdapter>());
  rclcpp::shutdown();
  return 0;
}
