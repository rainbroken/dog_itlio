#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <mutex>

// Boundary adapter only: forwards initial poses/search requests and reports the
// actual localizer state. It never estimates a pose or commands robot motion.
class LocalizationBridge {
public:
  LocalizationBridge(rclcpp::Node &node,tf2_ros::Buffer &tf);
  nlohmann::json snapshot() const;
  nlohmann::json command(const std::string &path,const nlohmann::json &body);
private:
  rclcpp::Node &node_;
  tf2_ros::Buffer &tf_;
  mutable std::mutex mutex_;
  nlohmann::json state_=nlohmann::json::object();
  std::chrono::steady_clock::time_point receipt_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr seed_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr local_,global_;
};
