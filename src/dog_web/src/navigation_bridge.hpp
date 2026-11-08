#pragma once
#include <dog_interfaces/action/navigate_terrain.hpp>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

// HTTP/action boundary. Planning, control and safety remain in dog_navigation.
class NavigationBridge {
public:
  explicit NavigationBridge(rclcpp::Node &node);
  nlohmann::json snapshot() const;
  nlohmann::json command(const std::string &path, const nlohmann::json &body);

private:
  using Action = dog_interfaces::action::NavigateTerrain;
  rclcpp::Node &node_;
  mutable std::mutex mutex_;
  nlohmann::json state_ = nlohmann::json::object();
  std::chrono::steady_clock::time_point received_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_;
  rclcpp_action::Client<Action>::SharedPtr action_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_, resume_, cancel_;
};
