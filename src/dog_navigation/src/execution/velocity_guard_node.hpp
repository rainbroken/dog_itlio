#pragma once
#include "dog_navigation/localization_gate.hpp"
#include "dog_navigation/safety_arbiter.hpp"
#include "dog_terrain/terrain_graph.hpp"
#include <dog_interfaces/msg/navigation_command.hpp>
#include <dog_interfaces/msg/robot_status.hpp>
#include <dog_interfaces/msg/terrain_route.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace dog_navigation {
class VelocityGuard : public rclcpp::Node {
public:
  VelocityGuard();

private:
  void command(dog_interfaces::msg::NavigationCommand::ConstSharedPtr msg);
  void tick();
  bool collisionFree(const Velocity &command);
  bool fresh(const builtin_interfaces::msg::Time &stamp, double maximum) const;
  SafetyArbiter arbiter_;
  SafetyInput input_;
  double status_at_ = 0, feedback_at_ = 0, route_deadline_ = 0, offset_ = .40;
  bool auto_enabled_ = false;
  bool collision_check_expired_=false;
  double braking_deceleration_ = .15;
  double yaw_braking_deceleration_ = .5;
  double executor_limits_at_ = 0;
  std::string executor_limits_task_;
  uint64_t sequence_ = 0;
  uint64_t handoff_generation_=0;
  double handoff_deadline_=0;
  std::string project_, revision_;
  Velocity measured_;
  dog_interfaces::msg::RobotStatus robot_;
  dog_interfaces::msg::NavigationCommand last_command_;
  dog_interfaces::msg::TerrainRoute::ConstSharedPtr route_;
  std::shared_ptr<dog_terrain::TerrainGraph> graph_;
  std::unique_ptr<LocalizationGate> localization_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_;
  rclcpp::Subscription<dog_interfaces::msg::NavigationCommand>::SharedPtr
      command_;
  rclcpp::Subscription<dog_interfaces::msg::TerrainRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<dog_interfaces::msg::RobotStatus>::SharedPtr robot_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr feedback_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr executor_status_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_;
  rclcpp::TimerBase::SharedPtr timer_;
};
inline double guardWall() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
} // namespace dog_navigation
