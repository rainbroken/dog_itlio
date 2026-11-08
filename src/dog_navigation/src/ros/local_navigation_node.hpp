#pragma once
#include "dog_navigation/local_planner.hpp"
#include <dog_interfaces/msg/terrain_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <type_traits>

#include "dog_navigation/localization_gate.hpp"

namespace dog_navigation {
using Clock=std::chrono::steady_clock;
class LocalNavigation:public rclcpp::Node {
public:
  LocalNavigation();
private:
  void configureParameters();
  void receiveGoal(geometry_msgs::msg::PoseStamped::ConstSharedPtr m);
  void receiveLocalization(std_msgs::msg::String::ConstSharedPtr m);
  void receiveCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr m);
  Pose lookup(const std::string &parent,const std::string &child,const builtin_interfaces::msg::Time &stamp);
  nav_msgs::msg::Path path(const Points &points,const builtin_interfaces::msg::Time &stamp);
  void status(const std::string &reason,bool valid,const Route &route={});
  void invalidate(const std::string &reason,const Route &route={});
  void tick();
  Options options_;double timeout_=.6,localization_timeout_=.8,tf_timeout_=.4,budget_=.4,rate_=5.,ground_offset_=.40,jump_=.3,speed_=3.;
  double duration_ms_=0,last_received_=0,last_scan_stamp_=0,tf_expiry_=0;
  uint64_t generation_=0;bool scan_valid_=false;
  bool route_generation_valid_=false;
  std::vector<int> generation_ids_;
  std::vector<bool> generation_stairs_;
  builtin_interfaces::msg::Time generation_goal_stamp_;
  std::unique_ptr<LocalizationGate> gate_;
  builtin_interfaces::msg::Time route_stamp_;
  builtin_interfaces::msg::Time goal_stamp_;
  std::filesystem::path project_;std::string map_frame_,odom_frame_,base_frame_,sensor_frame_;
  std::shared_ptr<dog_terrain::TerrainGraph> graph_;std::unique_ptr<LocalPlanner> planner_;
  std::unique_ptr<ObstacleMemory> memory_;std::optional<Point> goal_;std::optional<Pose> last_body_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_;std_msgs::msg::Header scan_header_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_,local_pub_;
  rclcpp::Publisher<dog_interfaces::msg::TerrainRoute>::SharedPtr route_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_;rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace dog_navigation
