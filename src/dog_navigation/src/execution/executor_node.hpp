#pragma once
#include "dog_navigation/localization_gate.hpp"
#include "dog_navigation/path_tracker.hpp"
#include "dog_navigation/task_supervisor.hpp"
#include "dog_terrain/terrain_graph.hpp"
#include <dog_interfaces/action/navigate_terrain.hpp>
#include <dog_interfaces/msg/navigation_command.hpp>
#include <dog_interfaces/msg/robot_status.hpp>
#include <dog_interfaces/msg/terrain_route.hpp>
#include <dog_interfaces/msg/timed_terrain_trajectory.hpp>
#include <dog_interfaces/srv/set_motion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace dog_navigation {
class NavigationExecutor : public rclcpp::Node {
  using Action = dog_interfaces::action::NavigateTerrain;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;

public:
  NavigationExecutor();

private:
  void configure();
  void tick();
  void receiveRoute(dog_interfaces::msg::TerrainRoute::ConstSharedPtr route);
  void submitPlannerGoal();
  bool planner_goal_deferred_=false;
  std::optional<rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId> planner_cleanup_;
  void finish(bool success, const std::string &reason, bool cancelled = false);
  void publishCommand(const TrackResult &result, bool permit);
  void publishState(const TrackResult &result);
  TrackingPose readPose(Pose &map_from_odom);
  bool enabled_ = false, use_yaw_ = false;
  double controller_frequency_ = 20.;
  double retry_interval_=4.;
  double ground_offset_ = .40, route_deadline_ = 0, feedback_at_ = 0,
         robot_at_ = 0, guard_at_ = 0;
  double route_stamp_ = 0, last_tick_ = 0, last_request_ = -100,
         last_progress_ = 0;
  uint64_t sequence_ = 0, accepted_generation_ = 0;
  int64_t cancelled_before_ns_=0;
  std::string task_id_, revision_, project_, guard_reason_, guard_task_id_;
  std::string last_block_reason_,last_guard_block_reason_,planner_block_reason_;
  geometry_msgs::msg::PoseStamped goal_;
  dog_interfaces::msg::TerrainRoute::ConstSharedPtr route_;
  std::vector<int32_t> installed_ids_;
  bool guard_auto_enabled_ = false;
  std::shared_ptr<dog_terrain::TerrainGraph> graph_;
  std::shared_ptr<Handle> handle_;
  dog_interfaces::msg::RobotStatus robot_;
  Velocity measured_;
  Point progress_position_ = Point::Zero();
  double progress_yaw_ = 0;
  ExecutionOptions options_;
  std::unique_ptr<PathTracker> tracker_;
  TaskSupervisor supervisor_;
  std::unique_ptr<LocalizationGate> localization_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::optional<Pose> correction_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<dog_interfaces::msg::NavigationCommand>::SharedPtr
      command_pub_;
  rclcpp::Publisher<dog_interfaces::msg::TimedTerrainTrajectory>::SharedPtr
      trajectory_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<dog_interfaces::msg::TerrainRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<dog_interfaces::msg::RobotStatus>::SharedPtr robot_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr loc_sub_, guard_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_, resume_, cancel_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr planner_cancel_,
      relocalize_;
  rclcpp::Client<dog_interfaces::srv::SetMotion>::SharedPtr walk_;
  rclcpp::TimerBase::SharedPtr timer_;
};
double monotonicSeconds();
} // namespace dog_navigation
