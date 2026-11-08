#pragma once
#include "dog_slam/map_localization.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>

namespace dog_slam {
using Steady = std::chrono::steady_clock;
// ROS process coordinator. Registration/state transitions remain in the pure
// LocalizationEngine; input, worker execution, and publication are separate units.
class MapLocalizer : public rclcpp::Node {
public:
  MapLocalizer();
  ~MapLocalizer() override;
private:
  enum class Kind {Load, Seed, Relocalize, GlobalRelocalize, Reset, Invalidate};
  struct Command {Kind kind=Kind::Reset; std::string path; Pose3 seed=Pose3::Identity(); builtin_interfaces::msg::Time stamp;};

  void configureParameters();
  void enqueue(Command c, const std::string &label);
  void receive(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  Pose3 lookup(const std::string &parent, const std::string &child, const builtin_interfaces::msg::Time &stamp);
  void run();
  bool freshLocked() const;
  void publishTransform();
  void publishStatus();
  LocalizationOptions options_;
  std::string map_frame_,odom_frame_,base_frame_,cloud_frame_,cloud_topic_,loaded_path_;
  double rate_,timeout_,max_scan_age_,max_result_age_,recovery_interval_,odom_jump_,odom_speed_,odom_rotation_;
  double global_retry_interval_=3;
  std::mutex mutex_; std::condition_variable condition_; std::thread worker_; bool stopping_=false,map_ready_=false;
  uint64_t generation_=0; std::deque<Command> commands_; sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_;
  double last_received_stamp_=0; Steady::time_point input_receipt_,valid_receipt_;
  LocalizationUpdate public_; std::string label_="no_map",reason_="not_initialized";
  std::unique_ptr<tf2_ros::Buffer> buffer_; std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr seed_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_,relocalize_srv_,reset_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr global_srv_;
  rclcpp::TimerBase::SharedPtr tf_timer_,status_timer_;
};
}  // namespace dog_slam
