#ifndef DOG_DRIVER__DOG_DRIVER_NODE_HPP_
#define DOG_DRIVER__DOG_DRIVER_NODE_HPP_

#include "dog_driver/bpx_read_only_client.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "dog_interfaces/msg/robot_status.hpp"
#include "dog_interfaces/srv/query_version.hpp"
#include "dog_interfaces/srv/set_motion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <chrono>
#include <memory>
#include <string>

namespace dog_driver
{

class DogDriverNode : public rclcpp::Node
{
public:
  explicit DogDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DogDriverNode() override;

private:
  ReadOnlyClientConfig readParameters();
  void publishState();
  void publishStatusTimer();
  void publishDiagnostics();
  void publishJointState(const RobotStateSnapshot & state, const rclcpp::Time & stamp);
  void publishImu(const RobotStateSnapshot & state, const rclcpp::Time & stamp);
  void publishOdometry(const RobotStateSnapshot & state, const rclcpp::Time & stamp);
  void publishBattery(const RobotStateSnapshot & state, const rclcpp::Time & stamp);
  void publishRobotStatus(const RobotStateSnapshot & state, const rclcpp::Time & stamp);
  void handleVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr message);
  void enforceVelocityDeadman();

  std::unique_ptr<BpxReadOnlyClient> client_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
  rclcpp::Publisher<dog_interfaces::msg::RobotStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
  rclcpp::Service<dog_interfaces::srv::QueryVersion>::SharedPtr query_version_service_;
  rclcpp::Service<dog_interfaces::srv::SetMotion>::SharedPtr motion_service_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr velocity_deadman_timer_;

  std::string odom_frame_;
  std::string odom_topic_;
  std::string robot_ip_;
  std::string base_footprint_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  bool publish_odom_tf_{true};
  double odom_position_stddev_{0.05};
  double odom_orientation_stddev_{0.05};
  double odom_linear_velocity_stddev_{0.03};
  double odom_angular_velocity_stddev_{0.05};
  bool posture_control_enabled_{true};
  bool velocity_control_enabled_{true};
  double safe_velocity_x_{1.00};
  double safe_velocity_y_{0.25};
  double safe_velocity_yaw_{1.50};
  std::chrono::milliseconds velocity_timeout_{250};
  std::chrono::steady_clock::time_point last_velocity_command_at_{};
  bool velocity_command_active_{false};
  bool status_transition_initialized_{false};
  bool last_status_connected_{false};
  bool last_status_fresh_{false};
  uint8_t last_status_motion_state_{0};
  uint8_t last_status_gait_{0};
  std::string last_status_fault_reason_;
};

}  // namespace dog_driver

#endif  // DOG_DRIVER__DOG_DRIVER_NODE_HPP_
