#include "dog_driver/dog_driver_node.hpp"

#include "bpx_sdk_version.h"

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

namespace dog_driver
{

namespace
{

std::string formatRobotVersion(const RobotStateSnapshot & state)
{
  if (!state.robot_version_supported) {
    return "unsupported";
  }

  std::ostringstream stream;
  stream << state.robot_version_major << "."
         << state.robot_version_minor << "."
         << state.robot_version_patch;
  return stream.str();
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

bool isKnownMotion(uint8_t motion)
{
  return motion <= dog_interfaces::srv::SetMotion::Request::BOUND;
}

}  // namespace

DogDriverNode::DogDriverNode(const rclcpp::NodeOptions & options)
: Node("dog_driver", options)
{
  const auto client_config = readParameters();
  const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);
  const double status_rate_hz = declare_parameter<double>("status_rate_hz", 10.0);
  const double diagnostic_rate_hz = declare_parameter<double>("diagnostic_rate_hz", 1.0);
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/dog/odom");
  base_footprint_frame_ = declare_parameter<std::string>(
    "base_footprint_frame", "base_footprint");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
  publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", true);
  odom_position_stddev_ = declare_parameter<double>("odom_position_stddev", 0.05);
  odom_orientation_stddev_ = declare_parameter<double>("odom_orientation_stddev", 0.05);
  odom_linear_velocity_stddev_ = declare_parameter<double>("odom_linear_velocity_stddev", 0.03);
  odom_angular_velocity_stddev_ = declare_parameter<double>("odom_angular_velocity_stddev", 0.05);

  joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SensorDataQoS());
  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SensorDataQoS());
  odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS());
  battery_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
    "/battery_state", rclcpp::QoS(10));
  status_publisher_ = create_publisher<dog_interfaces::msg::RobotStatus>(
    "/dog/status", rclcpp::QoS(1).reliable());
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  velocity_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    declare_parameter<std::string>("velocity_topic", "/dog/safe_cmd_vel"), rclcpp::QoS(1).reliable(),
    std::bind(&DogDriverNode::handleVelocityCommand, this, std::placeholders::_1));
  query_version_service_ = create_service<dog_interfaces::srv::QueryVersion>(
    "/dog/query_version",
    [this](
      const std::shared_ptr<dog_interfaces::srv::QueryVersion::Request>,
      std::shared_ptr<dog_interfaces::srv::QueryVersion::Response> response)
    {
      const auto state = client_->snapshot();
      response->success = state.connected;
      response->sdk_version = BPX_SDK_PROJECT_VERSION;
      response->robot_version_supported = state.robot_version_supported;
      response->robot_major = state.robot_version_major;
      response->robot_minor = state.robot_version_minor;
      response->robot_patch = state.robot_version_patch;
      response->robot_commit = state.robot_version_commit;
      response->robot_build_date = state.robot_build_date;
      response->robot_build_time = state.robot_build_time;
      response->message = state.connected ? "version data is cached from SDK connection" :
      "SDK client is not connected";
    });

  motion_service_ = create_service<dog_interfaces::srv::SetMotion>(
    "/dog/set_motion",
    [this](const std::shared_ptr<dog_interfaces::srv::SetMotion::Request> request,
    std::shared_ptr<dog_interfaces::srv::SetMotion::Response> response)
    {
      response->accepted = false;
      if (!posture_control_enabled_) {
        response->message = "posture control is disabled by configuration";
        return;
      }
      if (!request->confirm_restricted_motion) {
        response->message = "restricted motion requires explicit confirmation";
        return;
      }
      if (!isKnownMotion(request->motion)) {
        response->message = "unknown motion command";
        return;
      }
      client_->stopVelocity();
      velocity_command_active_ = false;
      response->accepted = client_->sendMotionCommand(request->motion, &response->message);
      const auto state = client_->snapshot();
      RCLCPP_INFO(
        get_logger(),
        "Motion request=%u sdk_sent=%s result='%s' connected=%s fresh=%s "
        "motion_available=%s state=%u gait=%u motion_timestamp_ms=%u",
        static_cast<unsigned>(request->motion), response->accepted ? "true" : "false",
        response->message.c_str(), state.connected ? "true" : "false",
        state.state_fresh ? "true" : "false", state.has_motion_state ? "true" : "false",
        static_cast<unsigned>(state.current_motion_state),
        static_cast<unsigned>(state.current_gait), state.motion_timestamp_ms);
    });

  client_ = std::make_unique<BpxReadOnlyClient>(client_config);
  client_->start();

  state_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(publish_rate_hz, 1.0)),
    std::bind(&DogDriverNode::publishState, this));
  status_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(status_rate_hz, 1.0)),
    std::bind(&DogDriverNode::publishStatusTimer, this));
  diagnostics_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(diagnostic_rate_hz, 0.1)),
    std::bind(&DogDriverNode::publishDiagnostics, this));
  velocity_deadman_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&DogDriverNode::enforceVelocityDeadman, this));

  RCLCPP_INFO(
    get_logger(),
    "BPX driver started for %s; restricted motion service and deadman-limited Walk teleoperation are enabled",
    client_config.robot_ip.c_str());
}

DogDriverNode::~DogDriverNode()
{
  if (client_) {
    client_->stop();
  }
}

ReadOnlyClientConfig DogDriverNode::readParameters()
{
  ReadOnlyClientConfig config;
  config.robot_ip = declare_parameter<std::string>("robot_ip", config.robot_ip);
  robot_ip_ = config.robot_ip;
  config.robot_state_port = static_cast<uint16_t>(
    declare_parameter<int>("robot_state_port", config.robot_state_port));
  config.tcp_local_port = static_cast<uint16_t>(
    declare_parameter<int>("tcp_local_port", config.tcp_local_port));
  config.session_id = static_cast<uint16_t>(
    declare_parameter<int>("session_id", config.session_id));
  config.state_upload_rate_hz = static_cast<uint16_t>(
    declare_parameter<int>("state_upload_rate_hz", config.state_upload_rate_hz));
  config.motion_command_rate_hz = static_cast<uint16_t>(
    declare_parameter<int>("motion_command_rate_hz", config.motion_command_rate_hz));
  posture_control_enabled_ = declare_parameter<bool>("posture_control_enabled", true);
  velocity_control_enabled_ = declare_parameter<bool>("velocity_control_enabled", true);
  safe_velocity_x_ = declare_parameter<double>("safe_velocity_x", config.safe_velocity_x);
  safe_velocity_y_ = declare_parameter<double>("safe_velocity_y", config.safe_velocity_y);
  safe_velocity_yaw_ = declare_parameter<double>("safe_velocity_yaw", config.safe_velocity_yaw);
  velocity_timeout_ = std::chrono::milliseconds(
    declare_parameter<int>("velocity_timeout_ms", velocity_timeout_.count()));
  config.safe_velocity_x = static_cast<float>(safe_velocity_x_);
  config.safe_velocity_y = static_cast<float>(safe_velocity_y_);
  config.safe_velocity_yaw = static_cast<float>(safe_velocity_yaw_);
  config.poll_period = std::chrono::milliseconds(
    declare_parameter<int>("poll_period_ms", config.poll_period.count()));
  config.stale_timeout = std::chrono::milliseconds(
    declare_parameter<int>("state_timeout_ms", config.stale_timeout.count()));
  config.reconnect_period = std::chrono::milliseconds(
    declare_parameter<int>("reconnect_period_ms", config.reconnect_period.count()));
  return config;
}

void DogDriverNode::publishState()
{
  const auto state = client_->snapshot();
  const auto stamp = now();

  if (state.has_joint_state) {
    publishJointState(state, stamp);
  }
  if (state.has_imu) {
    publishImu(state, stamp);
  }
  if (state.has_odometry) {
    publishOdometry(state, stamp);
  }
  if (state.has_battery) {
    publishBattery(state, stamp);
  }
  const bool transition = !status_transition_initialized_ ||
    state.connected != last_status_connected_ || state.state_fresh != last_status_fresh_ ||
    state.current_motion_state != last_status_motion_state_ ||
    state.current_gait != last_status_gait_ || state.fault_reason != last_status_fault_reason_;
  if (transition) {
    publishRobotStatus(state, stamp);
  }
  status_transition_initialized_ = true;
  last_status_connected_ = state.connected;
  last_status_fresh_ = state.state_fresh;
  last_status_motion_state_ = state.current_motion_state;
  last_status_gait_ = state.current_gait;
  last_status_fault_reason_ = state.fault_reason;
}

void DogDriverNode::publishStatusTimer()
{
  publishRobotStatus(client_->snapshot(), now());
}

void DogDriverNode::publishDiagnostics()
{
  const auto state = client_->snapshot();
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus connection;
  connection.name = "dog_driver/sdk_connection";
  connection.hardware_id = "bpx@" + robot_ip_;
  if (!state.connected) {
    connection.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    connection.message = state.fault_reason.empty() ? "disconnected" : state.fault_reason;
  } else if (!state.state_fresh) {
    connection.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    connection.message = "connected but robot state is stale";
  } else {
    connection.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    connection.message = "read-only state stream is healthy";
  }
  connection.values.push_back(keyValue("control_level", "safe_teleop"));
  connection.values.push_back(keyValue("safe_velocity_x", std::to_string(safe_velocity_x_)));
  connection.values.push_back(keyValue("safe_velocity_y", std::to_string(safe_velocity_y_)));
  connection.values.push_back(keyValue("safe_velocity_yaw", std::to_string(safe_velocity_yaw_)));
  connection.values.push_back(keyValue("sdk_version", BPX_SDK_PROJECT_VERSION));
  connection.values.push_back(keyValue("robot_version", formatRobotVersion(state)));
  connection.values.push_back(
    keyValue(
      "joint_timestamp_ms",
      std::to_string(state.joint_timestamp_ms)));
  connection.values.push_back(keyValue("imu_timestamp_ms", std::to_string(state.imu_timestamp_ms)));
  array.status.push_back(connection);

  if (state.has_temperature) {
    diagnostic_msgs::msg::DiagnosticStatus temperature;
    temperature.name = "dog_driver/temperature";
    temperature.hardware_id = "bpx";
    const float max_motor = *std::max_element(
      state.motor_temperature.begin(), state.motor_temperature.end());
    const float max_driver = *std::max_element(
      state.driver_temperature.begin(), state.driver_temperature.end());
    temperature.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    temperature.message = "temperature telemetry available";
    temperature.values.push_back(keyValue("max_motor_c", std::to_string(max_motor)));
    temperature.values.push_back(keyValue("max_driver_c", std::to_string(max_driver)));
    array.status.push_back(temperature);
  }

  diagnostics_publisher_->publish(array);
}

void DogDriverNode::publishJointState(
  const RobotStateSnapshot & state, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::JointState message;
  message.header.stamp = stamp;
  message.name.assign(bpx_sdk::kJointNames.begin(), bpx_sdk::kJointNames.end());
  message.position.assign(state.joint_position.begin(), state.joint_position.end());
  message.velocity.assign(state.joint_velocity.begin(), state.joint_velocity.end());
  message.effort.assign(state.joint_torque.begin(), state.joint_torque.end());
  joint_state_publisher_->publish(message);
}

void DogDriverNode::publishImu(
  const RobotStateSnapshot & state, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Imu message;
  message.header.stamp = stamp;
  message.header.frame_id = imu_frame_;
  message.orientation.x = state.imu_quaternion[0];
  message.orientation.y = state.imu_quaternion[1];
  message.orientation.z = state.imu_quaternion[2];
  message.orientation.w = state.imu_quaternion[3];
  message.angular_velocity.x = state.imu_angular_velocity[0];
  message.angular_velocity.y = state.imu_angular_velocity[1];
  message.angular_velocity.z = state.imu_angular_velocity[2];
  message.linear_acceleration.x = state.imu_acceleration[0];
  message.linear_acceleration.y = state.imu_acceleration[1];
  message.linear_acceleration.z = state.imu_acceleration[2];
  message.orientation_covariance[0] = -1.0;
  message.angular_velocity_covariance[0] = -1.0;
  message.linear_acceleration_covariance[0] = -1.0;
  imu_publisher_->publish(message);
}

void DogDriverNode::publishOdometry(
  const RobotStateSnapshot & state, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry message;
  message.header.stamp = stamp;
  message.header.frame_id = odom_frame_;
  message.child_frame_id = base_frame_;
  message.pose.pose.position.x = state.leg_odometry.position[0];
  message.pose.pose.position.y = state.leg_odometry.position[1];
  message.pose.pose.position.z = state.leg_odometry.position[2];
  message.pose.pose.orientation.x = state.leg_odometry.orientation[0];
  message.pose.pose.orientation.y = state.leg_odometry.orientation[1];
  message.pose.pose.orientation.z = state.leg_odometry.orientation[2];
  message.pose.pose.orientation.w = state.leg_odometry.orientation[3];
  message.twist.twist.linear.x = state.leg_odometry.velocity_body[0];
  message.twist.twist.linear.y = state.leg_odometry.velocity_body[1];
  message.twist.twist.linear.z = state.leg_odometry.velocity_body[2];
  message.twist.twist.angular.x = state.leg_odometry.angular_velocity[0];
  message.twist.twist.angular.y = state.leg_odometry.angular_velocity[1];
  message.twist.twist.angular.z = state.leg_odometry.angular_velocity[2];
  for (int axis = 0; axis < 3; ++axis) {
    message.pose.covariance[axis * 6 + axis] =
      odom_position_stddev_ * odom_position_stddev_;
    message.pose.covariance[(axis + 3) * 6 + axis + 3] =
      odom_orientation_stddev_ * odom_orientation_stddev_;
    message.twist.covariance[axis * 6 + axis] =
      odom_linear_velocity_stddev_ * odom_linear_velocity_stddev_;
    message.twist.covariance[(axis + 3) * 6 + axis + 3] =
      odom_angular_velocity_stddev_ * odom_angular_velocity_stddev_;
  }
  odometry_publisher_->publish(message);

  if (publish_odom_tf_) {
    const auto & orientation = message.pose.pose.orientation;
    const double yaw = std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      1.0 - 2.0 *
      (orientation.y * orientation.y + orientation.z * orientation.z));
    const double half_yaw = 0.5 * yaw;
    const double yaw_sin = std::sin(half_yaw);
    const double yaw_cos = std::cos(half_yaw);

    geometry_msgs::msg::TransformStamped odom_to_footprint;
    odom_to_footprint.header = message.header;
    odom_to_footprint.child_frame_id = base_footprint_frame_;
    odom_to_footprint.transform.translation.x = message.pose.pose.position.x;
    odom_to_footprint.transform.translation.y = message.pose.pose.position.y;
    odom_to_footprint.transform.translation.z = 0.0;
    odom_to_footprint.transform.rotation.z = yaw_sin;
    odom_to_footprint.transform.rotation.w = yaw_cos;

    geometry_msgs::msg::TransformStamped footprint_to_base;
    footprint_to_base.header.stamp = stamp;
    footprint_to_base.header.frame_id = base_footprint_frame_;
    footprint_to_base.child_frame_id = base_frame_;
    footprint_to_base.transform.translation.z = message.pose.pose.position.z;
    // q_relative = inverse(q_yaw) * q_full. Composing both transforms
    // reconstructs the original full 6-DoF odom -> base_link pose exactly.
    footprint_to_base.transform.rotation.x =
      yaw_cos * orientation.x + yaw_sin * orientation.y;
    footprint_to_base.transform.rotation.y =
      yaw_cos * orientation.y - yaw_sin * orientation.x;
    footprint_to_base.transform.rotation.z =
      yaw_cos * orientation.z - yaw_sin * orientation.w;
    footprint_to_base.transform.rotation.w =
      yaw_cos * orientation.w + yaw_sin * orientation.z;

    tf_broadcaster_->sendTransform(odom_to_footprint);
    tf_broadcaster_->sendTransform(footprint_to_base);
  }
}

void DogDriverNode::publishBattery(
  const RobotStateSnapshot & state, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::BatteryState message;
  message.header.stamp = stamp;
  message.voltage = std::numeric_limits<float>::quiet_NaN();
  message.current = state.battery_current;
  message.percentage = static_cast<float>(state.battery_level) / 100.0F;
  message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  message.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  message.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  message.present = true;
  battery_publisher_->publish(message);
}

void DogDriverNode::publishRobotStatus(
  const RobotStateSnapshot & state, const rclcpp::Time & stamp)
{
  dog_interfaces::msg::RobotStatus message;
  message.header.stamp = stamp;
  message.connected = state.connected;
  message.state_fresh = state.state_fresh;
  message.control_level = dog_interfaces::msg::RobotStatus::CONTROL_SAFE_TELEOP;
  message.safety_state = state.connected && state.state_fresh ?
    dog_interfaces::msg::RobotStatus::SAFETY_STANDBY :
    dog_interfaces::msg::RobotStatus::SAFETY_FAULT;
  message.current_motion_state = state.current_motion_state;
  message.current_gait = state.current_gait;
  message.last_motion_state = state.last_motion_state;
  message.last_gait = state.last_gait;
  message.sub_gait = state.sub_gait;
  message.max_velocity = state.max_velocity;
  message.joint_timestamp_ms = state.joint_timestamp_ms;
  message.imu_timestamp_ms = state.imu_timestamp_ms;
  message.odometry_timestamp_ms = state.odometry_timestamp_ms;
  message.motion_timestamp_ms = state.motion_timestamp_ms;
  message.battery_timestamp_ms = state.battery_timestamp_ms;
  message.sdk_version = BPX_SDK_PROJECT_VERSION;
  message.robot_version = formatRobotVersion(state);
  message.fault_reason = state.fault_reason;
  status_publisher_->publish(message);
}

void DogDriverNode::handleVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr message)
{
  if (!velocity_control_enabled_) {
    client_->stopVelocity();
    velocity_command_active_ = false;
    return;
  }
  if (!std::isfinite(message->linear.x) || !std::isfinite(message->linear.y) ||
    !std::isfinite(message->angular.z))
  {
    client_->stopVelocity();
    velocity_command_active_ = false;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Rejected non-finite /cmd_vel command");
    return;
  }

  const float x =
    static_cast<float>(std::clamp(message->linear.x, -safe_velocity_x_, safe_velocity_x_));
  const float y =
    static_cast<float>(std::clamp(message->linear.y, -safe_velocity_y_, safe_velocity_y_));
  const float yaw = static_cast<float>(
    std::clamp(message->angular.z, -safe_velocity_yaw_, safe_velocity_yaw_));
  const bool is_zero = std::abs(x) < 1.0e-4F && std::abs(y) < 1.0e-4F &&
    std::abs(yaw) < 1.0e-4F;
  std::string result;
  const bool accepted = is_zero ? client_->stopVelocity(&result) :
    client_->sendVelocityCommand(x, y, yaw, &result);
  if (!accepted) {
    client_->stopVelocity();
    velocity_command_active_ = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", result.c_str());
    return;
  }
  last_velocity_command_at_ = std::chrono::steady_clock::now();
  velocity_command_active_ = !is_zero;
}

void DogDriverNode::enforceVelocityDeadman()
{
  if (!velocity_command_active_) {
    return;
  }
  const auto state = client_->snapshot();
  const bool safety_gate_open = state.connected && state.state_fresh && state.has_motion_state &&
    state.current_motion_state == static_cast<uint8_t>(bpx_sdk::MotionState::Motion) &&
    state.current_gait == static_cast<uint8_t>(bpx_sdk::MotionGait::Walk);
  const bool timed_out = std::chrono::steady_clock::now() - last_velocity_command_at_ >
    velocity_timeout_;
  if (!safety_gate_open || timed_out) {
    client_->stopVelocity();
    velocity_command_active_ = false;
    if (timed_out) {
      const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_velocity_command_at_).count();
      RCLCPP_WARN(
        get_logger(),
        "Velocity deadman stopped the robot: /cmd_vel age %ld ms exceeded %ld ms",
        age_ms, velocity_timeout_.count());
    } else if (!state.connected) {
      RCLCPP_WARN(get_logger(), "Velocity deadman stopped the robot: SDK disconnected");
    } else if (!state.state_fresh) {
      RCLCPP_WARN(get_logger(), "Velocity deadman stopped the robot: robot state is stale");
    } else if (!state.has_motion_state) {
      RCLCPP_WARN(get_logger(), "Velocity deadman stopped the robot: motion state unavailable");
    } else if (state.current_motion_state != static_cast<uint8_t>(bpx_sdk::MotionState::Motion)) {
      RCLCPP_WARN(
        get_logger(), "Velocity deadman stopped the robot: motion state changed to %u",
        state.current_motion_state);
    } else {
      RCLCPP_WARN(
        get_logger(), "Velocity deadman stopped the robot: gait changed to %u",
        state.current_gait);
    }
  }
}

}  // namespace dog_driver
