#ifndef DOG_DRIVER__BPX_READ_ONLY_CLIENT_HPP_
#define DOG_DRIVER__BPX_READ_ONLY_CLIENT_HPP_

#include "motion_types.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace bpx_sdk
{
class RequestRobotState;
class MotionLevelControl;
}

namespace dog_driver
{

struct ReadOnlyClientConfig
{
  std::string robot_ip{"10.21.20.1"};
  uint16_t robot_state_port{9873};
  uint16_t tcp_local_port{0};
  uint16_t session_id{0};
  uint16_t state_upload_rate_hz{100};
  uint16_t motion_command_rate_hz{50};
  float safe_velocity_x{1.00F};
  float safe_velocity_y{0.25F};
  float safe_velocity_yaw{1.50F};
  std::chrono::milliseconds poll_period{10};
  std::chrono::milliseconds stale_timeout{500};
  std::chrono::milliseconds reconnect_period{1000};
};

struct RobotStateSnapshot
{
  bool connected{false};
  bool state_fresh{false};
  bool has_joint_state{false};
  bool has_imu{false};
  bool has_odometry{false};
  bool has_battery{false};
  bool has_temperature{false};
  bool has_motion_state{false};
  bool robot_version_supported{false};

  std::array<float, 12> joint_position{};
  std::array<float, 12> joint_velocity{};
  std::array<float, 12> joint_torque{};
  std::array<float, 12> motor_temperature{};
  std::array<float, 12> driver_temperature{};
  std::array<float, 4> imu_quaternion{0.0F, 0.0F, 0.0F, 1.0F};
  std::array<float, 3> imu_acceleration{};
  std::array<float, 3> imu_angular_velocity{};
  bpx_sdk::LegOdom leg_odometry{};

  uint8_t battery_level{0};
  float battery_current{0.0F};
  uint8_t current_motion_state{0};
  uint8_t current_gait{0};
  uint8_t last_motion_state{0};
  uint8_t last_gait{0};
  uint8_t sub_gait{0};
  std::array<float, 3> max_velocity{};

  uint32_t joint_timestamp_ms{0};
  uint32_t imu_timestamp_ms{0};
  uint32_t odometry_timestamp_ms{0};
  uint32_t motion_timestamp_ms{0};
  uint32_t battery_timestamp_ms{0};

  uint16_t robot_version_major{0};
  uint16_t robot_version_minor{0};
  uint16_t robot_version_patch{0};
  uint32_t robot_version_commit{0};
  uint32_t robot_build_date{0};
  uint32_t robot_build_time{0};

  std::string fault_reason;
  std::chrono::steady_clock::time_point received_at{};
};

class BpxReadOnlyClient
{
public:
  explicit BpxReadOnlyClient(ReadOnlyClientConfig config);
  ~BpxReadOnlyClient();

  BpxReadOnlyClient(const BpxReadOnlyClient &) = delete;
  BpxReadOnlyClient & operator=(const BpxReadOnlyClient &) = delete;

  void start();
  void stop();
  RobotStateSnapshot snapshot() const;
  bool sendMotionCommand(uint8_t motion, std::string * message);
  bool sendVelocityCommand(float x, float y, float yaw, std::string * message);
  bool stopVelocity(std::string * message = nullptr);

private:
  void workerLoop();
  bool connect();
  void disconnect();
  void pollState();
  void maintainStandCommand();
  void setDisconnected(const std::string & reason);

  ReadOnlyClientConfig config_;
  std::unique_ptr<bpx_sdk::MotionLevelControl> robot_state_;
  mutable std::mutex sdk_mutex_;
  mutable std::mutex snapshot_mutex_;
  RobotStateSnapshot snapshot_;
  std::thread worker_thread_;
  std::atomic<bool> running_{false};
  uint64_t last_timestamp_signature_{0};
  std::chrono::steady_clock::time_point last_posture_command_at_{};
  std::chrono::steady_clock::time_point stand_command_deadline_{};
  std::chrono::steady_clock::time_point next_stand_command_at_{};
  bool stand_command_active_{false};
  bool velocity_control_enabled_{false};
};

}  // namespace dog_driver

#endif  // DOG_DRIVER__BPX_READ_ONLY_CLIENT_HPP_
