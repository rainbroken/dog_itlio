#include "dog_driver/bpx_read_only_client.hpp"

#include "motion_level_control.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <utility>

namespace dog_driver
{

BpxReadOnlyClient::BpxReadOnlyClient(ReadOnlyClientConfig config)
: config_(std::move(config))
{
}

BpxReadOnlyClient::~BpxReadOnlyClient()
{
  stop();
}

void BpxReadOnlyClient::start()
{
  if (running_.exchange(true)) {
    return;
  }
  worker_thread_ = std::thread(&BpxReadOnlyClient::workerLoop, this);
}

void BpxReadOnlyClient::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

RobotStateSnapshot BpxReadOnlyClient::snapshot() const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

bool BpxReadOnlyClient::sendMotionCommand(uint8_t motion, std::string * message)
{
  const auto state = snapshot();
  if (!state.connected || !state.state_fresh) {
    if (message) {
      *message = "robot state is not connected and fresh";
    }
    return false;
  }
  if (!state.has_motion_state) {
    if (message) {
      *message = "robot motion state is unavailable";
    }
    return false;
  }
  if (motion == 0U &&
    state.current_motion_state != static_cast<uint8_t>(bpx_sdk::MotionState::LyingDown) &&
    state.current_motion_state != static_cast<uint8_t>(bpx_sdk::MotionState::SitDown) &&
    state.current_motion_state != static_cast<uint8_t>(bpx_sdk::MotionState::Passive))
  {
    if (message) {
      *message = "stand is allowed only from lying-down, sit-down, or damping state";
    }
    return false;
  }
  if (motion == 1U &&
    state.current_motion_state != static_cast<uint8_t>(bpx_sdk::MotionState::Motion))
  {
    if (message) {
      *message = "sit-down is allowed only from stable standing motion state";
    }
    return false;
  }
  const bool is_stand_or_sit = motion == 0U || motion == 1U;
  const bool is_damping = motion == 2U;
  if (!is_stand_or_sit && !is_damping &&
    state.current_motion_state != static_cast<uint8_t>(bpx_sdk::MotionState::Motion))
  {
    if (message) {
      *message = "restricted motion is allowed only from stable standing motion state";
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(sdk_mutex_);
  if (!robot_state_ || !robot_state_->isConnected()) {
    if (message) {
      *message = "motion SDK is disconnected";
    }
    return false;
  }
  const auto now = std::chrono::steady_clock::now();
  if (last_posture_command_at_.time_since_epoch().count() != 0 &&
    now - last_posture_command_at_ < std::chrono::seconds(2))
  {
    if (message) {
      *message = "motion command cooldown is active";
    }
    return false;
  }

  if (velocity_control_enabled_) {
    robot_state_->setVelocity(0.0F, 0.0F, 0.0F);
    robot_state_->setVelocityControlFlag(false);
    velocity_control_enabled_ = false;
  }

  bool sent = true;
  stand_command_active_ = false;
  switch (motion) {
    case 0U:
      sent = robot_state_->setStandUp();
      if (sent) {
        stand_command_active_ = true;
        stand_command_deadline_ = now + std::chrono::seconds(30);
        next_stand_command_at_ = now + std::chrono::milliseconds(100);
      }
      break;
    case 1U:
      sent = robot_state_->setSitDown();
      break;
    case 2U:
      sent = robot_state_->setDamping();
      break;
    case 3U:
      robot_state_->setWalk();
      break;
    case 4U:
      robot_state_->setRunning();
      break;
    case 5U:
      robot_state_->setPace();
      break;
    case 6U:
      robot_state_->setLeftFlip();
      break;
    case 7U:
      robot_state_->setRightFlip();
      break;
    case 8U:
      robot_state_->setBipedal();
      break;
    case 9U:
      robot_state_->setInvBipedal();
      break;
    case 10U:
      robot_state_->setPronk();
      break;
    case 11U:
      robot_state_->setBound();
      break;
    default:
      if (message) {
        *message = "unknown motion command";
      }
      return false;
  }
  if (sent) {
    last_posture_command_at_ = now;
  }
  if (message) {
    *message = sent ? "motion command sent" : "SDK rejected the motion command";
  }
  return sent;
}

bool BpxReadOnlyClient::sendVelocityCommand(
  float x, float y, float yaw, std::string * message)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
    if (message) {
      *message = "velocity command contains a non-finite value";
    }
    return false;
  }
  if (std::abs(x) > config_.safe_velocity_x ||
    std::abs(y) > config_.safe_velocity_y ||
    std::abs(yaw) > config_.safe_velocity_yaw)
  {
    if (message) {
      *message = "velocity command exceeds the configured safe limits";
    }
    return false;
  }
  if (std::sqrt(x * x + y * y + yaw * yaw) < 1.0e-4F) {
    return stopVelocity(message);
  }

  const auto state = snapshot();
  if (!state.connected || !state.state_fresh || !state.has_motion_state) {
    if (message) {
      *message = "robot state is not connected and fresh";
    }
    return false;
  }
  if (state.current_motion_state != static_cast<uint8_t>(bpx_sdk::MotionState::Motion)) {
    if (message) {
      *message = "velocity is allowed only from stable standing motion state";
    }
    return false;
  }
  if (state.current_gait != static_cast<uint8_t>(bpx_sdk::MotionGait::Walk)) {
    if (message) {
      *message = "velocity is allowed only while Walk gait is selected";
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(sdk_mutex_);
  if (!robot_state_ || !robot_state_->isConnected()) {
    if (message) {
      *message = "motion SDK is disconnected";
    }
    return false;
  }
  if (!velocity_control_enabled_) {
    robot_state_->setVelocityControlFlag(true);
    velocity_control_enabled_ = true;
  }
  const bool sent = robot_state_->setVelocity(x, y, yaw);
  if (message) {
    *message = sent ? "velocity command sent" : "SDK rejected the velocity command";
  }
  return sent;
}

bool BpxReadOnlyClient::stopVelocity(std::string * message)
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  if (!robot_state_) {
    if (message) {
      *message = "motion SDK is disconnected";
    }
    return false;
  }
  if (!velocity_control_enabled_) {
    if (message) {
      *message = "velocity control is already inactive";
    }
    return true;
  }
  const bool sent = robot_state_->setVelocity(0.0F, 0.0F, 0.0F);
  if (message) {
    *message = sent ? "zero velocity sent" : "SDK rejected the zero velocity command";
  }
  return sent;
}

void BpxReadOnlyClient::workerLoop()
{
  while (running_) {
    if (!robot_state_) {
      if (!connect()) {
        std::this_thread::sleep_for(config_.reconnect_period);
        continue;
      }
    }

    pollState();
    maintainStandCommand();

    bool reconnect = false;
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex_);
      if (snapshot_.received_at.time_since_epoch().count() != 0) {
        const auto age = std::chrono::steady_clock::now() - snapshot_.received_at;
        snapshot_.state_fresh = snapshot_.connected && age <= config_.stale_timeout;
        reconnect = age > config_.stale_timeout * 5;
      }
    }

    if (reconnect) {
      setDisconnected("robot state stream timed out");
      disconnect();
      std::this_thread::sleep_for(config_.reconnect_period);
      continue;
    }

    std::this_thread::sleep_for(config_.poll_period);
  }

  disconnect();
}

bool BpxReadOnlyClient::connect()
{
  auto robot_state = std::make_unique<bpx_sdk::MotionLevelControl>();
  robot_state->setRobotIp(config_.robot_ip.c_str());
  robot_state->setRobotStateUploadPort(config_.robot_state_port);
  robot_state->setTcpLocalPort(config_.tcp_local_port);
  robot_state->setSessionId(config_.session_id);
  robot_state->setRobotStateUploadRate(config_.state_upload_rate_hz);
  robot_state->setMotionCommandRate(config_.motion_command_rate_hz);
  robot_state->setVelocityControlFlag(false);

  if (!robot_state->connect()) {
    setDisconnected("failed to connect read-only robot state client");
    return false;
  }

  RobotStateSnapshot connected_snapshot;
  connected_snapshot.connected = robot_state->isConnected();
  connected_snapshot.fault_reason = connected_snapshot.connected ? "" :
    "waiting for SDK state connection";
  connected_snapshot.robot_version_supported = robot_state->getRobotVersion(
    &connected_snapshot.robot_version_major,
    &connected_snapshot.robot_version_minor,
    &connected_snapshot.robot_version_patch,
    &connected_snapshot.robot_version_commit,
    &connected_snapshot.robot_build_date,
    &connected_snapshot.robot_build_time);

  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_ = connected_snapshot;
  }

  last_timestamp_signature_ = 0;
  robot_state_ = std::move(robot_state);
  return true;
}

void BpxReadOnlyClient::disconnect()
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  if (robot_state_) {
    if (velocity_control_enabled_) {
      robot_state_->setVelocity(0.0F, 0.0F, 0.0F);
      robot_state_->setVelocityControlFlag(false);
      velocity_control_enabled_ = false;
    }
    robot_state_->disconnect();
    robot_state_.reset();
  }

  {
    std::lock_guard<std::mutex> snapshot_lock(snapshot_mutex_);
    snapshot_.connected = false;
    snapshot_.state_fresh = false;
  }
}

void BpxReadOnlyClient::pollState()
{
  std::lock_guard<std::mutex> lock(sdk_mutex_);
  RobotStateSnapshot next = snapshot();
  if (!robot_state_->isConnected()) {
    // SDK 1.0.8+ reconnects in the background. Never replay an action from
    // the previous session or treat its cached UDP state as a live connection.
    stand_command_active_ = false;
    velocity_control_enabled_ = false;
    robot_state_->setVelocityControlFlag(false);
    next = RobotStateSnapshot{};
    next.fault_reason = "waiting for SDK state connection";
    last_timestamp_signature_ = 0;
    std::lock_guard<std::mutex> snapshot_lock(snapshot_mutex_);
    snapshot_ = next;
    return;
  }
  next.connected = true;
  next.robot_version_supported = robot_state_->getRobotVersion(
    &next.robot_version_major, &next.robot_version_minor, &next.robot_version_patch,
    &next.robot_version_commit, &next.robot_build_date, &next.robot_build_time);

  next.has_joint_state =
    robot_state_->getJointPosition(next.joint_position.data()) &&
    robot_state_->getJointVelocity(next.joint_velocity.data()) &&
    robot_state_->getJointTorque(next.joint_torque.data());

  next.has_imu =
    robot_state_->getImuQuat(next.imu_quaternion.data()) &&
    robot_state_->getImuAcc(next.imu_acceleration.data()) &&
    robot_state_->getImuOmega(next.imu_angular_velocity.data());

  next.has_odometry = robot_state_->getLegOdom(&next.leg_odometry);
  next.has_temperature =
    robot_state_->getMotorTemperature(next.motor_temperature.data()) &&
    robot_state_->getDriverTemperature(next.driver_temperature.data());
  next.has_battery =
    robot_state_->getBatteryLevel(&next.battery_level) &&
    robot_state_->getBatteryCurrent(&next.battery_current);
  next.has_motion_state =
    robot_state_->getCurrentMotionState(&next.current_motion_state) &&
    robot_state_->getCurrentGait(&next.current_gait) &&
    robot_state_->getLastMotionState(&next.last_motion_state) &&
    robot_state_->getLastGait(&next.last_gait) &&
    robot_state_->getSubGait(&next.sub_gait) &&
    robot_state_->getMaxVelocity(next.max_velocity.data());

  robot_state_->getJointStateTimestamp(&next.joint_timestamp_ms);
  robot_state_->getImuTimestamp(&next.imu_timestamp_ms);
  robot_state_->getOdometryTimestamp(&next.odometry_timestamp_ms);
  robot_state_->getMotionStateTimestamp(&next.motion_timestamp_ms);
  robot_state_->getBatteryTimestamp(&next.battery_timestamp_ms);

  const uint64_t timestamp_signature =
    static_cast<uint64_t>(next.joint_timestamp_ms) ^
    (static_cast<uint64_t>(next.imu_timestamp_ms) << 7U) ^
    (static_cast<uint64_t>(next.odometry_timestamp_ms) << 13U) ^
    (static_cast<uint64_t>(next.motion_timestamp_ms) << 19U) ^
    (static_cast<uint64_t>(next.battery_timestamp_ms) << 25U);
  const bool has_any_state =
    next.has_joint_state || next.has_imu || next.has_odometry ||
    next.has_battery || next.has_temperature || next.has_motion_state;

  if (has_any_state &&
    (timestamp_signature != last_timestamp_signature_ ||
    next.received_at.time_since_epoch().count() == 0))
  {
    next.received_at = std::chrono::steady_clock::now();
    next.state_fresh = true;
    next.fault_reason.clear();
    last_timestamp_signature_ = timestamp_signature;
  }

  std::lock_guard<std::mutex> snapshot_lock(snapshot_mutex_);
  snapshot_ = next;
}

void BpxReadOnlyClient::maintainStandCommand()
{
  const auto state = snapshot();
  const auto now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(sdk_mutex_);
  if (!stand_command_active_) {
    return;
  }
  if (!robot_state_ || !robot_state_->isConnected() ||
    !state.connected || !state.state_fresh ||
    (state.has_motion_state &&
    state.current_motion_state == static_cast<uint8_t>(bpx_sdk::MotionState::Motion)) ||
    now >= stand_command_deadline_)
  {
    stand_command_active_ = false;
    return;
  }
  if (now >= next_stand_command_at_) {
    robot_state_->setStandUp();
    next_stand_command_at_ = now + std::chrono::milliseconds(100);
  }
}

void BpxReadOnlyClient::setDisconnected(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_.connected = false;
  snapshot_.state_fresh = false;
  snapshot_.fault_reason = reason;
}

}  // namespace dog_driver
