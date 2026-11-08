#pragma once
#include "localization_bridge.hpp"
#include "navigation_bridge.hpp"
#include "motion_settings.hpp"
#include "cloud_preview.hpp"
#include "stream_frequency.hpp"

#include <rclcpp/rclcpp.hpp>
#include <dog_interfaces/msg/robot_status.hpp>
#include <dog_interfaces/srv/set_motion.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nlohmann/json.hpp>
#include <chrono>
#include <cstring>
#include <cmath>
#include <mutex>
#include <thread>
#include <spawn.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>

extern char **environ;

// HTTP threads only exchange copied JSON with the ROS executor. No shell commands
// or user-supplied process arguments are accepted by this bridge.
class RobotBridge : public rclcpp::Node {
  using Json = nlohmann::json;
  using Clock = std::chrono::steady_clock;
public:
  RobotBridge(const std::string &output, bool sim_time, bool allow_mapping = true, bool mapping_bridge = true,
      std::shared_ptr<MotionSettings> settings = std::make_shared<MotionSettings>())
      : Node("dog_web_bridge"), settings_(std::move(settings)), output_(output), sim_time_(sim_time), allow_mapping_(allow_mapping), mapping_bridge_(mapping_bridge) {
    set_parameter(rclcpp::Parameter("use_sim_time", sim_time));
    tf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
    clouds_=std::make_unique<CloudPreview>(*this,*tf_);
    localization_=std::make_unique<LocalizationBridge>(*this,*tf_);
    navigation_=std::make_unique<NavigationBridge>(*this);
    velocity_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1).reliable());
    save_ = create_client<std_srvs::srv::Trigger>("/lio/save_pcd");
    motion_ = create_client<dog_interfaces::srv::SetMotion>("/dog/set_motion");
    subscribe<dog_interfaces::msg::RobotStatus>("/dog/status", "control", [this](const auto &m) {
      status_ = {{"connected", m.connected}, {"state_fresh", m.state_fresh},
        {"motion", m.current_motion_state}, {"gait", m.current_gait},
        {"control_level", m.control_level}, {"fault", m.fault_reason}};
    });
    subscribe<sensor_msgs::msg::BatteryState>("/battery_state", "battery", [this](const auto &m) {
      battery_ = std::isfinite(m.percentage) && m.percentage >= 0 ? Json(m.percentage * 100) : Json(nullptr);
    });
    subscribe<sensor_msgs::msg::PointCloud2>("/interfaces/lidar", "lidar", [](const auto &) {});
    subscribe<sensor_msgs::msg::Imu>("/interfaces/imu", "imu", [](const auto &) {});
    subscribe<sensor_msgs::msg::Imu>("/imu/data", "body_imu", [this](const auto &m) {
      const auto &q=m.orientation;imu_orientation_={q.x,q.y,q.z,q.w};
    });
    subscribe<nav_msgs::msg::Odometry>("/lio/odom", "lio", [](const auto &) {});
    subscribe<nav_msgs::msg::Odometry>("/dog/odom", "velocity", [this](const auto &m) {
      twist_ = {m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z};
      const auto &q=m.pose.pose.orientation;
      body_orientation_={q.x,q.y,q.z,q.w};
    });
    subscribe<sensor_msgs::msg::JointState>("/joint_states", "joints", [this](const auto &m) {
      joints_={{"names",m.name},{"positions",m.position}};
    });
    subscribe<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", "diagnostics", [this](const auto &m) {
      diagnostics_ = Json::array();
      for (const auto &d : m.status) diagnostics_.push_back({{"name", d.name}, {"level", d.level}, {"message", d.message}});
    });
    control_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    deadman_ = create_wall_timer(std::chrono::milliseconds(20), [this] {
      std::lock_guard<std::mutex> guard(mutex_);
      if (!owner_.empty() && (Clock::now() > lease_deadline_ || !ready())) {
        stop_reason_=!ready()?"robot_state_stale":"input_lease_expired";++timeout_stops_;stopLocked();
      }else if(!owner_.empty()&&velocity_active_){velocity_->publish(pending_velocity_);++published_commands_;}
    },control_group_);
  }
  ~RobotBridge() override {
    std::lock_guard<std::mutex> guard(mutex_);
    stopLocked();
    if (mapping_pid_ > 0) {
      kill(-mapping_pid_, SIGINT);
      // Do not kill map export during normal shutdown. Launch has a 110s timeout.
      waitpid(mapping_pid_, nullptr, 0);
    }
  }
  Json telemetry() {
    std::lock_guard<std::mutex> guard(mutex_);
    reap();
    Json streams = Json::object();
    for (const auto &[key, s] : streams_) {
      const double age = std::chrono::duration<double>(Clock::now() - s.received).count();
      streams[key] = {{"age_ms", age * 1000}, {"stamp_age_ms", (now().seconds() - s.stamp) * 1000},
        {"hz", s.rate.hz()},{"hz_available",s.rate.available()},{"hz_source","message_stamp"},
        {"fresh", age < 1.0 && std::abs(now().seconds() - s.stamp) < 1.0}};
    }
    const auto pose=poseLocked();const bool chain=!pose.is_null();
    return {{"ros", true}, {"streams", streams}, {"robot", status_}, {"battery", fresh("battery") ? battery_ : Json(nullptr)},
      {"velocity", fresh("velocity") ? twist_ : Json(nullptr)}, {"pose", pose}, {"tf_ready", chain},
      {"joints",fresh("joints")?joints_:Json(nullptr)},
      {"body_orientation",fresh("body_imu")?imu_orientation_:fresh("velocity")?body_orientation_:Json(nullptr)},
      {"diagnostics", diagnostics_}, {"control_ready", ready()}, {"control_claimed", !owner_.empty()},
      {"control_timing",{{"received_commands",received_commands_},{"published_commands",published_commands_},
        {"max_input_gap_ms",max_input_gap_ms_},{"timeout_stops",timeout_stops_},{"last_stop_reason",stop_reason_}}},
      {"limits", settings_->snapshot()},
      {"mapping", {{"owned", mapping_pid_ > 0}, {"stopping", stopping_}, {"exit_code", mapping_exit_},
        {"output_dir", session_output_}, {"start_allowed", allow_mapping_},
        {"save_available", allow_mapping_ && save_->service_is_ready()}}},
      {"localization",localization_->snapshot()},
      {"localization_ready",localization_->snapshot().value("valid",false)},
      {"navigation",navigation_->snapshot()},{"navigation_ready",navigation_->snapshot().value("ready",false)}};
  }
  std::shared_ptr<const CloudPacket> cloudPacket(bool scan) const {return clouds_->packet(scan);}
  Json cloudMetadata(const CloudPacket &p) const {return clouds_->metadata(p);}
  Json cloud() const {
    const auto p=cloudPacket(false);Json rows=Json::array();
    if(!p)return {{"points",rows},{"sequence",0},{"frame","odom"}};
    const auto stride=std::max(size_t(1),(p->bytes.size()/16+59999)/60000);
    for(size_t i=0;i<p->bytes.size();i+=16*stride){float xyz[3];std::memcpy(xyz,p->bytes.data()+i,12);rows.push_back({xyz[0],xyz[1],xyz[2]});}
    auto result=cloudMetadata(*p);result["full_count"]=p->bytes.size()/16;result["count"]=rows.size();
    result["points"]=std::move(rows);return result;
  }
  Json saveMotionSettings(const Json &request) {
    std::lock_guard<std::mutex> guard(mutex_);auto result=settings_->save(request);
    if(!owner_.empty())stopLocked();
    return result;
  }
  Json modelTelemetry() {
    std::lock_guard<std::mutex> guard(mutex_);const auto pose=poseLocked();
    return {{"joints",ready()&&fresh("joints")?joints_:Json(nullptr)},
      {"body_orientation",ready()?(fresh("body_imu")?imu_orientation_:fresh("velocity")?body_orientation_:Json(nullptr)):Json(nullptr)},
      {"pose",pose},{"tf_ready",!pose.is_null()},{"robot",ready()?status_:Json::object()}};
  }
  Json navigationPaths() { return navigation_->snapshot().at("live_paths"); }
  Json command(const std::string &path, const Json &body) {
    if(path=="/api/navigation/cancel"){std::lock_guard<std::mutex> lock(mutex_);stop_reason_="navigation_cancel";stopLocked();}
    if(path.rfind("/api/navigation/",0)==0)return navigation_->command(path,body);
    if(path.rfind("/api/localization/",0)==0)return localization_->command(path,body);
    if (!allow_mapping_ && (path == "/api/slam/start" || path == "/api/slam/save"))
      throw std::invalid_argument("当前为已有地图定位/重规划模式；请退出该模式后再启动建图");
    if (path == "/api/slam/save") {
      if (!save_->service_is_ready()) throw std::invalid_argument("LIO 保存服务未就绪");
      auto pending = save_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      if (pending.wait_for(std::chrono::seconds(60)) != std::future_status::ready) {
        save_->remove_pending_request(pending);
        throw std::invalid_argument("保存回执超时；服务端可能仍在保存，请检查输出目录再重试");
      }
      auto result = pending.get(); return {{"success", result->success}, {"message", result->message}};
    }
    std::unique_lock<std::mutex> guard(mutex_);
    reap();
    if (path == "/api/slam/start") {
      if (mapping_pid_ > 0 || save_->service_is_ready() || fresh("lio"))
        throw std::invalid_argument("已有建图进程或外部 LIO；请使用现有会话");
      session_output_ = output_ + "/session-" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
      std::string arg = "output_dir:=" + session_output_;
      std::string sim_arg = sim_time_ ? "use_sim_time:=true" : "use_sim_time:=false";
      std::string bridge_arg=mapping_bridge_?"bridge:=true":"bridge:=false";
      char *args[] = {const_cast<char *>("ros2"), const_cast<char *>("launch"), const_cast<char *>("dog_slam"),
        const_cast<char *>("mapping.launch.py"), arg.data(), const_cast<char *>("loop_closure:=false"), sim_arg.data(), bridge_arg.data(), nullptr};
      posix_spawnattr_t attr; posix_spawnattr_init(&attr);
      posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP); posix_spawnattr_setpgroup(&attr, 0);
      const int error = posix_spawnp(&mapping_pid_, "ros2", nullptr, &attr, args, environ);
      posix_spawnattr_destroy(&attr);
      if (error) { mapping_pid_ = -1; throw std::runtime_error("Cannot start ros2 launch: " + std::to_string(error)); }
      clouds_->resetMap();
      stopping_ = false; mapping_exit_ = nullptr;
      return {{"success", true}, {"message", "建图启动请求已提交；请等待 LIO 数据就绪"}};
    }
    if (path == "/api/slam/stop") {
      if (mapping_pid_ <= 0) throw std::invalid_argument("此网页没有管理中的建图进程；不能停止外部会话");
      if (!stopping_) kill(-mapping_pid_, SIGINT);
      stopping_ = true;
      return {{"success", true}, {"message", "已请求结束建图，等待退出保存完成"}};
    }
    if (path == "/api/control/stop") {
      stop_reason_="stop_requested";
      stopLocked();
      return {{"success", true}, {"message", "已发布零速度；机器人实际停车须由遥测确认"}};
    }
    if (path == "/api/control/acquire") {
      if (!ready()) throw std::invalid_argument("机器人连接或状态不新鲜，无法取得网页控制权");
      if (!owner_.empty() && Clock::now() < lease_deadline_) throw std::invalid_argument("另一网页正在控制机器人");
      if(!owner_.empty())stopLocked();
      owner_ = std::to_string(++lease_id_) + "-" + std::to_string(Clock::now().time_since_epoch().count());
      lease_deadline_ = Clock::now() + std::chrono::milliseconds(350);
      return {{"token", owner_}};
    }
    if (path.rfind("/api/control/", 0) != 0) throw std::invalid_argument("Unknown robot command");
    if (owner_.empty() || body.value("token", "") != owner_ || Clock::now() > lease_deadline_ || !ready()) {
      throw std::invalid_argument("网页控制权已失效，请重新按住控制键");
    }
    if (path == "/api/control/release") {stop_reason_="client_release";stopLocked(); return {{"success", true}}; }
    if (path == "/api/control/velocity") {
      geometry_msgs::msg::Twist msg;
      auto bounded = [&body](const char *key, double limit) {
        double value = body.at(key).get<double>();
        if (!std::isfinite(value) || std::abs(value) > limit) throw std::invalid_argument("速度超出网页控制上限");
        return value;
      };
      const auto limits=settings_->limits();
      msg.linear.x = bounded("x", limits[0]); msg.linear.y = bounded("y", limits[1]); msg.angular.z = bounded("yaw", limits[2]);
      // Invalid requests must never keep a previous nonzero command alive.
      const auto receipt=Clock::now();
      if(velocity_active_)max_input_gap_ms_=std::max(max_input_gap_ms_,std::chrono::duration<double,std::milli>(receipt-last_input_).count());
      last_input_=receipt;++received_commands_;
      lease_deadline_ = receipt + std::chrono::milliseconds(350);
      const bool changed=msg.linear.x!=pending_velocity_.linear.x||msg.linear.y!=pending_velocity_.linear.y||msg.angular.z!=pending_velocity_.angular.z;
      pending_velocity_=msg;
      const bool moving=std::abs(msg.linear.x)+std::abs(msg.linear.y)+std::abs(msg.angular.z)>1e-8;
      if(changed||!velocity_active_||!moving){velocity_->publish(msg);++published_commands_;}
      velocity_active_=moving;return {{"success", true}};
    }
    if (path == "/api/control/motion") {
      int value = body.at("motion").get<int>();
      if (value < 0 || value > 3 || !body.value("confirm", false)) throw std::invalid_argument("仅允许明确确认的站立、坐下、阻尼和行走模式");
      if (!motion_->service_is_ready()) throw std::invalid_argument("姿态控制服务未就绪");
      velocity_active_=false;pending_velocity_=geometry_msgs::msg::Twist();stop_reason_="posture_request";
      velocity_->publish(geometry_msgs::msg::Twist());
      auto req = std::make_shared<dog_interfaces::srv::SetMotion::Request>();
      req->motion = value; req->confirm_restricted_motion = true;
      auto pending = motion_->async_send_request(req);
      guard.unlock();
      if (pending.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        motion_->remove_pending_request(pending); throw std::invalid_argument("姿态指令回执超时，执行结果未知");
      }
      auto result = pending.get(); return {{"success", result->accepted}, {"message", result->message}};
    }
    throw std::invalid_argument("Unknown control command");
  }
private:
  Json poseLocked() const {
    try {
      auto t=tf_->lookupTransform("map","base_link",tf2::TimePointZero);
      tf_->lookupTransform("base_link","livox_frame",tf2::TimePointZero);
      const double age=now().seconds()-rclcpp::Time(t.header.stamp).seconds();
      if(std::abs(age)>=1||!fresh("lio"))return nullptr;
      const auto &p=t.transform.translation;const auto &q=t.transform.rotation;
      return {{"position",{p.x,p.y,p.z}},{"quaternion",{q.x,q.y,q.z,q.w}},{"frame","map"},{"age_ms",age*1000}};
    }catch(const tf2::TransformException &){return nullptr;}
  }
  struct Stream { Clock::time_point received; double stamp = 0;StreamFrequency rate; };
  template<class Message, class Callback> void subscribe(const std::string &topic, const std::string &key, Callback callback) {
    subscriptions_.push_back(create_subscription<Message>(topic, rclcpp::SensorDataQoS(),
      [this, key, callback](typename Message::ConstSharedPtr m) {
        std::lock_guard<std::mutex> guard(mutex_);
        auto &s = streams_[key]; const auto received = Clock::now();
        s.rate.observe(rclcpp::Time(m->header.stamp).nanoseconds());
        s.received = received; s.stamp = rclcpp::Time(m->header.stamp).seconds(); callback(*m);
      }));
  }
  bool fresh(const std::string &key) const {
    auto i = streams_.find(key);
    return i != streams_.end() && Clock::now() - i->second.received < std::chrono::seconds(1) &&
      std::abs(now().seconds() - i->second.stamp) < 1;
  }
  bool ready() const { return fresh("control") && status_.value("connected", false) && status_.value("state_fresh", false); }
  void stopLocked() {
    if (rclcpp::ok()) velocity_->publish(geometry_msgs::msg::Twist());
    owner_.clear();
    pending_velocity_=geometry_msgs::msg::Twist();velocity_active_=false;
  }
  void reap() {
    if (mapping_pid_ <= 0) return;
    int status;
    if (waitpid(mapping_pid_, &status, WNOHANG) == mapping_pid_) {
      mapping_exit_ = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
      mapping_pid_ = -1; stopping_ = false;
    }
  }
  std::mutex mutex_;
  std::map<std::string, Stream> streams_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_;
  rclcpp::Client<dog_interfaces::srv::SetMotion>::SharedPtr motion_;
  rclcpp::TimerBase::SharedPtr deadman_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  geometry_msgs::msg::Twist pending_velocity_;
  bool velocity_active_=false;
  Clock::time_point last_input_;
  uint64_t received_commands_=0,published_commands_=0,timeout_stops_=0;
  double max_input_gap_ms_=0;
  std::string stop_reason_="idle";
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<LocalizationBridge> localization_;
  std::unique_ptr<NavigationBridge> navigation_;
  std::unique_ptr<CloudPreview> clouds_;
  std::shared_ptr<MotionSettings> settings_;
  Json status_ = Json::object(), battery_ = nullptr, twist_ = nullptr, diagnostics_ = Json::array();
  Json joints_=nullptr,body_orientation_=nullptr,imu_orientation_=nullptr;
  uint64_t lease_id_ = 0;
  std::string owner_, output_, session_output_;
  Clock::time_point lease_deadline_;
  pid_t mapping_pid_ = -1;
  bool stopping_ = false;
  bool sim_time_ = false;
  const bool allow_mapping_;
  const bool mapping_bridge_;
  Json mapping_exit_ = nullptr;
};
