#include "navigation_bridge.hpp"
#include <cmath>
#include <filesystem>

NavigationBridge::NavigationBridge(rclcpp::Node &node) : node_(node) {
  action_ = rclcpp_action::create_client<Action>(&node, "/navigate_terrain");
  pause_ =
      node.create_client<std_srvs::srv::Trigger>("/navigation_executor/pause");
  resume_ =
      node.create_client<std_srvs::srv::Trigger>("/navigation_executor/resume");
  cancel_ =
      node.create_client<std_srvs::srv::Trigger>("/navigation_executor/cancel");
  status_ = node.create_subscription<std_msgs::msg::String>(
      "/navigation_executor/status", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        try {
          state_ = nlohmann::json::parse(m->data);
          if (!state_.is_object())
            state_ = nlohmann::json::object();
        } catch (const std::exception &) {
          state_ = nlohmann::json::object();
        }
        received_ = std::chrono::steady_clock::now();
      });
}
nlohmann::json NavigationBridge::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = state_;
  const bool fresh = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - received_)
                         .count() < .8;
  result["fresh"] = fresh;
  result["ready"] = fresh && result.value("enabled", false) &&
                    result.value("guard_auto_enabled", false) &&
                    result.value("localization_ready", false);
  return result;
}
nlohmann::json NavigationBridge::command(const std::string &path,
                                         const nlohmann::json &body) {
  // Cancellation remains available even after localization or a map selection
  // changes.
  if (path != "/api/navigation/start") {
    auto client = path == "/api/navigation/pause"    ? pause_
                  : path == "/api/navigation/resume" ? resume_
                                                     : cancel_;
    if (path != "/api/navigation/pause" && path != "/api/navigation/resume" &&
        path != "/api/navigation/cancel")
      throw std::invalid_argument("Unknown navigation action");
    if (!client->service_is_ready())
      throw std::invalid_argument("导航任务服务未就绪");
    auto future = client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      client->remove_pending_request(future);
      throw std::invalid_argument("导航任务回执超时");
    }
    auto response = future.get();
    return {{"success", response->success}, {"message", response->message}};
  }
  const auto state = snapshot();
  if (!state.value("ready", false))
    throw std::invalid_argument(
        "导航未就绪：检查执行开关、标定、安全仲裁与定位状态");
  if (state.at("map_revision") != body.at("revision") ||
      std::filesystem::weakly_canonical(
          state.at("map_project").get<std::string>()) !=
          std::filesystem::weakly_canonical(
              body.at("map_project").get<std::string>()))
    throw std::invalid_argument("导航与网页所选地图版本不一致");
  const auto &point = body.at("goal");
  if (!point.is_array() || point.size() != 3)
    throw std::invalid_argument("目标需要 XYZ 坐标");
  Action::Goal goal;
  goal.goal.header.frame_id = "map";
  goal.goal.header.stamp = node_.now();
  goal.goal.pose.orientation.w = 1;
  double p[3];
  for (int i = 0; i < 3; ++i) {
    p[i] = point.at(i).get<double>();
    if (!std::isfinite(p[i]))
      throw std::invalid_argument("无效目标坐标");
  }
  goal.goal.pose.position.x = p[0];
  goal.goal.pose.position.y = p[1];
  goal.goal.pose.position.z = p[2];
  if (!action_->action_server_is_ready())
    throw std::invalid_argument("导航 Action 未就绪");
  auto future = action_->async_send_goal(goal);
  if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    throw std::invalid_argument(
        "任务回执超时；请查看任务状态或取消，勿重复提交");
  const bool accepted = bool(future.get());
  return {{"success", accepted},
          {"message",
           accepted ? "导航任务已接收，起点使用实时定位" : "导航任务被拒绝"}};
}
