#include "navigation_bridge.hpp"
#include <cmath>
#include <filesystem>

NavigationBridge::NavigationBridge(rclcpp::Node &node) : node_(node) {
  planner_status_=node.create_subscription<std_msgs::msg::String>(
      "/local_navigation/status",rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr m){
        std::lock_guard<std::mutex> lock(mutex_);
        try{planner_state_=nlohmann::json::parse(m->data);if(!planner_state_.is_object())planner_state_=nlohmann::json::object();}
        catch(const std::exception &){planner_state_=nlohmann::json::object();}
        planner_received_=std::chrono::steady_clock::now();++path_sequence_;
      });
  auto path_callback=[this](bool local,nav_msgs::msg::Path::ConstSharedPtr m){
    std::lock_guard<std::mutex> lock(mutex_);
    auto &path=local?local_path_:global_path_;path=nav_msgs::msg::Path();++path_sequence_;
    if(m->header.frame_id!="map"||m->poses.size()>5000)return;
    for(const auto &pose:m->poses){const auto &p=pose.pose.position;if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return;}
    path=*m;
  };
  global_path_sub_=node.create_subscription<nav_msgs::msg::Path>("/local_navigation/global_path",1,
      [path_callback](nav_msgs::msg::Path::ConstSharedPtr m){path_callback(false,m);});
  local_path_sub_=node.create_subscription<nav_msgs::msg::Path>("/local_navigation/local_path",1,
      [path_callback](nav_msgs::msg::Path::ConstSharedPtr m){path_callback(true,m);});
  action_ = rclcpp_action::create_client<Action>(&node, "/navigate_terrain");
  pause_ =
      node.create_client<std_srvs::srv::Trigger>("/navigation_executor/pause");
  resume_ =
      node.create_client<std_srvs::srv::Trigger>("/navigation_executor/resume");
  cancel_ =
      node.create_client<std_srvs::srv::Trigger>("/navigation_executor/cancel");
  guard_cancel_=node.create_client<std_srvs::srv::Trigger>("/velocity_guard/cancel");
  planner_cancel_=node.create_client<std_srvs::srv::Trigger>("/local_navigation/cancel");
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
  auto paths=nlohmann::json{{"valid",false},{"reason",planner_state_.value("state","waiting_planner")},
      {"sequence",path_sequence_},{"frame_id","map"},{"global_path",nlohmann::json::array()},
      {"local_path",nlohmann::json::array()}};
  try{
    const auto stamp=planner_state_.at("route_stamp");
    const auto &header=global_path_.header;
    const double age=std::chrono::duration<double>(std::chrono::steady_clock::now()-planner_received_).count();
    const double lease=planner_state_.at("valid_for_seconds").get<double>();
    const double source_age=node_.now().seconds()-rclcpp::Time(header.stamp).seconds();
    const bool same_map=planner_state_.at("map_project")==state_.at("map_project")&&
                        planner_state_.at("map_revision")==state_.at("map_revision");
    const bool same_frame=header.frame_id=="map"&&local_path_.header.frame_id=="map";
    const bool same_goal=state_.at("planner_goal_stamp")==planner_state_.at("goal_stamp");
    const bool same_stamp=header.stamp==local_path_.header.stamp&&
        header.stamp.sec==stamp.at("sec").get<int32_t>()&&header.stamp.nanosec==stamp.at("nanosec").get<uint32_t>();
    paths["valid"]=fresh&&same_map&&same_goal&&same_frame&&same_stamp&&planner_state_.at("valid").get<bool>()&&
        std::isfinite(lease)&&lease>0&&age<std::min(.8,lease)&&source_age>=-.05&&source_age<lease&&
        !global_path_.poses.empty()&&!local_path_.poses.empty();
    if(paths["valid"].get<bool>()){
      paths["valid_for_ms"]=1000*(lease-std::max(age,source_age));
      paths["task_id"]=state_.value("task_id","");
      paths["map_revision"]=state_.at("map_revision");paths["map_project"]=state_.at("map_project");
      for(const auto &pose:global_path_.poses){const auto &p=pose.pose.position;paths["global_path"].push_back({p.x,p.y,p.z});}
      for(const auto &pose:local_path_.poses){const auto &p=pose.pose.position;paths["local_path"].push_back({p.x,p.y,p.z});}
    }else if(!same_map)paths["reason"]="map_mismatch";
    else if(!same_goal)paths["reason"]="goal_mismatch";
    else if(planner_state_.value("valid",false)&&(age>=std::min(.8,lease)||source_age>=lease))paths["reason"]="route_expired";
  }catch(const std::exception &){}
  result["live_paths"]=std::move(paths);
  return result;
}
nlohmann::json NavigationBridge::command(const std::string &path,
                                         const nlohmann::json &body) {
  if(path=="/api/navigation/cancel"){
    // Dispatch every request before waiting: a slow executor must not delay
    // the independent guard's stop. Cancellation never depends on a map.
    using Client=rclcpp::Client<std_srvs::srv::Trigger>;
    std::vector<std::pair<Client::SharedPtr,Client::FutureAndRequestId>> requests;
    bool complete=true;std::string failures;
    for(auto client:{guard_cancel_,cancel_,planner_cancel_}){
      if(!client->service_is_ready()){complete=false;failures+=std::string(client->get_service_name())+" 未就绪；";continue;}
      requests.emplace_back(client,client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()));
    }
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    for(auto &[client,future]:requests){
      if(future.wait_until(deadline)!=std::future_status::ready){client->remove_pending_request(future);complete=false;failures+=std::string(client->get_service_name())+" 超时；";}
      else if(!future.get()->success){complete=false;failures+=std::string(client->get_service_name())+" 未确认；";}
    }
    return {{"success",complete},{"message",complete?"已取消导航任务并发布停车指令；实际停车以速度反馈为准":"已发布停车请求，但取消未全部确认："+failures}};
  }
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
