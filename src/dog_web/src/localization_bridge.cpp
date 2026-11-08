#include "localization_bridge.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <filesystem>
#include <stdexcept>

LocalizationBridge::LocalizationBridge(rclcpp::Node &node,tf2_ros::Buffer &tf):node_(node),tf_(tf) {
  seed_=node.create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10);
  local_=node.create_client<std_srvs::srv::Trigger>("/map_localizer/relocalize");
  global_=node.create_client<std_srvs::srv::Trigger>("/map_localizer/global_relocalize");
  status_=node.create_subscription<std_msgs::msg::String>("/map_localizer/status",rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr msg){
      std::lock_guard<std::mutex> lock(mutex_);
      try {state_=nlohmann::json::parse(msg->data);if(!state_.is_object())state_=nlohmann::json::object();}
      catch(const std::exception &){state_=nlohmann::json::object();}
      receipt_=std::chrono::steady_clock::now();
    });
}
nlohmann::json LocalizationBridge::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);auto result=state_;
  result["fresh"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-receipt_).count()<.8;
  if(!result["fresh"].get<bool>())result["valid"]=false;
  return result;
}
nlohmann::json LocalizationBridge::command(const std::string &path,const nlohmann::json &body) {
  const auto state=snapshot();
  if(!state.value("fresh",false)||!state.value("map_ready",false))throw std::invalid_argument("定位节点或地图尚未就绪");
  namespace fs=std::filesystem;
  const auto project=fs::weakly_canonical(body.at("map_project").get<std::string>());
  const auto loaded=fs::weakly_canonical(state.at("map_path").get<std::string>());
  if(loaded!=project&&loaded!=project/"localization.pcd"&&loaded!=project/"cloud.pcd")
    throw std::invalid_argument("网页所选地图与定位节点加载的地图不同");
  if(path=="/api/localization/initialize") {
    const auto &ground=body.at("ground_position");const double yaw=body.at("yaw").get<double>(),height=body.at("body_height").get<double>();
    if(!ground.is_array()||ground.size()!=3||!std::isfinite(yaw)||std::abs(yaw)>3.141592654||
        !std::isfinite(height)||height<.1||height>1.5)throw std::invalid_argument("初始朝向或机身高度无效");
    double p[3];for(int i=0;i<3;++i){p[i]=ground.at(i).get<double>();if(!std::isfinite(p[i])||std::abs(p[i])>100000)throw std::invalid_argument("初始地面坐标无效");}
    auto odom=tf_.lookupTransform(state.value("odom_frame","odom"),state.value("base_frame","base_link"),tf2::TimePointZero);
    if(std::abs(node_.now().seconds()-rclcpp::Time(odom.header.stamp).seconds())>.5)throw std::invalid_argument("机身里程计 TF 已过期，无法确定当前倾角");
    const auto &r=odom.transform.rotation;tf2::Quaternion old(r.x,r.y,r.z,r.w);
    if(!std::isfinite(old.length())||std::abs(old.length()-1)>.001)throw std::invalid_argument("机身 TF 朝向无效");
    double roll,pitch,old_yaw;tf2::Matrix3x3(old).getRPY(roll,pitch,old_yaw);
    tf2::Quaternion q;q.setRPY(roll,pitch,yaw);q.normalize();
    const auto offset=tf2::quatRotate(q,tf2::Vector3(0,0,height));
    geometry_msgs::msg::PoseWithCovarianceStamped seed;seed.header.frame_id=state.value("map_frame","map");
    seed.pose.pose.position.x=p[0]+offset.x();seed.pose.pose.position.y=p[1]+offset.y();seed.pose.pose.position.z=p[2]+offset.z();
    seed.pose.pose.orientation.x=q.x();seed.pose.pose.orientation.y=q.y();seed.pose.pose.orientation.z=q.z();seed.pose.pose.orientation.w=q.w();
    seed_->publish(seed);
    return {{"success",true},{"message","初始位置与朝向已提交，等待多帧配准确认"}};
  }
  auto client=path=="/api/localization/global_relocalize"?global_:local_;
  if(path!="/api/localization/global_relocalize"&&path!="/api/localization/relocalize")throw std::invalid_argument("Unknown localization action");
  if(client==global_&&!state.value("global_relocalization_enabled",false))throw std::invalid_argument("全局重定位未在 YAML 中开启");
  if(!client->service_is_ready())throw std::invalid_argument("重定位服务未就绪");
  auto pending=client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if(pending.wait_for(std::chrono::seconds(2))!=std::future_status::ready){client->remove_pending_request(pending);throw std::invalid_argument("重定位请求回执超时");}
  const auto result=pending.get();return {{"success",result->success},{"message",result->message}};
}
