#include "local_navigation_node.hpp"

namespace dog_navigation {
LocalNavigation::LocalNavigation():Node("local_navigation") {
    configureParameters();
    buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_=std::make_unique<tf2_ros::TransformListener>(*buffer_);
    path_pub_=create_publisher<nav_msgs::msg::Path>("~/global_path",1);
    local_pub_=create_publisher<nav_msgs::msg::Path>("~/local_path",1);
    route_pub_=create_publisher<dog_interfaces::msg::TerrainRoute>("~/terrain_route",rclcpp::QoS(1).reliable());
    status_pub_=create_publisher<std_msgs::msg::String>("~/status",rclcpp::QoS(1).reliable().transient_local());
    stop_pub_=create_publisher<std_msgs::msg::Bool>("~/stop_required",1);
    obstacle_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("~/obstacles",1);
    goal_sub_=create_subscription<geometry_msgs::msg::PoseStamped>(
      declare_parameter<std::string>("goal_topic","/goal_pose"),10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m){receiveGoal(std::move(m));});
    localization_sub_=create_subscription<std_msgs::msg::String>(
      declare_parameter<std::string>("localization_topic","/map_localizer/status"),10,
      [this](std_msgs::msg::String::ConstSharedPtr m){receiveLocalization(std::move(m));});
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("cloud_topic","/lio/cloud_deskewed"),rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){receiveCloud(std::move(m));});
    cancel_=create_service<std_srvs::srv::Trigger>("~/cancel",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> r){
        goal_.reset();invalidate("cancelled");r->success=true;r->message="Goal cleared; no motion controller is connected";
      });
    timer_=create_wall_timer(std::chrono::duration<double>(1./rate_),[this]{tick();});
    invalidate("waiting_inputs");
  }
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<dog_navigation::LocalNavigation>());}
  catch(const std::exception &e){RCLCPP_ERROR(rclcpp::get_logger("local_navigation"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
