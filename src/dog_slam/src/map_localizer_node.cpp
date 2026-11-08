#include "map_localizer_node.hpp"
#include "ros_pose.hpp"

namespace dog_slam {
MapLocalizer::MapLocalizer() : Node("map_localizer") {
    configureParameters();
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", rclcpp::QoS(1).transient_local().reliable());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/map", rclcpp::QoS(1).transient_local().reliable());
    diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { receive(std::move(msg)); });
    seed_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        try {
          if (msg->header.frame_id != map_frame_) throw std::invalid_argument("Initial pose must be in map_frame");
          const double stamp = rclcpp::Time(msg->header.stamp).seconds();
          if (stamp < 0 || (stamp > 0 && std::abs(now().seconds() - stamp) > max_scan_age_))
            throw std::invalid_argument("Initial pose timestamp is stale or in the future");
          Command c; c.kind=Kind::Seed; c.seed=pose(msg->pose.pose); c.stamp=msg->header.stamp;
          enqueue(std::move(c), "initializing");
        } catch (const std::exception &e) { RCLCPP_WARN(get_logger(), "Reject initial pose: %s", e.what()); }
      });
    reload_srv_ = create_service<std_srvs::srv::Trigger>("~/reload_map",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> out) {
        const auto path=get_parameter("map_path").as_string();
        if (path.empty()) {out->message="Set map_path first"; return;}
        Command c; c.kind=Kind::Load; c.path=path; enqueue(std::move(c), "loading_map");
        out->success=true; out->message="Map load queued; watch map_localizer/status for completion";
      });
    relocalize_srv_ = create_service<std_srvs::srv::Trigger>("~/relocalize",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> out) {
        {std::lock_guard<std::mutex> lock(mutex_);
          if (!public_.has_prior || !map_ready_) {out->message="Load a map and provide /initialpose first"; return;}}
        Command c; c.kind=Kind::Relocalize; enqueue(std::move(c), "relocalizing");
        out->success=true; out->message="Bounded relocalization queued; this is not a successful match yet";
      });
    reset_srv_ = create_service<std_srvs::srv::Trigger>("~/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> out) {
        Command c; c.kind=Kind::Reset; enqueue(std::move(c), "waiting_initial_pose");
        out->success=true; out->message="Localization reset queued";
      });
    global_srv_=create_service<std_srvs::srv::Trigger>("~/global_relocalize",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> out) {
        if(!options_.global.enabled){out->message="Global relocalization is disabled in YAML";return;}
        {std::lock_guard<std::mutex> lock(mutex_);if(!map_ready_){out->message="Load a map first";return;}}
        Command c;c.kind=Kind::GlobalRelocalize;enqueue(std::move(c),"global_relocalizing");
        out->success=true;out->message="Map-wide gravity-constrained search queued; watch status for confirmation";
      });
    tf_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {publishTransform();});
    status_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] {publishStatus();});
    const auto initial_path=get_parameter("map_path").as_string();
    if (!initial_path.empty()) {Command c; c.kind=Kind::Load; c.path=initial_path; enqueue(std::move(c), "loading_map");}
    const auto initial=declare_parameter<std::vector<double>>("initial_pose",std::vector<double>{});
    if (!initial.empty()) {
      if (initial.size()!=6 || !std::all_of(initial.begin(),initial.end(),[](double x){return std::isfinite(x);}))
        throw std::invalid_argument("initial_pose must contain finite x,y,z,roll,pitch,yaw");
      Command c; c.kind=Kind::Seed; c.seed.translation()=Eigen::Vector3d(initial[0],initial[1],initial[2]);
      c.seed.linear()=(Eigen::AngleAxisd(initial[5],Eigen::Vector3d::UnitZ())*
          Eigen::AngleAxisd(initial[4],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(initial[3],Eigen::Vector3d::UnitX())).toRotationMatrix();
      enqueue(std::move(c),"initializing");
    }
    worker_ = std::thread([this] {run();});
  }
MapLocalizer::~MapLocalizer() {
    {std::lock_guard<std::mutex> lock(mutex_); stopping_=true;}
    condition_.notify_all(); if (worker_.joinable()) worker_.join();
  }

}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<dog_slam::MapLocalizer>());}
  catch (const std::exception &e) {RCLCPP_ERROR(rclcpp::get_logger("map_localizer"),"%s",e.what()); rclcpp::shutdown(); return 1;}
  rclcpp::shutdown(); return 0;
}
