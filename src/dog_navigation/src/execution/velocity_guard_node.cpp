#include "velocity_guard_node.hpp"
#include "dog_navigation/navigation_motion_settings.hpp"

namespace dog_navigation {
bool VelocityGuard::fresh(const builtin_interfaces::msg::Time &stamp,
                          double maximum) const {
  const double age = now().seconds() - rclcpp::Time(stamp).seconds();
  return age >= -.05 && age <= maximum;
}
VelocityGuard::VelocityGuard() : Node("velocity_guard") {
  const double frequency = declare_parameter<double>("guard_frequency", 50.);
  if (!std::isfinite(frequency) || frequency <= 0 || 1. / frequency >= .15)
    throw std::invalid_argument("guard_frequency must be finite with period < 0.15 s (braking reaction horizon)");
  const bool requested = declare_parameter<bool>("allow_autonomous", false);
  const bool calibrated =
      declare_parameter<bool>("calibration_confirmed", false);
  offset_ = declare_parameter<double>("base_ground_offset", .40);
  const auto root = canonicalMapPath(
      declare_parameter<std::string>("map_root", "~/.local/share/dog/maps"));
  const auto motion_limits=NavigationMotionSettings(root/".navigation-motion.json").load();
  input_.autonomous_limits={motion_limits.walk_speed,autonomous_lateral_speed,motion_limits.yaw_rate};
  braking_deceleration_=std::min(.15,motion_limits.acceleration);
  yaw_braking_deceleration_=std::min(.5,motion_limits.yaw_acceleration);
  const auto id = declare_parameter<std::string>("map_id", "");
  if (!id.empty()) {
    project_ = std::filesystem::weakly_canonical(dog_terrain::mapPath(root, id))
                   .string();
    graph_ = dog_terrain::loadProject(root, id);
    revision_ = graph_->revision();
    localization_ = std::make_unique<LocalizationGate>(project_, .8);
  }
  if (!std::isfinite(offset_) || offset_ <= 0)
    throw std::invalid_argument("Invalid calibrated ground offset");
  auto_enabled_ = requested && calibrated && bool(graph_);
  input_.auto_enabled = auto_enabled_;
  buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
  output_ = create_publisher<geometry_msgs::msg::Twist>("/dog/safe_cmd_vel", 1);
  status_ = create_publisher<std_msgs::msg::String>(
      "~/status", rclcpp::QoS(1).transient_local());
  manual_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 1, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        arbiter_.manualTakeover(input_.task_id);
        input_.manual = {m->linear.x, m->linear.y, m->angular.z};
        input_.manual_deadline = guardWall() + .35;
      });
  command_ = create_subscription<dog_interfaces::msg::NavigationCommand>(
      "/navigation/command", 1,
      [this](dog_interfaces::msg::NavigationCommand::ConstSharedPtr m) {
        command(m);
      });
  robot_sub_ = create_subscription<dog_interfaces::msg::RobotStatus>(
      "/dog/status", 1,
      [this](dog_interfaces::msg::RobotStatus::ConstSharedPtr m) {
        robot_ = *m;
        status_at_ = fresh(m->header.stamp, .5) ? guardWall() : 0;
      });
  feedback_ = create_subscription<nav_msgs::msg::Odometry>(
      "/dog/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        measured_ = {m->twist.twist.linear.x, m->twist.twist.linear.y,
                     m->twist.twist.angular.z};
        feedback_at_ =
            fresh(m->header.stamp, .3) && std::isfinite(speed(measured_))
                ? guardWall()
                : 0;
      });
  localization_sub_ = create_subscription<std_msgs::msg::String>(
      "/map_localizer/status", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        if (localization_)
          localization_->updateJson(m->data, std::chrono::steady_clock::now());
      });
  executor_status_sub_=create_subscription<std_msgs::msg::String>(
      "/navigation_executor/status",rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg){
        try{
          const auto status=dog_terrain::Json::parse(msg->data);
          if(status.at("map_project")!=project_||status.at("map_revision")!=revision_)return;
          const auto limits=NavigationMotionSettings::apply(status.at("motion_limits"));
          input_.autonomous_limits={limits.walk_speed,autonomous_lateral_speed,limits.yaw_rate};
          // Never make the braking envelope shorter after receiving a profile.
          // The task match below also prevents a newly started executor from
          // inheriting stale limits from a preceding process/task.
          braking_deceleration_=std::min(braking_deceleration_,limits.acceleration);
          yaw_braking_deceleration_=std::min(yaw_braking_deceleration_,limits.yaw_acceleration);
          executor_limits_task_=status.at("task_id").get<std::string>();executor_limits_at_=guardWall();
        }catch(const std::exception &){executor_limits_at_=0;}
      });
  route_sub_ = create_subscription<dog_interfaces::msg::TerrainRoute>(
      "/local_navigation/terrain_route", 1,
      [this](dog_interfaces::msg::TerrainRoute::ConstSharedPtr m) {
        const auto previous=route_;const double previous_deadline=route_deadline_;
        const double previous_handoff_deadline=handoff_deadline_;
        handoff_deadline_=0;
        route_deadline_ = 0;
        route_.reset();
        if (!graph_ || !m->valid || m->header.frame_id != "map" ||
            m->map_revision != revision_ || m->map_project != project_ ||
            !fresh(m->header.stamp, .6) || m->points.empty() ||
            m->points.size() > 5000 ||
            m->stairs.size() + 1 != m->points.size() ||
            m->node_ids.size() != m->points.size() ||
            m->obstacles.size() > 120000 ||
            !std::isfinite(m->valid_for_seconds))
          return;
        for (size_t i = 0; i < m->points.size(); ++i) {
          const auto id = m->node_ids[i];
          const auto &p = m->points[i];
          if (id < 0 || size_t(id) >= graph_->nodes().size() || !Point(p.x,p.y,p.z).allFinite()||
              (graph_->nodes()[id].point - Point(p.x, p.y, p.z)).norm() > 1e-5)
            return;
        }
        for (const auto &p : m->obstacles)
          if (!Point(p.x, p.y, p.z).allFinite())
            return;
        route_ = m;
        const double age =
            now().seconds() - rclcpp::Time(m->header.stamp).seconds();
        route_deadline_ =
            guardWall() +
            std::max(0., std::min(.6, m->valid_for_seconds) - age);
        // Permit only the already-issued, still-live command during route
        // handoff. Collision checks use the NEW route and its NEW obstacles.
        // Invalid routes, changed goals and expired commands never carry over.
        if(previous&&previous->has_goal&&m->has_goal&&previous->goal_stamp==m->goal_stamp&&
           previous->goal==m->goal){
          if(previous->generation!=m->generation&&previous->generation==last_command_.route_generation){
            handoff_generation_=previous->generation;
            handoff_deadline_=std::min(previous_deadline,input_.command_deadline);
          }else if(previous->generation==m->generation){
            // An obstacle/lease refresh cannot extend the original handoff.
            handoff_deadline_=previous_handoff_deadline;
          }
        }
      });
  estop_ = create_service<std_srvs::srv::SetBool>(
      "~/estop", [this](std::shared_ptr<std_srvs::srv::SetBool::Request> q,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> r) {
        if (q->data) {
          input_.estop = true;
          arbiter_.manualTakeover(input_.task_id);
          r->success = true;
        } else {
          r->success =
              guardWall() - feedback_at_ < .3 && speed(measured_) < .025;
          if (r->success)
            input_.estop = false;
        }
        r->message = input_.estop
                         ? "Stop latched; clearing requires measured standstill"
                         : "Stop cleared; old task remains revoked";
      });
  cancel_=create_service<std_srvs::srv::Trigger>("~/cancel",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> r){
        arbiter_.manualTakeover(input_.task_id.empty()?executor_limits_task_:input_.task_id);
        input_.command_deadline=0;input_.autonomous={};input_.manual={};input_.manual_deadline=0;
        route_.reset();route_deadline_=0;
        handoff_deadline_=0;
        output_->publish(geometry_msgs::msg::Twist());
        r->success=true;r->message="Autonomous task revoked; zero velocity published";
      });
  timer_ = create_wall_timer(std::chrono::duration<double>(1. / frequency), [this] { tick(); });
}
void VelocityGuard::command(
    dog_interfaces::msg::NavigationCommand::ConstSharedPtr m) {
  // Reordered packets cannot revoke or renew the current command lease.
  if (m->task_id == input_.task_id && m->sequence <= sequence_)return;
  input_.command_deadline = 0;
  if (m->header.frame_id != "base_link" || !fresh(m->header.stamp, .15) ||
      m->map_revision != revision_ || !std::isfinite(m->valid_for_seconds) ||
      m->valid_for_seconds < 0 || m->valid_for_seconds > .15)
    return;
  input_.task_id = m->task_id;
  sequence_ = m->sequence;
  last_command_ = *m;
  input_.autonomous = {m->velocity.linear.x, m->velocity.linear.y,
                       m->velocity.angular.z};
  if (m->phase.find("stair") != std::string::npos &&
      (input_.autonomous.x < 0 || std::abs(input_.autonomous.y) > 1e-6))
    return;
  if (m->permit) {
    const double age =
        now().seconds() - rclcpp::Time(m->header.stamp).seconds();
    input_.command_deadline =
        guardWall() + std::max(0., m->valid_for_seconds - age);
  }
}
} // namespace dog_navigation
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<dog_navigation::VelocityGuard>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("velocity_guard"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
}
