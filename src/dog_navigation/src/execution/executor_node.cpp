#include "executor_node.hpp"
#include "dog_navigation/navigation_motion_settings.hpp"
#include <iomanip>
#include <sstream>

namespace dog_navigation {
double monotonicSeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
void NavigationExecutor::configure() {
  enabled_ = declare_parameter<bool>("enabled", false);
  ground_offset_ = declare_parameter<double>("base_ground_offset", .40);
  auto param = [this](const char *key, double &value) {
    value = declare_parameter<double>(key, value);
  };
  param("controller_frequency", controller_frequency_);
  if (!std::isfinite(controller_frequency_) || controller_frequency_ <= 0 ||
      1. / controller_frequency_ >= .15)
    throw std::invalid_argument("controller_frequency must be finite with period < 0.15 s (command lease)");
  param("walk_speed", options_.walk_speed);
  param("stair_speed", options_.stair_speed);
  param("acceleration", options_.acceleration);
  param("yaw_rate", options_.yaw_rate);
  param("yaw_acceleration", options_.yaw_acceleration);
  param("stair_yaw_rate", options_.stair_yaw_rate);
  param("startup_linear_speed",options_.startup_linear_speed);
  param("startup_yaw_rate",options_.startup_yaw_rate);
  param("goal_xy_tolerance", options_.goal_xy);
  param("goal_z_tolerance", options_.goal_z);
  param("max_tracking_error", options_.max_tracking_error);
  param("lookahead_distance", options_.lookahead_distance);
  param("continuous_turn_angle", options_.continuous_turn_angle);
  DecisionOptions decision;
  param("recovery_timeout", decision.recovery_timeout);
  param("progress_timeout", decision.progress_timeout);
  param("retry_interval", decision.retry_interval);
  retry_interval_=decision.retry_interval;
  decision.max_retries = declare_parameter<int>("max_retries", decision.max_retries);
  supervisor_ = TaskSupervisor(decision);
  options_.validate();
  if (!std::isfinite(ground_offset_) || ground_offset_ <= 0)
    throw std::invalid_argument("Invalid calibrated body offset");
  const auto root = canonicalMapPath(
      declare_parameter<std::string>("map_root", "~/.local/share/dog/maps"));
  options_=NavigationMotionSettings(root/".navigation-motion.json").load(options_);
  options_.validate();
  for(const auto *key:NavigationMotionSettings::keys)
    set_parameter(rclcpp::Parameter(key,NavigationMotionSettings::values(options_).at(key).get<double>()));
  const auto id = declare_parameter<std::string>("map_id", "full_map");
  project_ = std::filesystem::weakly_canonical(dog_terrain::mapPath(root, id))
                 .string();
  graph_ = dog_terrain::loadProject(root, id);
  revision_ = graph_->revision();
  localization_ = std::make_unique<LocalizationGate>(project_, .8);
  tracker_ = std::make_unique<PathTracker>(options_);
}
NavigationExecutor::NavigationExecutor() : Node("navigation_executor") {
  configure();
  buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/navigation/planner_goal", 1);
  command_pub_ = create_publisher<dog_interfaces::msg::NavigationCommand>(
      "/navigation/command", 1);
  trajectory_pub_ =
      create_publisher<dog_interfaces::msg::TimedTerrainTrajectory>(
          "~/trajectory", rclcpp::QoS(1).transient_local());
  status_pub_ = create_publisher<std_msgs::msg::String>(
      "~/status", rclcpp::QoS(1).transient_local());
  planner_cancel_ =
      create_client<std_srvs::srv::Trigger>("/local_navigation/cancel");
  relocalize_ =
      create_client<std_srvs::srv::Trigger>("/map_localizer/global_relocalize");
  walk_ = create_client<dog_interfaces::srv::SetMotion>("/dog/set_motion");
  route_sub_ = create_subscription<dog_interfaces::msg::TerrainRoute>(
      "/local_navigation/terrain_route", 1,
      [this](dog_interfaces::msg::TerrainRoute::ConstSharedPtr m) {
        receiveRoute(m);
      });
  loc_sub_ = create_subscription<std_msgs::msg::String>(
      "/map_localizer/status", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        localization_->updateJson(m->data, std::chrono::steady_clock::now());
      });
  robot_sub_ = create_subscription<dog_interfaces::msg::RobotStatus>(
      "/dog/status", 1,
      [this](dog_interfaces::msg::RobotStatus::ConstSharedPtr m) {
        robot_ = *m;
        robot_at_ = monotonicSeconds();
      });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/dog/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        const double age =
            now().seconds() - rclcpp::Time(m->header.stamp).seconds();
        if (age < -.05 || age > .3)
          return;
        measured_ = {m->twist.twist.linear.x, m->twist.twist.linear.y,
                     m->twist.twist.angular.z};
        feedback_at_ = monotonicSeconds();
      });
  guard_sub_ = create_subscription<std_msgs::msg::String>(
      "/velocity_guard/status", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        try {
          const auto j = dog_terrain::Json::parse(m->data);
          guard_reason_ = j.value("reason", "");
          if(handle_&&guard_reason_!="permitted"&&guard_reason_!="autonomous_command_expired")last_guard_block_reason_=guard_reason_;
          guard_task_id_ = j.value("task_id", "");
          guard_auto_enabled_ = j.value("auto_enabled", false);
          guard_at_ = monotonicSeconds();
        } catch (const std::exception &) {
          guard_at_ = 0;
        }
      });
  server_ = rclcpp_action::create_server<Action>(
      this, "/navigate_terrain",
      [this](const rclcpp_action::GoalUUID &,
             std::shared_ptr<const Action::Goal> g) {
        const auto &p = g->goal.pose.position;
        // Web goals carry their creation stamp. A submission already in
        // flight when cancel is handled must not become a new active task.
        const auto stamp=rclcpp::Time(g->goal.header.stamp).nanoseconds();
        if(stamp>0&&stamp<=cancelled_before_ns_)return rclcpp_action::GoalResponse::REJECT;
        Eigen::Quaterniond q(
            g->goal.pose.orientation.w, g->goal.pose.orientation.x,
            g->goal.pose.orientation.y, g->goal.pose.orientation.z);
        if (!enabled_ || handle_ || g->goal.header.frame_id != "map" ||
            !Point(p.x, p.y, p.z).allFinite() ||
            (g->use_goal_yaw &&
             (!q.coeffs().allFinite() || std::abs(q.norm() - 1) > .001)))
          return rclcpp_action::GoalResponse::REJECT;
        try {
          const auto id = graph_->snap({p.x, p.y, p.z});
          const auto delta = graph_->nodes()[id].point - Point(p.x, p.y, p.z);
          if (delta.head<2>().norm() > options_.goal_xy ||
              std::abs(delta.z()) > options_.goal_z)
            return rclcpp_action::GoalResponse::REJECT;
        } catch (const std::exception &) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<Handle>) {
        cancelled_before_ns_=now().nanoseconds();
        supervisor_.cancel();
        tracker_->stop();route_.reset();route_deadline_=0;
        TrackResult stopped;stopped.phase="cancelled";stopped.reason="cancelled_by_user";
        publishCommand(stopped,false);publishState(stopped);
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<Handle> h) {
        handle_ = h;
        goal_ = h->get_goal()->goal;
        use_yaw_ = h->get_goal()->use_goal_yaw;
        std::ostringstream id;
        for (auto b : h->get_goal_id())
          id << std::hex << std::setw(2) << std::setfill('0') << unsigned(b);
        task_id_ = id.str();
        last_block_reason_.clear();last_guard_block_reason_.clear();
        tracker_ = std::make_unique<PathTracker>(options_);
        correction_.reset();
        supervisor_.start(monotonicSeconds());
        route_.reset();
        accepted_generation_ = 0;
        installed_ids_.clear();
        submitPlannerGoal();
      });
  pause_ = create_service<std_srvs::srv::Trigger>(
      "~/pause", [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
        r->success = bool(handle_);
        if (handle_)
          supervisor_.pause();
        r->message = "Task paused; motion permit revoked";
      });
  resume_ = create_service<std_srvs::srv::Trigger>(
      "~/resume", [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
        r->success = supervisor_.paused();
        if (r->success) {
          supervisor_.resume(monotonicSeconds());
          route_.reset();
          tracker_ = std::make_unique<PathTracker>(options_);
          submitPlannerGoal();
        }
      });
  cancel_ = create_service<std_srvs::srv::Trigger>(
      "~/cancel", [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
        cancelled_before_ns_=now().nanoseconds();
        supervisor_.cancel();
        finish(false,"cancelled_by_user",true);
        r->success=true;r->message="Task cancelled; motion permit revoked and planner cancellation requested";
      });
  last_tick_ = monotonicSeconds();
  timer_ = create_wall_timer(std::chrono::duration<double>(1. / controller_frequency_), [this] { tick(); });
}
} // namespace dog_navigation
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<dog_navigation::NavigationExecutor>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("navigation_executor"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
}
