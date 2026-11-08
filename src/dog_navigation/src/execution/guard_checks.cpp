#include "../common/navigation_ros.hpp"
#include "dog_terrain/collision_geometry.hpp"
#include "dog_navigation/braking_sweep.hpp"
#include "velocity_guard_node.hpp"

namespace dog_navigation {
bool VelocityGuard::collisionFree(const Velocity &v) {
  collision_check_expired_=false;
  if (!graph_ || !route_)
    return false;
  const auto map = buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
  const auto body =
      buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
  if (!fresh(map.header.stamp, .3) || !fresh(body.header.stamp, .3))
    return false;
  const Pose map_body = transform(map.transform) * transform(body.transform);
  const Point a = map_body * Point(0, 0, -offset_);
  const double yaw =
      std::atan2(map_body.linear()(1, 0), map_body.linear()(0, 0));
  const auto &c = graph_->config();
  const double radius =
      (c.body_length > 0 ? std::hypot(c.body_length, c.body_width) * .5
                         : c.body_radius) +
      c.footprint_padding + .12 + .10 * .866025404;
  const double padding=c.footprint_padding+.12+.10*.866025404;
  const double begin=guardWall();
  const auto terrain=[&](Point &p){
    double best=std::numeric_limits<double>::infinity();double height=p.z();
    for(size_t i=1;i<route_->points.size();++i){
      const auto &u=route_->points[i-1],&v=route_->points[i];
      const Point first(u.x,u.y,u.z),delta(v.x-u.x,v.y-u.y,v.z-u.z);
      const double n=delta.head<2>().squaredNorm();if(n<1e-12)continue;
      const double t=std::clamp((p-first).head<2>().dot(delta.head<2>())/n,0.,1.);
      const Point projected=first+t*delta;const double distance=(p-projected).squaredNorm();
      if(distance<best){best=distance;height=projected.z();}
    }
    p.z()=height;return best;
  };
  const auto clear=[&](const Velocity &candidate){
    if(std::hypot(candidate.x,candidate.y)>1e-9){Point origin=a;if(terrain(origin)>.25*.25)return false;}
    const auto sweep=brakingSweep(candidate,braking_deceleration_,yaw_braking_deceleration_);
    Point previous=a;double previous_yaw=yaw;
    for(size_t i=1;i<sweep.size();++i){
      if(guardWall()-begin>.07){collision_check_expired_=true;return false;}
      const auto &sample=sweep[i];
      Point next=a+Point(std::cos(yaw)*sample.point.x()-std::sin(yaw)*sample.point.y(),
                         std::sin(yaw)*sample.point.x()+std::cos(yaw)*sample.point.y(),0);
      next.z()=previous.z();if(sample.point.head<2>().norm()>1e-9)terrain(next);
      const double next_yaw=yaw+sample.yaw;
      if(!graph_->motionClear(previous,next,previous_yaw,next_yaw))return false;
      for(const auto &p:route_->obstacles)
        if(c.body_length>0?
           bool(dog_terrain::sweptTurningBoxContact(previous,next,Point(p.x,p.y,p.z),previous_yaw,next_yaw,
             c.body_length*.5+padding,c.body_width*.5+padding,.12-.05,c.body_top+.12+.05)):
           bool(dog_terrain::sweptCylinderContact(previous,next,Point(p.x,p.y,p.z),radius,.12-.05,c.body_top+.12+.05)))return false;
      previous=next;previous_yaw=next_yaw;
    }
    return true;
  };
  if(!clear(v))return false;
  // Check measured momentum independently, including when the desired
  // translation has already reached zero or its turn direction has changed.
  if(std::abs(v.x-measured_.x)>1e-6||std::abs(v.y-measured_.y)>1e-6||std::abs(v.yaw-measured_.yaw)>1e-6)
    return clear(measured_);
  return true;
}
void VelocityGuard::tick() {
  const double wall = guardWall();
  input_.robot_ready = wall - status_at_ < .5 && robot_.connected &&
                       robot_.state_fresh && robot_.current_motion_state == 6 &&
                       robot_.current_gait == 0 &&
                       robot_.safety_state != robot_.SAFETY_FAULT;
  input_.feedback_fresh = wall - feedback_at_ < .3;
  input_.localization_ready =
      localization_ && localization_->ready(std::chrono::steady_clock::now());
  input_.route_ready = route_ && wall < route_deadline_ &&
                       (route_->generation == last_command_.route_generation||
                        (wall<handoff_deadline_&&last_command_.route_generation==handoff_generation_));
  auto result = arbiter_.update(input_, wall);
  if(result.owner=="autonomous"&&(wall-executor_limits_at_>.3||executor_limits_task_!=last_command_.task_id)){
    result.command={};result.owner="none";result.reason="executor_motion_limits_stale";
  }
  try {
    if (result.owner == "autonomous" && (speed(result.command) > 1e-9||speed(measured_)>.025) &&
        !collisionFree(result.command)) {
      result.command = {};
      result.owner = "none";
      result.reason = collision_check_expired_?"safety_processing_or_command_expired":"braking_or_footprint_collision";
    }
  } catch (const std::exception &) {
    result.command = {};
    result.owner = "none";
    result.reason = "safety_tf_unavailable";
  }
  if (guardWall() - wall > .08 ||
      (guardWall() > input_.command_deadline && result.owner == "autonomous")) {
    result.command = {};
    result.owner = "none";
    result.reason = "safety_processing_or_command_expired";
  }
  geometry_msgs::msg::Twist command;
  command.linear.x = result.command.x;
  command.linear.y = result.command.y;
  command.angular.z = result.command.yaw;
  output_->publish(command);
  std_msgs::msg::String state;
  state.data =
      dog_terrain::Json({{"owner", result.owner},
                         {"reason", result.reason},
                         {"auto_enabled", auto_enabled_},
                         {"max_linear_speed",input_.autonomous_limits.x},
                         {"max_yaw_rate",input_.autonomous_limits.yaw},
                         {"braking_deceleration",braking_deceleration_},
                         {"yaw_braking_deceleration",yaw_braking_deceleration_},
                         {"estop", input_.estop},
                         {"task_id", input_.task_id},
                         {"map_revision", revision_},
                         {"measured_speed", speed(measured_)},
                         {"processing_ms", (guardWall() - wall) * 1000}})
          .dump();
  status_->publish(state);
}
} // namespace dog_navigation
