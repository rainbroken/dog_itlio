#include "dog_navigation/path_tracker.hpp"
#include <algorithm>
#include <stdexcept>

namespace dog_navigation {
PathTracker::PathTracker(ExecutionOptions options) : options_(options) {
  options_.validate();
}
void PathTracker::stop() {
  previous_ = {};
  settled_ = 0;
  entered_ = false;
  aligning_ = true;
  rotation_started_ = false;
  entry_settled_ = 0;
}
void PathTracker::setRoute(Points path, std::vector<bool> stairs,
                           const TrackingPose &pose, Point goal, double yaw,
                           bool use_yaw) {
  if (path.empty() || stairs.size() + 1 != path.size() || !goal.allFinite() ||
      !std::isfinite(yaw))
    throw std::invalid_argument("Invalid tracker route");
  for (const auto &p : path)
    if (!p.allFinite())
      throw std::invalid_argument("Nonfinite tracker route");
  // Only remove exactly collinear intermediate points with identical terrain
  // flags. This preserves the original 3D segments and every actual corner.
  if(options_.startup_linear_speed>0&&path.size()>2){
    Points continuous{path.front()};std::vector<bool> flags;
    for(size_t i=1;i<path.size();++i){
      if(i+1<path.size()&&stairs[i-1]==stairs[i]&&straightContinuation(continuous.back(),path[i],path[i+1]))continue;
      continuous.push_back(path[i]);flags.push_back(stairs[i-1]);
    }
    path=std::move(continuous);stairs=std::move(flags);
  }
  // Associate the actual pose with the new route first. A shared endpoint can
  // acquire a different incoming edge when the snapped start moves; retaining
  // its index blindly can put the robot behind that edge and falsely abort.
  size_t target = path.size() > 1 ? 1 : 0;
  double best = std::numeric_limits<double>::infinity();
  const auto segmentDistance = [&](size_t i) {
    const Point d = path[i] - path[i - 1];
    if (d.squaredNorm() < 1e-12) return std::numeric_limits<double>::infinity();
    const double t = std::clamp((pose.ground - path[i - 1]).dot(d) / d.squaredNorm(), 0., 1.);
    return (pose.ground - path[i - 1] - t * d).norm();
  };
  if (path.size() > 1) {
    for (size_t i = 1; i < path.size(); ++i) {
      const double distance = segmentDistance(i);
      if (distance < best) {
        best = distance;
        target = i;
      }
    }
  }
  if (!path_.empty() && target_ < path_.size()) {
    const auto old = path_[target_];
    for (size_t i = 1; i < path.size(); ++i)
      if ((path[i] - old).norm() < 1e-6 && segmentDistance(i) <= best + .02) {
        target = i;
        break;
      }
  }
  path_ = std::move(path);
  stairs_ = std::move(stairs);
  target_ = target;
  goal_ = goal;
  goal_yaw_ = yaw;
  use_yaw_ = use_yaw;
}
bool PathTracker::continuousAfter(size_t endpoint) const {
  if(options_.lookahead_distance<=0||endpoint==0||endpoint+1>=path_.size()||
     stairs_[endpoint-1]||stairs_[endpoint])return false;
  const Point a=path_[endpoint]-path_[endpoint-1],b=path_[endpoint+1]-path_[endpoint];
  if(a.head<2>().norm()<1e-6||b.head<2>().norm()<1e-6)return false;
  return std::abs(angleError(std::atan2(b.y(),b.x()),std::atan2(a.y(),a.x())))<options_.continuous_turn_angle;
}
bool PathTracker::readyToRotate(const TrackingPose &pose) {
  // Confirm the transition once. Gait-induced lateral velocity noise during
  // an established in-place turn must not switch rotation off every frame.
  if(!rotation_started_&&std::hypot(previous_.x,previous_.y)<=options_.stopped_speed&&
     std::hypot(pose.measured.x,pose.measured.y)<=options_.stopped_speed)
    rotation_started_=true;
  return rotation_started_;
}
TrackResult PathTracker::update(const TrackingPose &pose, double dt,
                                const std::function<bool(const Velocity &)> &command_clear) {
  TrackResult out;
  double heading_command=0,lateral_command=0;
  bool allow_lateral_recovery=false,need_velocity_search=false;
  std::vector<std::pair<Velocity,bool>> certificates;
  const auto checked=[&](const Velocity &v){
    for(const auto &entry:certificates)
      if(entry.first.x==v.x&&entry.first.y==v.y&&entry.first.yaw==v.yaw)return entry.second;
    const bool result=command_clear(v);
    certificates.emplace_back(v,result);return result;
  };
  out.predicted = pose.ground;
  out.predicted_yaw = pose.yaw;
  if (!pose.ground.allFinite() || !std::isfinite(pose.yaw) || !std::isfinite(speed(pose.measured)) ||
      !std::isfinite(dt) || dt <= 0 || dt > .25) {
    out.fault = true;
    out.reason = "invalid_control_state_or_period";
    stop();
    return out;
  }
  if (path_.empty())
    return out;
  if (!entered_) {
    out.phase =
        !stairs_.empty() && stairs_[0] ? "stair_entry_wait" : "entry_stop";
    entry_settled_ = speed(pose.measured) <= options_.stopped_speed
                         ? entry_settled_ + dt
                         : 0;
    if (entry_settled_ >= options_.settle_time)
      entered_ = true;
    return out;
  }
  const double goal_xy = (pose.ground - goal_).head<2>().norm();
  out.remaining = goal_xy;
  for (size_t i = target_ + 1; i < path_.size(); ++i)
    out.remaining += (path_[i] - path_[i - 1]).head<2>().norm();
  const bool at_goal = goal_xy <= options_.goal_xy &&
                       std::abs(pose.ground.z() - goal_.z()) <= options_.goal_z;
  if (at_goal) {
    if (use_yaw_ &&
        std::abs(angleError(goal_yaw_, pose.yaw)) > options_.goal_yaw) {
      out.phase = "goal_align";
      if (readyToRotate(pose))
        out.command.yaw = std::clamp(2 * angleError(goal_yaw_, pose.yaw),
                                     -options_.yaw_rate, options_.yaw_rate);
    } else {
      out.phase = "settling";
      settled_ = speed(pose.measured) <= options_.stopped_speed &&
                         speed(previous_) <= options_.stopped_speed
                     ? settled_ + dt
                     : 0;
      if (settled_ >= options_.settle_time) {
        out.done = true;
        out.phase = "succeeded";
      }
      if (out.done)
        return out;
    }
  } else {
    if (path_.size() == 1) {
      out.fault = true;
      out.reason = "goal_snap_outside_arrival_tolerance";
      stop();
      return out;
    }
    if (target_ == 0)
      target_ = 1;
    // Advance only through connected, same-terrain gentle bends. Sharp turns
    // and all stair entry/exit boundaries keep the measured-stop handshake.
    while(continuousAfter(target_)){
      const Point a=path_[target_-1],d=path_[target_]-a;
      const double along=(pose.ground-a).head<2>().dot(d.head<2>())/d.head<2>().squaredNorm();
      if(along<1&&(path_[target_]-pose.ground).head<2>().norm()>.04)break;
      ++target_;
    }
    const Point a = path_[target_ - 1], b = path_[target_], delta = b - a;
    const double length = delta.head<2>().norm();
    if (length < 1e-6) {
      out.fault = true;
      out.reason = "vertical_or_duplicate_path_edge";
      stop();
      return out;
    }
    const double along =
        (pose.ground - a).head<2>().dot(delta.head<2>()) / (length * length);
    const Point projected = a + delta * std::clamp(along, 0., 1.);
    out.error = (pose.ground - projected).head<2>().norm();
    if (out.error > options_.max_tracking_error ||
        std::abs(pose.ground.z() - projected.z()) > .25) {
      out.fault = true;
      out.reason = "terrain_tracking_error";
      stop();
      return out;
    }
    const double distance = (b - pose.ground).head<2>().norm();
    if (distance <= options_.waypoint_tolerance || along >= 1) {
      out.phase = "waypoint_stop";
      if (speed(pose.measured) <= options_.stopped_speed) {
        settled_ += dt;
        if (settled_ >= options_.settle_time && target_ + 1 < path_.size()) {
          ++target_;
          aligning_=true;
          rotation_started_=false;
          settled_ = 0;
        }
      } else
        settled_ = 0;
    } else {
      settled_ = 0;
      const bool stair = stairs_[target_ - 1];
      Point carrot=b;
      double stopping_distance=(b-projected).head<2>().norm();
      size_t end=target_;
      while(continuousAfter(end)){
        stopping_distance+=(path_[end+1]-path_[end]).head<2>().norm();++end;
      }
      if(options_.lookahead_distance>0){
        Point cursor=projected;double ahead=options_.lookahead_distance;
        for(size_t i=target_;i<=end;++i){
          const Point d=path_[i]-cursor;const double length=d.head<2>().norm();
          if(length>=ahead&&length>1e-9){carrot=cursor+d*(ahead/length);break;}
          ahead-=length;cursor=path_[i];carrot=cursor;
        }
      }
      const double desired = std::atan2(carrot.y()-pose.ground.y(),carrot.x()-pose.ground.x());
      const double error = angleError(desired, pose.yaw);
      const double yaw_limit =
          stair ? options_.stair_yaw_rate : options_.yaw_rate;
      heading_command=std::clamp(2*error,-yaw_limit,yaw_limit);
      allow_lateral_recovery=!stair&&
        !(target_<stairs_.size()&&stairs_[target_])&&!(target_>1&&stairs_[target_-2]);
      // Rolling replans may trim away a corner we have just crossed. Ordinary
      // ground remains holonomic on its outgoing edge too; otherwise trimming
      // one route node would silently switch back to a forward-only controller.
      const bool holonomic_tracking=command_clear&&allow_lateral_recovery;
      const Point to_carrot=carrot-pose.ground;
      lateral_command=std::clamp(-std::sin(pose.yaw)*to_carrot.x()+std::cos(pose.yaw)*to_carrot.y(),-autonomous_lateral_speed,autonomous_lateral_speed);
      // Hysteresis: small heading noise while walking must not repeatedly
      // alternate translation and a complete stop for alignment.
      if(aligning_)aligning_=std::abs(error)>options_.heading_tolerance;
      else if(std::abs(error)>options_.continuous_turn_angle&&
              !(holonomic_tracking&&std::abs(error)<1.5707963267948966)){
        aligning_=true;rotation_started_=false;
      }
      if (aligning_) {
        out.phase = stair ? "stair_align" : "align";
        if (readyToRotate(pose))
          out.command.yaw = std::clamp(2 * error, -yaw_limit, yaw_limit);
      } else {
        rotation_started_=false;
        out.phase = stair ? (delta.z() > 0 ? "stairs_up" : "stairs_down")
                    : target_ > 1 && stairs_[target_ - 2] ? "stairs_exit"
                                                          : "walk";
        const double limit = stair ? options_.stair_speed : options_.walk_speed;
        const double reserve = options_.acceleration * .20;
        out.command.x = std::min(
            limit, -reserve + std::sqrt(reserve * reserve +
                                        2 * options_.acceleration * stopping_distance));
        // Continue heading feedback during translation to avoid drift and
        // repeated stop/turn cycles at the alignment threshold.
        const Point relative=carrot-pose.ground;
        const double lateral=-std::sin(pose.yaw)*relative.x()+std::cos(pose.yaw)*relative.y();
        const double curvature=2*lateral/std::max(1e-6,relative.head<2>().squaredNorm());
        if(options_.lookahead_distance>0){
          // Regulate before reaching a bend, rather than waiting until the
          // braking guard has to reject a fast command. The yaw-rate ceiling
          // alone does not account for a lagging chassis near tight turns.
          if(!stair)out.command.x=std::min(out.command.x,std::max(options_.startup_linear_speed,
            limit/std::max(1.,2*options_.lookahead_distance*std::abs(curvature))));
          const double turning_limit=yaw_limit/std::max(1e-6,std::abs(curvature));
          if(turning_limit<options_.startup_linear_speed&&std::abs(error)>options_.heading_tolerance&&!holonomic_tracking){
            aligning_=true;out.phase=stair?"stair_align":"align";out.command={};
            if(readyToRotate(pose))out.command.yaw=std::clamp(2 * error,-yaw_limit,yaw_limit);
          }else{
            out.command.x=std::min(out.command.x,holonomic_tracking?
              std::max(options_.startup_linear_speed,turning_limit):turning_limit);
            out.command.yaw=std::clamp(curvature*out.command.x,-yaw_limit,yaw_limit);
          }
        }else out.command.yaw = std::clamp(2 * error, -yaw_limit, yaw_limit);
        if(holonomic_tracking&&out.command.x>0){
          // XY travel is independent of body yaw on this chassis. Keep heading
          // feedback active when translation is regulated down; coupling yaw
          // to forward speed here delays alignment until the narrow passage.
          out.command.yaw=heading_command;
          // Decouple ordinary-ground body heading from travel direction.
          // Follow the certified incoming segment while turning
          // the body, rather than cutting the inside of a tight corner.
          Point travel=delta/length;
          const Point correction=(projected-pose.ground)*1.5;
          travel.x()+=correction.x();travel.y()+=correction.y();
          const double travel_norm=travel.head<2>().norm();
          if(travel_norm>1e-9){
            const double body_x=(std::cos(pose.yaw)*travel.x()+std::sin(pose.yaw)*travel.y())/travel_norm;
            const double body_y=(-std::sin(pose.yaw)*travel.x()+std::cos(pose.yaw)*travel.y())/travel_norm;
            const double planar=std::min(out.command.x,autonomous_lateral_speed/std::max(1e-9,std::abs(body_y)));
            if(body_x>0&&planar>=options_.startup_linear_speed){
              out.command.x=planar*body_x;out.command.y=planar*body_y;
              // Body-frame XY rotates while the chassis velocity responds.
              // For a first-order response, u = v + tau * dv/dt and
              // dv/dt = -yaw_rate * J*v for constant world-frame travel.
              // Compensate this frame rotation before applying certificates
              // and the existing vector acceleration bound.
              // Nominal seconds from the recorded forward response; lateral
              // response is not independently calibrated. Offline plants vary
              // their response time over 0.1--0.3 s.
              constexpr double planar_response_time=.20;
              const double anticipation=planar_response_time*pose.measured.yaw;
              const double x=out.command.x+anticipation*out.command.y;
              const double y=out.command.y-anticipation*out.command.x;
              const double scale=std::min({1.,limit/std::max(1e-9,std::hypot(x,y)),
                autonomous_lateral_speed/std::max(1e-9,std::abs(y))});
              if(x>0&&std::hypot(x,y)*scale>=options_.startup_linear_speed){
                out.command.x=x*scale;out.command.y=y*scale;
              }
              lateral_command=out.command.y;
            }
          }
        }
      }
    }
  }
  if(command_clear&&speed(out.command)>1e-9){
    Velocity desired=out.command;bool found=false;
    const double desired_planar=std::hypot(desired.x,desired.y);
    if(desired_planar>1e-9&&desired_planar<options_.startup_linear_speed){
      desired.x*=options_.startup_linear_speed/desired_planar;desired.y*=options_.startup_linear_speed/desired_planar;
    }
    if(desired_planar<=1e-9&&std::abs(desired.yaw)>1e-9&&std::abs(desired.yaw)<options_.startup_yaw_rate)
      desired.yaw=std::copysign(options_.startup_yaw_rate,desired.yaw);
    const double minimum=desired_planar>1e-9?options_.startup_linear_speed/std::hypot(desired.x,desired.y):
      options_.startup_yaw_rate/std::max(1e-9,std::abs(desired.yaw));
    double rejected=1.;
    for(double scale:{1.,.85,.70,.55,.40,.25,minimum}){
      if(scale>1.+1e-9||scale+1e-9<minimum)continue;
      for(bool keep_yaw:{true,false}){
        Velocity candidate{desired.x*scale,desired.y*scale,desired.yaw*(keep_yaw&&std::abs(desired.x)>1e-9?1.:scale)};
        if(!checked(candidate))continue;
        double low=scale,high=rejected;out.command=candidate;
        // Refine the certified bound so coarse trial spacing does not turn
        // a small necessary deceleration into an avoidable safety stop.
        for(int i=0;i<1&&high-low>.001;++i){
          const double middle=(low+high)*.5;
          candidate={desired.x*middle,desired.y*middle,desired.yaw*(keep_yaw&&std::abs(desired.x)>1e-9?1.:middle)};
          if(checked(candidate)){low=middle;out.command=candidate;}else high=middle;
        }
        found=true;break;
      }
      if(found)break;
      rejected=scale;
    }
    if(!found){
      need_velocity_search=true;out.command=desired;
    }
  }
  auto limited = [dt](double desired, double old, double acceleration) {
    return std::clamp(desired, old - acceleration * dt,
                      old + acceleration * dt);
  };
  const auto executable=[&](double desired,double old,double acceleration,double threshold){
    const double smooth=limited(desired,old,acceleration);
    if(threshold>0&&std::abs(smooth)<threshold)
      return std::abs(desired)>1e-9?std::copysign(threshold,desired):0.;
    return smooth;
  };
  const auto smooth_command=[&](const Velocity &target){
    Velocity candidate;
    const double old_norm=std::hypot(previous_.x,previous_.y),raw_norm=std::hypot(target.x,target.y);
    const double desired_norm=raw_norm>1e-9?
      std::clamp(raw_norm,options_.startup_linear_speed,options_.walk_speed):0.;
    const double desired_x=raw_norm>1e-9?target.x/raw_norm*desired_norm:0.;
    const double desired_y=raw_norm>1e-9?target.y/raw_norm*desired_norm:0.;
    const double change=options_.acceleration*dt;
    if(old_norm<1e-9&&desired_norm>0&&options_.startup_linear_speed>0){
      const double start=std::min(desired_norm,std::max(options_.startup_linear_speed,change));
      candidate.x=desired_x/desired_norm*start;candidate.y=desired_y/desired_norm*start;
    }else{
      // Project the complete velocity difference onto the reachable disk.
      // Giving all acceleration to speed magnitude first would prevent any
      // steering until acceleration/deceleration had finished.
      const double dx=desired_x-previous_.x,dy=desired_y-previous_.y;
      const double scale=std::min(1.,change/std::max(1e-12,std::hypot(dx,dy)));
      candidate.x=previous_.x+dx*scale;candidate.y=previous_.y+dy*scale;
    }
    double magnitude=std::hypot(candidate.x,candidate.y);
    if(magnitude<options_.startup_linear_speed&&old_norm>1e-9){
      if(desired_norm==0){candidate.x=0;candidate.y=0;magnitude=0;}
      else{
        // The inner dead-zone disk is forbidden during motion. When the direct
        // projection enters it, use the intersection with the minimum-speed
        // circle, still inside the same acceleration disk.
        magnitude=options_.startup_linear_speed;
        const double radial=magnitude-old_norm;
        const double remaining=std::sqrt(std::max(0.,change*change-radial*radial));
        const double angle=2*std::asin(std::min(1.,remaining/(2*std::sqrt(magnitude*old_norm))));
        const double old_direction=std::atan2(previous_.y,previous_.x);
        const double direction=old_direction+std::clamp(angleError(std::atan2(desired_y,desired_x),old_direction),-angle,angle);
        candidate.x=magnitude*std::cos(direction);candidate.y=magnitude*std::sin(direction);
      }
    }
    candidate.yaw=executable(target.yaw,previous_.yaw,options_.yaw_acceleration,
      magnitude>1e-9?0.:options_.startup_yaw_rate);
    return candidate;
  };
  const Velocity target=out.command;
  out.command=smooth_command(target);
  // BPX's startup threshold applies to activation. Translation already above
  // its threshold permits smooth, small yaw corrections (as in dog_ws).
  if(command_clear&&(need_velocity_search||std::abs(out.command.y)>autonomous_lateral_speed+1e-9||
      (speed(out.command)>1e-9&&!checked(out.command)))){
    // Search the next reachable command, not only a distant speed target.
    // Independent yaw samples can tighten a safe turn while reducing forward
    // speed. Every accepted sample still obeys the same braking certificate.
    std::vector<Velocity> tested;double best=std::numeric_limits<double>::infinity();Velocity selected;
    const double correction_speed=std::max(options_.startup_linear_speed,std::hypot(previous_.x,previous_.y));
    const double recovery_y=std::min(autonomous_lateral_speed,correction_speed);
    for(double y:{target.y,0.,lateral_command,recovery_y,-recovery_y}){
     if(y!=0&&(!allow_lateral_recovery||(std::isfinite(best)&&std::hypot(selected.x,selected.y)>1e-9)))break;
     std::vector<std::pair<double,Velocity>> candidates;
     const double corrected_x=std::sqrt(std::max(0.,correction_speed*correction_speed-y*y));
     for(double x:{target.x,previous_.x,corrected_x,0.})for(double yaw:{target.yaw,heading_command,previous_.yaw,0.}){
      const Velocity candidate=smooth_command({x,y,yaw});
      if(std::abs(candidate.y)>autonomous_lateral_speed+1e-9)continue;
      const double planar=std::hypot(candidate.x,candidate.y);
      if(planar==0&&std::abs(candidate.yaw)>1e-9&&!readyToRotate(pose))continue;
      if(std::any_of(tested.begin(),tested.end(),[&](const auto &v){return std::abs(v.x-candidate.x)<1e-9&&std::abs(v.y-candidate.y)<1e-9&&std::abs(v.yaw-candidate.yaw)<1e-9;}))continue;
      tested.push_back(candidate);
      const double score=std::abs(target.x-candidate.x)+.3*std::abs(target.yaw-candidate.yaw)+
        .5*std::abs(lateral_command-candidate.y)+(need_velocity_search&&speed(candidate)<1e-9?.5:0.);
      candidates.emplace_back(score,candidate);
     }
     // Testing in cost order returns exactly the same minimum-cost command
     // without spending collision queries on candidates that cannot improve
     // the current best. Stable ties preserve the original selection order.
     std::stable_sort(candidates.begin(),candidates.end(),[](const auto &a,const auto &b){return a.first<b.first;});
     for(const auto &candidate:candidates){
      if(candidate.first>=best)break;
      if(checked(candidate.second)){best=candidate.first;selected=candidate.second;break;}
     }
    }
    if(!std::isfinite(best)){
      out.command={};previous_={};out.phase="waiting_safe_velocity";out.reason="no_executable_braking_velocity";return out;
    }
    out.command=selected;
    if(std::hypot(out.command.x,out.command.y)==0&&std::abs(out.command.yaw)>1e-9){aligning_=true;out.phase="align";}
  }
  previous_ = out.command;
  out.predicted.x() += (out.command.x*std::cos(pose.yaw)-out.command.y*std::sin(pose.yaw))*dt;
  out.predicted.y() += (out.command.x*std::sin(pose.yaw)+out.command.y*std::cos(pose.yaw))*dt;
  if (target_ > 0) {
    const Point a = path_[target_ - 1], d = path_[target_] - a;
    const double horizontal = d.head<2>().squaredNorm();
    if (horizontal > 1e-12)
      out.predicted.z() =
          a.z() +
          d.z() * std::clamp((out.predicted - a).head<2>().dot(d.head<2>()) /
                                 horizontal,
                             0., 1.);
  }
  out.predicted_yaw += out.command.yaw * dt;
  return out;
}
} // namespace dog_navigation
