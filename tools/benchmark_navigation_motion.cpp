// Offline planar plant audit over a saved 3D terrain map. Never publishes ROS.
// The first-order velocity response and gait disturbance are synthetic; these
// results are not measured robot dynamics or a hardware safety certificate.
#include "dog_navigation/local_planner.hpp"
#include "dog_navigation/path_tracker.hpp"
#include "dog_navigation/braking_sweep.hpp"
#include "dog_navigation/braking_certificate.hpp"
#include "dog_terrain/terrain_graph.hpp"
#include "dog_terrain/graph_search.hpp"
#include "../src/dog_navigation/src/local_planner/collision_overlay.hpp"
#include <chrono>
#include <fstream>
#include <iostream>

using namespace dog_navigation;
using Json=dog_terrain::Json;

int main(int argc,char **argv) {
  if(argc!=4){std::cerr<<"Usage: benchmark_navigation_motion MAP_ROOT MAP_ID CASES.json\n";return 2;}
  const auto graph=dog_terrain::loadProject(argv[1],argv[2]);
  Json cases;std::ifstream input(argv[3]);input>>cases;Json report=Json::array();bool all_passed=true;
  const auto point=[](const Json &p){return Point(p[0].get<double>(),p[1].get<double>(),p[2].get<double>());};
  for(const auto &item:cases){
    LocalPlanner planner(graph);ExecutionOptions options;
    options.walk_speed=.6;options.stair_speed=.38;options.yaw_rate=.5;options.stair_yaw_rate=.38;
    options.startup_linear_speed=.21;options.startup_yaw_rate=.21;
    options.lookahead_distance=item.value("lookahead_distance",options.lookahead_distance);
    options.continuous_turn_angle=item.value("continuous_turn_angle",options.continuous_turn_angle);
    const double response_lag=item.value("response_lag",.2),gait_phase=item.value("gait_phase",0.);
    PathTracker tracker(options);TrackingPose pose;pose.ground=point(item["start"]);
    pose.yaw=item.value("initial_yaw",0.);const Point goal=point(item["goal"]);
    Points obstacles;for(const auto &p:item["obstacles"])obstacles.push_back(point(p));
    const auto &c=graph->config();Options overlay_options;
    CollisionOverlay overlay(obstacles,overlay_options,
      (c.body_length>0?std::hypot(c.body_length,c.body_width)*.5:c.body_radius)+c.footprint_padding,c.body_top,
      c.body_length>0?c.body_length+2*c.footprint_padding:0.,c.body_width>0?c.body_width+2*c.footprint_padding:0.);
    Route route;std::vector<int> installed;Velocity actual;
    double next_plan=0,first_walk=-1,zero_after_walk=0,linear_zero_after_walk=0,alignment_after_walk=0,max_plan_ms=0,max_control_ms=0,max_error=0;
    size_t guard_stops=0,command_guard_stops=0,feedback_guard_stops=0,control_deadline_stops=0,budget_stops=0,acceleration_violations=0;bool done=false;
    Json samples=Json::array();std::string failure;Velocity previous;
    const auto terrain=[&](Point &p){
      double best=std::numeric_limits<double>::infinity(),height=p.z();
      for(size_t i=1;i<route.points.size();++i){
        const Point a=route.points[i-1],d=route.points[i]-a;
        const double n=d.head<2>().squaredNorm();if(n<1e-12)continue;
        const double t=std::clamp((p-a).head<2>().dot(d.head<2>())/n,0.,1.);
        const Point q=a+t*d;const double distance=(p-q).squaredNorm();
        if(distance<best){best=distance;height=q.z();}
      }
      p.z()=height;
    };
    const auto braking_clear=[&](const Velocity &velocity){
      Point a=pose.ground;double yaw=pose.yaw;
      for(const auto &sample:brakingSweep(velocity,.15,.5)){
        Point b=pose.ground+Point(std::cos(pose.yaw)*sample.point.x()-std::sin(pose.yaw)*sample.point.y(),
          std::sin(pose.yaw)*sample.point.x()+std::cos(pose.yaw)*sample.point.y(),0);
        b.z()=a.z();if(sample.point.head<2>().norm()>1e-9)terrain(b);
        const double next_yaw=pose.yaw+sample.yaw;
        if(!graph->motionClear(a,b,yaw,next_yaw)||!overlay.motionClear(a,b,yaw,next_yaw))return false;
        a=b;yaw=next_yaw;
      }
      return true;
    };
    constexpr double dt=.05;
    for(int step=0;step<1200;++step){
      const double t=step*dt;
      if(t>=next_plan){
        const auto begin=std::chrono::steady_clock::now();
        try{route=planner.plan(pose.ground,goal,obstacles,Pose::Identity(),[&]{
          if(std::chrono::steady_clock::now()-begin>std::chrono::milliseconds(400))throw dog_terrain::SearchInterrupted();
        });}catch(const dog_terrain::SearchInterrupted &){route.valid=false;++budget_stops;}
        // Executor and guard consume the planner's classified packet, not the
        // raw observation input. Reusing raw points here would incorrectly
        // reintroduce static-map returns as newly appeared obstacles.
        if(route.valid)overlay=CollisionOverlay(route.obstacles_map,overlay_options,
          (c.body_length>0?std::hypot(c.body_length,c.body_width)*.5:c.body_radius)+c.footprint_padding,c.body_top,
          c.body_length>0?c.body_length+2*c.footprint_padding:0.,c.body_width>0?c.body_width+2*c.footprint_padding:0.);
        max_plan_ms=std::max(max_plan_ms,std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count());
        next_plan=t+.2;
        if(route.valid&&route.ids!=installed){tracker.setRoute(route.points,route.stairs,pose,goal,0,false);installed=route.ids;}
        if(budget_stops>=10){failure="repeated_planning_budget_exceeded";break;}
      }
      TrackResult out;
      const auto control_begin=std::chrono::steady_clock::now();
      if(route.valid)out=tracker.update(pose,dt,[&](const Velocity &command){
        return certifyTrackingBraking(*graph,pose.ground,pose.yaw,
          command,route.points,.15,.5,
          [&](const Point &a,const Point &b,double first,double last){return overlay.motionClear(a,b,first,last);});
      });else{tracker.stop();out.phase="waiting_route";}
      const double control_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-control_begin).count();
      max_control_ms=std::max(max_control_ms,control_ms);
      max_error=std::max(max_error,out.error);
      if(out.fault){failure=out.reason;break;}
      if(out.done){done=true;break;}
      Velocity command=out.command;
      if(control_ms>=100){command={};++control_deadline_stops;}
      // Match the executor's immediate actual/predicted footprint gate as
      // well as the independent guard's command/measured braking envelopes.
      if(route.valid&&!graph->motionClear(pose.ground,out.predicted,pose.yaw,out.predicted_yaw)){
        tracker.stop();command={};out.phase="blocked_footprint";++guard_stops;
      }
      if(route.valid){
        if(!braking_clear(command)){command={};++guard_stops;++command_guard_stops;}
        else if(!braking_clear(pose.measured)){command={};++guard_stops;++feedback_guard_stops;}
      }
      if(guard_stops>=200){failure="repeated_braking_rejection";break;}
      if(std::hypot(command.x,command.y)>0&&first_walk<0)first_walk=t;
      if(first_walk>=0&&out.phase!="settling"&&out.phase!="goal_align"){
        if(speed(command)<1e-9)zero_after_walk+=dt;
        if(std::hypot(command.x,command.y)<1e-9)linear_zero_after_walk+=dt;
        if(out.phase=="align")alignment_after_walk+=dt;
      }
      if(std::hypot(command.x,command.y)>0&&std::hypot(previous.x,previous.y)>0&&
         std::hypot(command.x-previous.x,command.y-previous.y)>options.acceleration*dt+1e-9)
        ++acceleration_violations;
      previous=command;
      // Synthetic lag and small lateral gait disturbance, not bag feedback.
      actual.x+=(command.x-actual.x)*dt/(response_lag+dt);
      actual.y+=(command.y-actual.y)*dt/(response_lag+dt);
      const double gait=speed(actual)>.025?.015*std::sin(8*t+gait_phase):0.;
      actual.yaw+=(command.yaw-actual.yaw)*dt/(response_lag+dt);
      samples.push_back({{"t",t},{"vx",command.x},{"vy",command.y},{"wz",command.yaw},
        {"feedback_vx",actual.x},{"feedback_vy",actual.y+gait},{"feedback_wz",actual.yaw},{"yaw",pose.yaw},{"phase",out.phase},
        {"xyz",{pose.ground.x(),pose.ground.y(),pose.ground.z()}},{"tracking_error",out.error}});
      pose.ground+=Point(std::cos(pose.yaw)*actual.x-std::sin(pose.yaw)*(actual.y+gait),
                         std::sin(pose.yaw)*actual.x+std::cos(pose.yaw)*(actual.y+gait),0)*dt;
      terrain(pose.ground);pose.yaw+=actual.yaw*dt;pose.measured=actual;pose.measured.y+=gait;
    }
    if(!done&&failure.empty())failure="simulation_timeout";
    const bool smooth=done&&linear_zero_after_walk<=.25+1e-9&&acceleration_violations==0&&
      command_guard_stops==0&&control_deadline_stops==0;
    all_passed=all_passed&&smooth;
    std::cerr<<item["name"]<<": reached="<<done<<", failure="<<failure
      <<", in_route_zero="<<zero_after_walk<<", alignment="<<alignment_after_walk<<'\n';
    report.push_back({{"case",item["name"]},{"goal_reached",done},{"failure",failure},
      {"duration",samples.size()*dt},{"first_walk",first_walk},{"zero_after_walk",zero_after_walk},
      {"linear_zero_after_walk",linear_zero_after_walk},{"smooth",smooth},
      {"alignment_after_walk",alignment_after_walk},{"guard_stops",guard_stops},{"planning_budget_stops",budget_stops},
      {"command_guard_stops",command_guard_stops},{"feedback_guard_stops",feedback_guard_stops},
      {"control_deadline_stops",control_deadline_stops},
      {"max_plan_ms",max_plan_ms},{"max_control_ms",max_control_ms},{"max_tracking_error",max_error},{"acceleration_violations",acceleration_violations},
      {"map_revision",graph->revision()},{"response_lag",response_lag},{"gait_phase",gait_phase},
      {"raw_obstacles",obstacles.size()},{"classified_obstacles",route.obstacles_map.size()},
      {"lookahead_distance",options.lookahead_distance},{"continuous_turn_angle",options.continuous_turn_angle},
      {"physical_robot_verified",false},{"samples",samples}});
  }
  std::cout<<report.dump()<<'\n';
  return all_passed?0:1;
}
