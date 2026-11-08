#include "local_navigation_node.hpp"
#include "../common/navigation_ros.hpp"
#include "dog_terrain/terrain_graph.hpp"
#include <nlohmann/json.hpp>

namespace dog_navigation {
using Json=nlohmann::json;
nav_msgs::msg::Path LocalNavigation::path(const Points &points,const builtin_interfaces::msg::Time &stamp) {
    nav_msgs::msg::Path msg;msg.header.frame_id=map_frame_;msg.header.stamp=stamp;
    for(const auto &p:points) {
      geometry_msgs::msg::PoseStamped pose;pose.header=msg.header;pose.pose.position.x=p.x();
      pose.pose.position.y=p.y();pose.pose.position.z=p.z();pose.pose.orientation.w=1.;msg.poses.push_back(pose);
    }return msg;
  }
void LocalNavigation::status(const std::string &reason,bool valid,const Route &route) {
    std_msgs::msg::Bool stop;stop.data=!valid;stop_pub_->publish(stop);
    Json steps=Json::array();for(size_t i=0;i<route.stairs.size();++i)if(route.stairs[i])steps.push_back(i);
    Json value={{"state",reason},{"valid",valid},{"planning_only",true},{"execution_enabled",false},
      {"stop_required",!valid},{"map_project",project_.string()},{"map_revision",graph_->revision()},
      {"scan_stamp",memory_->stamp()},{"scan_age",memory_->stamp()>0?now().seconds()-memory_->stamp():-1.},
      {"valid_for_seconds",valid?std::max(0.,std::min({timeout_-(now().seconds()-memory_->stamp()),
          gate_->remaining(Clock::now()),tf_expiry_-now().seconds()})):0.},
      {"route_stamp",{{"sec",route_stamp_.sec},{"nanosec",route_stamp_.nanosec}}},
      {"observed_voxels",memory_->size()},{"novel_obstacles",route.obstacle_count},
      {"route_generation",generation_},{"global_points",route.points.size()},{"local_points",route.local.size()},
      {"stairs_edges",steps},{"last_processing_ms",duration_ms_},{"observation_scope","bounded_local_window"}};
    const auto performance=planner_->performance();
    value["planner_timing_ms"]={{"total",performance.total_ms},{"classification",performance.classification_ms},
      {"search",performance.search_ms},{"certification",performance.certification_ms}};
    value["static_search_cache"]={{"turn_evaluations",performance.turn_evaluations},
      {"turn_hits",performance.turn_cache_hits},{"turn_entries",performance.turn_cache_entries},
      {"lower_bound_builds",performance.lower_bound_builds}};
    value["frame_edge_queries"]={{"evaluated",performance.dynamic_edge_evaluations},{"hits",performance.dynamic_edge_cache_hits}};
    value["goal_stamp"]={{"sec",goal_stamp_.sec},{"nanosec",goal_stamp_.nanosec}};
    std_msgs::msg::String msg;msg.data=value.dump();status_pub_->publish(msg);
    dog_interfaces::msg::TerrainRoute packet;packet.header.frame_id=map_frame_;packet.header.stamp=now();
    packet.map_project=project_.string();packet.map_revision=graph_->revision();packet.generation=generation_;
    packet.valid=valid;packet.reason=reason;packet.valid_for_seconds=value["valid_for_seconds"].get<double>();
    packet.has_goal=bool(goal_);packet.goal_stamp=goal_stamp_;if(goal_){packet.goal.x=goal_->x();packet.goal.y=goal_->y();packet.goal.z=goal_->z();}
    packet.node_ids.assign(route.ids.begin(),route.ids.end());packet.stairs=route.stairs;
    for(const auto &p:route.points){geometry_msgs::msg::Point point;point.x=p.x();point.y=p.y();point.z=p.z();packet.points.push_back(point);}
    for(const auto &p:route.obstacles_map){geometry_msgs::msg::Point point;point.x=p.x();point.y=p.y();point.z=p.z();packet.obstacles.push_back(point);}
    route_pub_->publish(packet);
  }
void LocalNavigation::invalidate(const std::string &reason,const Route &route) {
    route_generation_valid_=false;
    const auto empty=path({},now());path_pub_->publish(empty);local_pub_->publish(empty);status(reason,false,route);
  }
}  // namespace dog_navigation
