#include "local_navigation_node.hpp"
#include "dog_terrain/terrain_graph.hpp"

namespace dog_navigation {
void LocalNavigation::configureParameters() {
    auto param=[this](const char *name,auto &v){v=declare_parameter<std::decay_t<decltype(v)>>(name,v);};
    param("voxel",options_.voxel);param("range",options_.range);param("vertical_range",options_.vertical_range);
    param("min_range",options_.min_range);param("padding",options_.padding);param("obstacle_bottom",options_.obstacle_bottom);
    param("static_tolerance",options_.static_tolerance);param("avoidance_distance",options_.avoidance_distance);
    param("avoidance_weight",options_.avoidance_weight);param("local_length",options_.local_length);
    param("max_rays",options_.max_rays);param("max_voxels",options_.max_voxels);options_.validate();
    param("data_timeout",timeout_);param("localization_timeout",localization_timeout_);
    param("tf_timeout",tf_timeout_);param("processing_budget",budget_);param("planning_rate",rate_);
    param("base_ground_offset",ground_offset_);param("odom_jump_tolerance",jump_);param("max_odom_speed",speed_);
    for(double v:{timeout_,localization_timeout_,tf_timeout_,budget_,rate_,ground_offset_,jump_,speed_})
      if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Invalid planner timing/body configuration");
    map_frame_=declare_parameter<std::string>("map_frame","map");
    odom_frame_=declare_parameter<std::string>("odom_frame","odom");
    base_frame_=declare_parameter<std::string>("base_frame","base_link");
    sensor_frame_=declare_parameter<std::string>("sensor_frame","livox_frame");
    if(map_frame_!="map"||odom_frame_.empty()||base_frame_.empty()||sensor_frame_.empty()||
        map_frame_==odom_frame_||map_frame_==base_frame_||odom_frame_==base_frame_)
      throw std::invalid_argument("Terrain projects require map frame; odom/base frames must be distinct");
    const auto root=canonicalMapPath(declare_parameter<std::string>("map_root","~/.local/share/dog/maps"));
    const auto id=declare_parameter<std::string>("map_id","full_map");
    project_=std::filesystem::weakly_canonical(dog_terrain::mapPath(root,id));
    graph_=dog_terrain::loadProject(root,id);planner_=std::make_unique<LocalPlanner>(graph_,options_);
    memory_=std::make_unique<ObstacleMemory>(options_);
    gate_=std::make_unique<LocalizationGate>(project_,localization_timeout_);
}
}  // namespace dog_navigation
