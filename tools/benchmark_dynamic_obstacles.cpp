// Read-only saved-map regression: introduce a box on a cached route, then remove it.
#include "dog_navigation/local_planner.hpp"
#include "dog_terrain/terrain_graph.hpp"
#include "dog_terrain/graph_search.hpp"
#include "../src/dog_navigation/src/local_planner/collision_overlay.hpp"
#include <chrono>
#include <iostream>

int main(int argc,char **argv) {
  using namespace dog_navigation;
  if(argc!=9){std::cerr<<"Usage: benchmark_dynamic_obstacles ROOT ID sx sy sz gx gy gz\n";return 2;}
  const auto graph=dog_terrain::loadProject(argv[1],argv[2]);
  const Point start(std::stod(argv[3]),std::stod(argv[4]),std::stod(argv[5]));
  const Point goal(std::stod(argv[6]),std::stod(argv[7]),std::stod(argv[8]));
  LocalPlanner planner(graph);const auto baseline=planner.plan(start,goal,{},Pose::Identity());
  if(!baseline.valid){std::cerr<<baseline.reason<<'\n';return 1;}
  dog_terrain::Json report=dog_terrain::Json::array();
  for(size_t i=1;i+1<baseline.points.size();++i){
    const Point center=baseline.points[i];Points box;
    for(double x=-.15;x<=.151;x+=.1)for(double y=-.15;y<=.151;y+=.1)
      for(double z=.2;z<=.501;z+=.1)box.push_back(center+Point(x,y,z));
    for(int refresh=0;refresh<3;++refresh){
      const auto begin=std::chrono::steady_clock::now();std::string reason;Route route;
      try{route=planner.plan(start,goal,box,Pose::Identity(),[&]{
        if(std::chrono::steady_clock::now()-begin>std::chrono::milliseconds(400))
          throw dog_terrain::SearchInterrupted();
      });reason=route.reason;}catch(const dog_terrain::SearchInterrupted &){reason="planning_budget_exceeded";}
      const auto p=planner.performance();
      bool reference_valid=false;std::string reference_reason;
      if(!route.valid&&refresh==0){
        const auto &c=graph->config();Options options;
        CollisionOverlay overlay(box,options,std::hypot(c.body_length,c.body_width)*.5+c.footprint_padding,
          c.body_top,c.body_length+2*c.footprint_padding,c.body_width+2*c.footprint_padding);
        dog_terrain::GraphSearchWorkspace workspace(*graph);
        const auto reference_begin=std::chrono::steady_clock::now();
        try{
          dog_terrain::searchGraph(*graph,graph->snap(start),graph->snap(goal),
            [&](int a,const dog_terrain::Edge &e){
              const double risk=overlay.edgeRisk(graph->nodes()[a].point,graph->nodes()[e.target].point);
              return risk<0?std::numeric_limits<double>::infinity():e.cost*(1+options.avoidance_weight*risk);
            },&workspace,[&]{
              if(std::chrono::steady_clock::now()-reference_begin>std::chrono::seconds(3))
                throw dog_terrain::SearchInterrupted();
            },[&](int a,int b,int c){
              if(a<0)return true;
              const Point u=graph->nodes()[b].point-graph->nodes()[a].point;
              const Point v=graph->nodes()[c].point-graph->nodes()[b].point;
              return overlay.motionClear(graph->nodes()[b].point,graph->nodes()[b].point,
                std::atan2(u.y(),u.x()),std::atan2(v.y(),v.x()));
            },2.);
          reference_valid=true;
        }catch(const std::exception &e){reference_reason=e.what();}
      }
      report.push_back({{"box",{center.x(),center.y(),center.z()}},{"refresh",refresh},
        {"valid",route.valid},{"reason",reason},{"total_ms",p.total_ms},{"search_ms",p.search_ms},
        {"edges",p.dynamic_edge_evaluations},{"points",route.points.size()},
        {"reference_checked",!route.valid&&refresh==0},
        {"reference_valid",reference_valid},{"reference_reason",reference_reason},
        {"physical_robot_verified",false}});
    }
    const auto restored=planner.plan(start,goal,{},Pose::Identity());
    report.push_back({{"phase","obstacle_removed"},{"valid",restored.valid},
      {"reason",restored.reason},{"physical_robot_verified",false}});
  }
  std::cout<<report.dump(2)<<'\n';
}
