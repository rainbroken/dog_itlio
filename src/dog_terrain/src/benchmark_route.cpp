#include "dog_terrain/terrain_graph.hpp"
#include "dog_terrain/graph_search.hpp"
#include <chrono>
#include <iostream>

// Read-only benchmark: keep map loading separate from cold/warm graph search.
int main(int argc,char **argv) {
  if(argc!=9) {
    std::cerr<<"Usage: benchmark_route MAP_ROOT MAP_ID sx sy sz gx gy gz\n";return 1;
  }
  try {
    using Clock=std::chrono::steady_clock;
    const auto begin=Clock::now();
    auto graph=dog_terrain::loadProject(argv[1],argv[2]);
    const double load=std::chrono::duration<double>(Clock::now()-begin).count();
    const Eigen::Vector3d start(std::stod(argv[3]),std::stod(argv[4]),std::stod(argv[5]));
    const Eigen::Vector3d goal(std::stod(argv[6]),std::stod(argv[7]),std::stod(argv[8]));
    dog_terrain::GraphSearchWorkspace workspace(*graph);
    dog_terrain::Json runs=dog_terrain::Json::array();
    for(int i=0;i<3;++i) {
      const auto t=Clock::now();const auto before=workspace.counters();
      const auto route=dog_terrain::searchGraph(*graph,graph->snap(start),graph->snap(goal),{},&workspace);
      const double seconds=std::chrono::duration<double>(Clock::now()-t).count();
      const auto after=workspace.counters();
      runs.push_back({{"seconds",seconds},{"cost",route.cost},{"node_ids",route.ids},
        {"turn_evaluations",after.turn_evaluations-before.turn_evaluations},
        {"turn_cache_hits",after.turn_cache_hits-before.turn_cache_hits}});
    }
    const auto t=Clock::now();const auto certified=graph->plan(start,goal);
    std::cout<<dog_terrain::Json({{"map_id",argv[2]},{"revision",graph->revision()},
      {"load_seconds",load},{"search_runs",runs},{"certified_route",certified},
      {"cold_plan_seconds",std::chrono::duration<double>(Clock::now()-t).count()}}).dump(2)<<'\n';
    return 0;
  }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
