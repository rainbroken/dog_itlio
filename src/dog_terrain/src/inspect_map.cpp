#include "dog_terrain/terrain_graph.hpp"
#include <fstream>
#include <iostream>

// Read-only audit: input is an array of pairs of node IDs in this exact revision.
int main(int argc,char **argv) {
  if(argc!=4){std::cerr<<"Usage: inspect_map MAP_ROOT MAP_ID NODE_PAIRS_JSON\n";return 1;}
  try {
    auto g=dog_terrain::loadProject(argv[1],argv[2]);
    std::ifstream input(argv[3]);const auto pairs=dog_terrain::Json::parse(input);
    dog_terrain::Json result=dog_terrain::Json::array();
    for(const auto &pair:pairs) {
      const int a=pair.at(0).get<int>(),b=pair.at(1).get<int>();
      auto r=g->inspectConnection(g->nodes().at(a).point,g->nodes().at(b).point);
      r["nodes"]=pair;result.push_back(r);
    }
    std::cout<<result.dump(2)<<'\n';return 0;
  }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
