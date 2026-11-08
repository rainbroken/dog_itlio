#include "itera_lio/backend/backend_manager.hpp"
#include <Eigen/Geometry>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
using namespace itera_lio;
namespace {
std::vector<std::vector<std::string>> readCsv(const char* path) {
  std::ifstream f(path); if (!f) throw std::runtime_error("input missing");
  std::string line; std::getline(f,line); std::vector<std::vector<std::string>> out;
  while (std::getline(f,line)) { std::istringstream s(line); std::string v;
    std::vector<std::string> r; while(std::getline(s,v,',')) r.push_back(v);
    if(!r.empty()) out.push_back(std::move(r)); } return out;
}
common::Pose pose(const std::vector<std::string>& r, size_t s) {
  return {Eigen::Quaterniond(std::stod(r.at(s+6)),std::stod(r.at(s+3)),
    std::stod(r.at(s+4)),std::stod(r.at(s+5))).toRotationMatrix(),
    Eigen::Vector3d(std::stod(r.at(s)),std::stod(r.at(s+1)),std::stod(r.at(s+2)))};
}
}
int main(int argc,char** argv) {
 try {
  if(argc!=4) throw std::runtime_error("RAW CONSTRAINTS LAG");
  auto raw=readCsv(argv[1]); std::vector<backend::PoseGraphConstraint> loops;
  for(auto& r:readCsv(argv[2])) { if(r.at(2)!="1") continue;
    backend::PoseGraphConstraint c; c.from_id=std::stoull(r[0]);c.to_id=std::stoull(r[1]);
    c.type=backend::ConstraintType::kLoop;c.T_Ia_Ib=pose(r,3);
    for(int i=0;i<6;++i)for(int j=0;j<6;++j)c.information(i,j)=std::stod(r.at(10+6*i+j));
    loops.push_back(c);
  }
  std::stable_sort(loops.begin(),loops.end(),[](auto& a,auto& b){return a.from_id<b.from_id;});
  backend::BackendOptions o;o.enabled=true;
  o.keyframes.min_translation=0;o.keyframes.min_rotation_rad=0;
  backend::BackendManager b(o);size_t next=0;const size_t lag=std::stoull(argv[3]);
  std::cout<<"lag,from,to,node_count,success,iterations,initial_cost,final_cost\n"<<std::setprecision(17);
  for(size_t id=0;id<raw.size();++id) {
    auto u=b.addOdometryKeyframe(std::stod(raw[id].at(1)),pose(raw[id],2));
    if(!u.keyframe||u.keyframe->id!=id)throw std::runtime_error("keyframe mismatch");
    while(next<loops.size() && id>=std::min(loops[next].from_id+lag,raw.size()-1)) {
      auto& c=loops[next++];auto result=b.addLoopConstraint(c.from_id,c.to_id,c.T_Ia_Ib,c.information);
      std::cout<<lag<<','<<c.from_id<<','<<c.to_id<<','<<id+1<<','<<bool(result)<<',';
      if(result)std::cout<<result->optimization.iterations<<','<<result->optimization.initial_cost<<','<<result->optimization.final_cost;
      else std::cout<<"0,0,0";std::cout<<'\n';
    }
  }
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
