#include "itera_lio/backend/pose_graph_optimizer.hpp"
#include "itera_lio/backend/backend_profile.hpp"
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <chrono>
using namespace itera_lio;
namespace {
std::vector<std::vector<std::string>> readCsv(const std::string& path){
 std::ifstream f(path);if(!f)throw std::runtime_error("cannot read graph input");std::string line;std::getline(f,line);std::vector<std::vector<std::string>> out;
 while(std::getline(f,line)){if(line.empty())continue;std::istringstream stream(line);std::vector<std::string> fields;std::string token;while(std::getline(stream,token,','))fields.push_back(token);out.push_back(std::move(fields));}return out;
}
common::Pose pose(const std::vector<std::string>& row,std::size_t start){
 const Eigen::Vector3d p(std::stod(row.at(start)),std::stod(row.at(start+1)),std::stod(row.at(start+2)));
 const Eigen::Quaterniond q(std::stod(row.at(start+6)),std::stod(row.at(start+3)),std::stod(row.at(start+4)),std::stod(row.at(start+5)));
 return {q.toRotationMatrix(),p};
}
}
int main(int argc,char** argv){try{
 if(argc!=5)throw std::runtime_error("graph_replay RAW.csv CONSTRAINTS.csv OUTDIR REPETITIONS (6-DoF only)");
 const std::filesystem::path out(argv[3]);if(std::filesystem::exists(out))throw std::runtime_error("output exists");std::filesystem::create_directories(out);
 std::vector<backend::Keyframe> raw;
 for(const auto& row:readCsv(argv[1])){if(row.size()!=9)throw std::runtime_error("raw schema mismatch");raw.push_back({std::stoull(row[0]),std::stod(row[1]),pose(row,2)});}
 std::vector<backend::PoseGraphConstraint> constraints;
 for(const auto& row:readCsv(argv[2])){if(row.size()!=46)throw std::runtime_error("constraint schema mismatch");backend::PoseGraphConstraint c;c.from_id=std::stoull(row[0]);c.to_id=std::stoull(row[1]);c.type=static_cast<backend::ConstraintType>(std::stoi(row[2]));c.T_Ia_Ib=pose(row,3);for(int i=0;i<6;++i)for(int j=0;j<6;++j)c.information(i,j)=std::stod(row[10+i*6+j]);constraints.push_back(c);}
 backend::PoseGraphOptimizerOptions options;options.max_iterations=10;options.active_window_size=0;
 backend::PoseGraphOptimizer optimizer(options);
 std::ofstream metrics(out/"samples.csv"),stages(out/"stages.csv");metrics<<"repetition,success,converged,iterations,initial_cost,final_cost,symbolic_factorizations,wall_ms\n"<<std::setprecision(17);stages<<"repetition,stage,ms,calls,work\n"<<std::setprecision(17);
 const int repetitions=std::stoi(argv[4]);std::vector<backend::Keyframe> nodes;
 for(int rep=0;rep<=repetitions;++rep){nodes=raw;b007::reset();backend::PoseGraphOptimizationReport report;auto start=std::chrono::steady_clock::now();const bool ok=optimizer.optimize(nodes,constraints,report);const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
  metrics<<rep<<','<<ok<<','<<report.converged<<','<<report.iterations<<','<<report.initial_cost<<','<<report.final_cost<<','<<report.symbolic_factorizations<<','<<ms<<'\n';
  for(const auto& [key,e]:b007::entries)stages<<rep<<','<<key<<','<<e.ms<<','<<e.calls<<','<<e.work<<'\n';
  if(!ok)throw std::runtime_error("graph optimization failed");
 }
 std::ofstream poses(out/"final_nodes.csv");poses<<"id,px,py,pz,qx,qy,qz,qw\n"<<std::setprecision(17);
 // B015 is explicitly CSV-only, including diagnostic graph replays.
 for(const auto& node:nodes){const Eigen::Quaterniond q(node.pose_WI.R_WI);const auto& p=node.pose_WI.p_WI;poses<<node.id<<','<<p.x()<<','<<p.y()<<','<<p.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<'\n';}
 std::cout<<"nodes="<<raw.size()<<" constraints="<<constraints.size()<<"\n";
 return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
