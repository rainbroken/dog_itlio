// Copyright 2026 rainbroken
// SPDX-License-Identifier: Apache-2.0
#include "itera_lio/loop/loop_pair_archive.hpp"
#include <algorithm>
#include <Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include "itera_lio/loop/gicp_profile.hpp"
#include <cstdlib>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
int main(int argc,char**argv) {
  try {
    if(argc<4)throw std::runtime_error("usage: loop_pair_replay PAIR_ROOT OUTPUT.csv ENGINE [ENGINE...]");
    const std::filesystem::path root(argv[1]),output(argv[2]);
    if(std::filesystem::exists(output))throw std::runtime_error("output exists; choose a new attempt");
    std::vector<std::filesystem::path> pairs;
    for(const auto& entry:std::filesystem::directory_iterator(root))
      if(entry.is_directory()&&std::filesystem::is_regular_file(entry.path()/"metadata.txt"))pairs.push_back(entry.path());
    std::sort(pairs.begin(),pairs.end());if(pairs.empty())throw std::runtime_error("no complete archived pairs");
    std::vector<std::string> engines(argv+3,argv+argc);
    std::ofstream stream(output);if(!stream)throw std::runtime_error("cannot open result CSV");
    stream<<"pair,engine,current_id,matched_id,accepted,reason,fitness,inliers,reciprocal_overlap,condition_number,px,py,pz,qx,qy,qz,qw,wall_ms\n"<<std::setprecision(17);
    std::ofstream profile(output.string()+".profile.csv");
    profile<<"pair,engine,stage,ms,calls,work\n"<<std::setprecision(17);
    std::ofstream traces(output.string()+".hypotheses.csv");
    traces<<"pair,engine,initialization,converged,iterations,reason,px,py,pz,qx,qy,qz,qw\n"<<std::setprecision(17);
    const char* artifact_root=std::getenv("ITERA_REPLAY_ARTIFACT_DIR");
    b005::capture_poses=true;
    std::size_t number=0;
    for(const auto& path:pairs) {
      auto input=itera_lio::loop::loadLoopPair(path);
      if(number++%2)std::reverse(engines.begin(),engines.end());
      for(const auto& engine:engines) {
        input.options.registration_engine=engine;
        itera_lio::loop::LoopVerifier verifier(input.options);
        b005::reset();
        const auto start=std::chrono::steady_clock::now();
        const auto result=verifier.verify(input.candidate,input.current,input.matched,input.submap);
        const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        b005::dump(profile,path.filename().string(),engine);
        for(const auto& trace:b005::poses){
          const Eigen::Quaterniond tq(trace.pose.R_WI);
          traces<<path.filename().string()<<','<<engine<<','<<trace.initialization<<','<<trace.converged<<','<<trace.iterations<<','<<trace.reason<<','<<trace.pose.p_WI.x()<<','<<trace.pose.p_WI.y()<<','<<trace.pose.p_WI.z()<<','<<tq.x()<<','<<tq.y()<<','<<tq.z()<<','<<tq.w()<<'\n';
          if(artifact_root){
            const auto dest=std::filesystem::path(artifact_root)/path.filename()/engine/(trace.initialization+".pcd");
            if(std::filesystem::exists(dest))throw std::runtime_error("PCD exists; no overwrite");
            std::filesystem::create_directories(dest.parent_path());
            pcl::PointCloud<pcl::PointXYZ> aligned_cloud;aligned_cloud.reserve(input.current.points_I.size());
            for(const auto& point:input.current.points_I){const Eigen::Vector3d x=trace.pose.R_WI*point.position.cast<double>()+trace.pose.p_WI;aligned_cloud.emplace_back(x.x(),x.y(),x.z());}
            if(pcl::io::savePCDFileBinary(dest.string(),aligned_cloud)!=0)throw std::runtime_error("PCD save failed");
          }
        }
        const Eigen::Quaterniond q(result.T_Ia_Ib.R_WI);
        stream<<path.filename().string()<<','<<engine<<','<<result.current_id<<','<<result.matched_id<<','<<result.isValid()<<','
          <<result.rejection_reason<<','<<result.fitness<<','<<result.inliers<<','<<result.reciprocal_inlier_ratio<<','<<result.geometry_condition_number<<','
          <<result.T_Ia_Ib.p_WI.x()<<','<<result.T_Ia_Ib.p_WI.y()<<','<<result.T_Ia_Ib.p_WI.z()<<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w()<<','<<ms<<'\n';
      }
    }
    stream.close();if(!stream)throw std::runtime_error("failed to finalize result CSV");
    std::cout<<"replayed_pairs="<<pairs.size()<<'\n';return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
