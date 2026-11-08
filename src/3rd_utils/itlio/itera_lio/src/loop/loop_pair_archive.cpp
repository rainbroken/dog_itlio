// Copyright 2026 rainbroken
// SPDX-License-Identifier: Apache-2.0
#include "itera_lio/loop/loop_pair_archive.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <iomanip>
#include <stdexcept>
namespace itera_lio::loop {
namespace {
template<class Visitor> void visitOptions(LoopVerifierOptions & o,Visitor f) {
#define FIELD(name) f(#name,o.name)
  FIELD(registration_engine);FIELD(min_points);FIELD(min_inliers);FIELD(max_fitness);
  FIELD(max_translation_jump);FIELD(max_rotation_rad);FIELD(max_correspondence_distance);
  FIELD(inlier_distance);FIELD(gicp_voxel_size);FIELD(robust_kernel_scale);FIELD(num_threads);
  FIELD(min_inlier_ratio);FIELD(min_reciprocal_inlier_ratio);FIELD(min_geometry_eigenvalue_ratio);
  FIELD(max_normal_eigenvalue_ratio);FIELD(require_planar_normal_agreement);FIELD(local_refinement);
  FIELD(source_voxel_size);FIELD(target_voxel_size);FIELD(max_gravity_tilt_correction_rad);
  FIELD(max_vertical_correction);FIELD(normal_neighbors);FIELD(max_iterations);
  FIELD(max_source_points);FIELD(max_target_points);
#undef FIELD
}
void writeCloud(const std::filesystem::path & p,const std::vector<common::MapPoint>& points) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for(const auto& value:points) {
    pcl::PointXYZI point;point.x=value.position.x();point.y=value.position.y();point.z=value.position.z();point.intensity=value.intensity;
    cloud.push_back(point);
  }
  if(pcl::io::savePCDFileBinary(p.string(),cloud)!=0)throw std::runtime_error("failed to write loop-pair cloud");
}
std::vector<common::MapPoint> readCloud(const std::filesystem::path & p) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  if(pcl::io::loadPCDFile(p.string(),cloud)!=0)throw std::runtime_error("failed to read loop-pair cloud");
  std::vector<common::MapPoint> output;output.reserve(cloud.size());
  for(const auto& point:cloud)output.push_back({{point.x,point.y,point.z},point.intensity});
  return output;
}
void writePose(std::ostream& stream,const common::Pose& pose) {
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)stream<<pose.R_WI(i,j)<<' ';
  for(int i=0;i<3;++i)stream<<pose.p_WI[i]<<' ';
  stream<<'\n';
}
void readPose(std::istream& stream,common::Pose& pose) {
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)stream>>pose.R_WI(i,j);
  for(int i=0;i<3;++i)stream>>pose.p_WI[i];
}
}
void saveLoopPair(const std::filesystem::path& directory,const LoopPairInput& input) {
  std::filesystem::create_directories(directory.parent_path());
  if(!std::filesystem::create_directory(directory))throw std::runtime_error("loop-pair directory already exists");
  writeCloud(directory/"source.pcd",input.current.points_I);
  writeCloud(directory/"matched.pcd",input.matched.points_I);
  writeCloud(directory/"target.pcd",input.submap);
  std::ofstream options(directory/"options.txt");options<<std::setprecision(17);
  auto copied=input.options;
  visitOptions(copied,[&](const char* name,auto& value){options<<name<<' '<<value<<'\n';});
  options.close();if(!options)throw std::runtime_error("failed to write loop-pair options");
  std::ofstream meta(directory/"metadata.tmp");meta<<std::setprecision(17)<<"ITLIO_LOOP_PAIR 1\n";
  meta<<input.current.keyframe.id<<' '<<input.matched.keyframe.id<<' '
      <<input.current.keyframe.timestamp<<' '<<input.matched.keyframe.timestamp<<'\n';
  writePose(meta,input.current.keyframe.pose_WI);writePose(meta,input.matched.keyframe.pose_WI);
  const auto& c=input.candidate;
  meta<<c.current_id<<' '<<c.matched_id<<' '<<c.descriptor_distance<<' '<<c.yaw_offset_rad<<' '
      <<c.odometry_distance<<' '<<c.odometry_vertical_distance<<' '<<c.gravity_angle_rad<<'\n';
  meta.close();if(!meta)throw std::runtime_error("failed to write loop-pair metadata");
  std::filesystem::rename(directory/"metadata.tmp",directory/"metadata.txt");
}
LoopPairInput loadLoopPair(const std::filesystem::path& directory) {
  LoopPairInput input;std::ifstream meta(directory/"metadata.txt");std::string magic;int version=0;
  meta>>magic>>version;if(magic!="ITLIO_LOOP_PAIR"||version!=1)throw std::runtime_error("invalid loop-pair format");
  meta>>input.current.keyframe.id>>input.matched.keyframe.id>>input.current.keyframe.timestamp>>input.matched.keyframe.timestamp;
  readPose(meta,input.current.keyframe.pose_WI);readPose(meta,input.matched.keyframe.pose_WI);
  auto& c=input.candidate;
  meta>>c.current_id>>c.matched_id>>c.descriptor_distance>>c.yaw_offset_rad>>c.odometry_distance>>c.odometry_vertical_distance>>c.gravity_angle_rad;
  if(!meta)throw std::runtime_error("truncated loop-pair metadata");
  std::ifstream options(directory/"options.txt");
  visitOptions(input.options,[&](const char* name,auto& value){
    std::string found;options>>found>>value;
    if(!options||found!=name)throw std::runtime_error("invalid loop-pair options");
  });
  input.current.points_I=readCloud(directory/"source.pcd");
  input.matched.points_I=readCloud(directory/"matched.pcd");
  input.submap=readCloud(directory/"target.pcd");
  if(!input.current.isValid()||!input.matched.isValid()||!c.isValid())throw std::runtime_error("invalid loop-pair geometry");
  return input;
}
}
