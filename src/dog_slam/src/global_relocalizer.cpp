#include "dog_slam/global_relocalizer.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <stdexcept>

namespace dog_slam {
namespace {
constexpr double pi=3.14159265358979323846;
using Clock=std::chrono::steady_clock;
Eigen::Vector3d xyz(const pcl::PointXYZ &p){return {p.x,p.y,p.z};}
using Descriptor=Eigen::Matrix<double,128,1>;
Descriptor descriptor(const std::vector<Eigen::Vector3d> &points,double radius) {
  Descriptor d=Descriptor::Zero();
  for(const auto &p:points) {
    const double r=p.head<2>().norm();
    if(r>=radius || std::abs(p.z())>=4)continue;
    const int ring=std::min(15,int(16*r/radius));
    const int height=std::clamp(int(p.z()+4),0,7);
    d[ring*8+height]+=1;
  }
  // Occupancy, rather than return counts: accumulated map voxel density and
  // a single spinning scan have substantially different sampling densities.
  d=(d.array()>0).cast<double>();
  if(d.norm()>0)d.normalize();
  return d;
}
}
struct GlobalRelocalizer::Impl {
  LocalizationOptions options;
  LocalizationCloud::ConstPtr map;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  std::vector<Eigen::Vector3d> anchors;
  std::vector<Descriptor> descriptors;
  bool historical=false;
  bool overflow=false;
  Impl(const MapMatcher &matcher,LocalizationOptions o,const std::vector<Pose3> &keyframes):options(o),map(matcher.map()) {
    options.validate();tree.setInputCloud(map);
    if(!keyframes.empty()) {
      historical=true;
      std::map<std::array<int,3>,Eigen::Vector3d> positions;
      for(const auto &pose:keyframes) {
        if(!validPose(pose))throw std::invalid_argument("Invalid historical keyframe pose");
        const auto p=pose.translation();
        positions.emplace(std::array<int,3>{int(std::floor(p.x()/.5)),int(std::floor(p.y()/.5)),
                                          int(std::floor(p.z()/.25))},p);
        if(positions.size()>10000)throw std::invalid_argument("Historical keyframe index exceeds 10000 spatial bins");
      }
      if(std::min(positions.size(),size_t(o.global.keyframe_candidates))>
         size_t(o.global.max_hypotheses/o.global.yaw_steps)){overflow=true;return;}
      for(const auto &entry:positions) {
        const auto &p=entry.second; anchors.push_back(p);
        std::vector<int> ids;std::vector<float> distances;
        tree.radiusSearch(pcl::PointXYZ(p.x(),p.y(),p.z()),o.scan_max_range,ids,distances);
        std::vector<Eigen::Vector3d> local;local.reserve(ids.size());
        for(int id:ids)local.push_back(xyz((*map)[id])-p);
        descriptors.push_back(descriptor(local,o.scan_max_range));
      }
      return;
    }
    struct Bin {Eigen::Vector3d sum=Eigen::Vector3d::Zero();int count=0;};
    std::map<std::array<int,3>,Bin> bins;
    const auto normals=matcher.normals();const auto &g=o.global;
    for(size_t i=0;i<map->size();++i) {
      const auto &n=(*normals)[i];
      if(!std::isfinite(n.normal_z)||std::abs(n.normal_z)<.85)continue;
      const auto p=xyz((*map)[i]);
      auto &b=bins[{int(std::floor(p.x()/g.xy_step)),int(std::floor(p.y()/g.xy_step)),int(std::floor(p.z()/g.height_step))}];
      b.sum+=p;++b.count;
      if(bins.size()>size_t(g.max_hypotheses/g.yaw_steps)){overflow=true;return;}
    }
    for(const auto &item:bins)anchors.push_back(item.second.sum/item.second.count);
  }
};
GlobalRelocalizer::GlobalRelocalizer(const MapMatcher &matcher,LocalizationOptions o,std::vector<Pose3> keyframes)
  :impl_(std::make_unique<Impl>(matcher,o,keyframes)){}
GlobalRelocalizer::~GlobalRelocalizer()=default;
LocalizationMatch GlobalRelocalizer::search(LocalizationCloud::ConstPtr input,const Pose3 &odom,const MapMatcher &matcher) const {
  const auto begin=Clock::now();const auto &o=impl_->options;const auto &g=o.global;
  auto expired=[&]{return std::chrono::duration<double>(Clock::now()-begin).count()>g.time_budget;};
  auto finish=[&](LocalizationMatch r){r.scope=impl_->historical ? "historical_keyframe_search" : "gravity_constrained_map_search";
    r.duration_ms=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();return r;};
  LocalizationMatch rejected;
  if(!g.enabled){rejected.reason="global_disabled";return finish(rejected);}
  if(!validPose(odom)||odom.linear()(2,2)<.7){rejected.reason="global_requires_upright_gravity_frame";return finish(rejected);}
  if(impl_->overflow){rejected.reason="global_hypothesis_budget_exceeded";return finish(rejected);}
  if(impl_->anchors.empty()){rejected.reason="global_no_horizontal_surface_anchors";return finish(rejected);}
  std::vector<Eigen::Vector3d> scan;
  if(input)for(const auto &p:*input) {
    const auto v=xyz(p);if(v.allFinite()&&v.norm()>=o.scan_min_range&&v.norm()<=o.scan_max_range)scan.push_back(v);
  }
  if(scan.size()<size_t(o.min_points)){rejected.reason="scan_too_small";return finish(rejected);}
  std::vector<Eigen::Vector3d> sampled;
  for(size_t i=0;i<std::min(scan.size(),size_t(g.scoring_points));++i)
    sampled.push_back(scan[i*scan.size()/std::min(scan.size(),size_t(g.scoring_points))]);
  const double yaw=std::atan2(odom.linear()(1,0),odom.linear()(0,0));
  const Eigen::Matrix3d tilt=Eigen::AngleAxisd(-yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix()*odom.linear();
  std::vector<size_t> anchor_ids;
  for(size_t i=0;i<impl_->anchors.size();++i)anchor_ids.push_back(i);
  if(impl_->historical) {
    std::vector<Eigen::Vector3d> leveled;leveled.reserve(scan.size());
    for(const auto &p:scan)leveled.push_back(tilt*p);
    const auto query=descriptor(leveled,o.scan_max_range);
    std::stable_sort(anchor_ids.begin(),anchor_ids.end(),[&](size_t a,size_t b){
      return query.dot(impl_->descriptors[a])>query.dot(impl_->descriptors[b]);
    });
    if(anchor_ids.size()>size_t(g.keyframe_candidates))anchor_ids.resize(g.keyframe_candidates);
  }
  struct Candidate {double cost;Pose3 pose;};std::vector<Candidate> ranked;
  std::vector<int> index(1);std::vector<float> distance(1);
  for(const auto id:anchor_ids)for(int angle=0;angle<g.yaw_steps;++angle) {
    if(expired()){rejected.reason="global_time_budget_exceeded";return finish(rejected);}
    Pose3 pose=Pose3::Identity();
    pose.linear()=Eigen::AngleAxisd(2*pi*angle/g.yaw_steps,Eigen::Vector3d::UnitZ()).toRotationMatrix()*tilt;
    // Historical poses are IMU origins: leave their metric height intact.
    // The bounded SE(3) refinement resolves the sensor/body lever arm.
    pose.translation()=impl_->anchors[id];
    if(!impl_->historical)pose.translation()+=pose.linear()*Eigen::Vector3d(0,0,g.body_height);
    double cost=0;
    for(const auto &p:sampled) {
      const Eigen::Vector3d v=pose*p;pcl::PointXYZ q(v.x(),v.y(),v.z());
      cost+=impl_->tree.nearestKSearch(q,1,index,distance)==1 ?
        std::min(o.coarse_correspondence,std::sqrt(double(distance[0]))) : o.coarse_correspondence;
    }
    ranked.push_back({cost/sampled.size(),pose});
  }
  rejected.hypotheses=int(ranked.size());
  std::stable_sort(ranked.begin(),ranked.end(),[](const auto &a,const auto &b){return a.cost<b.cost;});
  std::vector<Pose3> selected;std::vector<LocalizationMatch> accepted;
  for(const auto &candidate:ranked) {
    bool nearby=false;
    for(const auto &p:selected)if((p.translation()-candidate.pose.translation()).norm()<g.xy_step*.8&&
                                 rotationDistance(p,candidate.pose)<2*pi/g.yaw_steps*1.5){nearby=true;break;}
    if(nearby)continue;
    if(selected.size()>=size_t(g.refine_candidates))break;
    if(expired()){rejected.reason="global_time_budget_exceeded";return finish(rejected);}
    selected.push_back(candidate.pose);
    auto result=matcher.refine(input,candidate.pose);
    if(!result.accepted)continue;
    if(result.overlap<g.min_overlap)continue;
    bool duplicate=false;
    for(auto &r:accepted)if((r.map_to_base.translation()-result.map_to_base.translation()).norm()<.25&&
                            rotationDistance(r.map_to_base,result.map_to_base)<.12) {
      if(result.score<r.score)r=result;
      duplicate=true;break;
    }
    if(!duplicate)accepted.push_back(result);
  }
  if(expired()){rejected.reason="global_time_budget_exceeded";return finish(rejected);}
  if(accepted.empty()){rejected.reason="global_no_verified_candidate";return finish(rejected);}
  std::sort(accepted.begin(),accepted.end(),[](const auto &a,const auto &b){return a.score<b.score;});
  auto result=accepted.front();result.hypotheses=rejected.hypotheses;
  if(accepted.size()>1&&accepted[1].score-result.score<o.ambiguity_margin) {
    result.accepted=false;result.reason="ambiguous_global_relocalization";
  }
  return finish(result);
}
}  // namespace dog_slam
