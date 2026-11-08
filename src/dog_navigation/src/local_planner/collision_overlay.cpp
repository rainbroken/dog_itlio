#include "collision_overlay.hpp"
#include "dog_terrain/collision_geometry.hpp"
#include <algorithm>
#include <cmath>

namespace dog_navigation {
CollisionOverlay::CollisionOverlay(const Points &points,Options options,double body_radius,double body_top,
                                   double body_length,double body_width)
    :options_(options),radius_(body_radius+options.padding+options.voxel*.866025404),top_(body_top) {
  if(body_length>0&&body_width>0){
    const double padding=options.padding+options.voxel*.866025404;
    half_length_=body_length*.5+padding;half_width_=body_width*.5+padding;
    radius_=std::hypot(half_length_,half_width_);
  }
  for(const auto &p:points)
    obstacles_[{int(std::floor(p.x()/bin_)),int(std::floor(p.y()/bin_))}].push_back(p);
}
double CollisionOverlay::risk(const Point &p) const {
  if(obstacles_.empty())return 0;
  const auto &o=options_;
  const double reach=radius_+o.avoidance_distance;
  double closest=reach;
  for(int x=int(std::floor((p.x()-reach)/bin_));x<=int(std::floor((p.x()+reach)/bin_));++x)
    for(int y=int(std::floor((p.y()-reach)/bin_));y<=int(std::floor((p.y()+reach)/bin_));++y) {
      auto it=obstacles_.find({x,y});if(it==obstacles_.end())continue;
      for(const auto &q:it->second) {
        const double dz=q.z()-p.z();
        if(dz<o.obstacle_bottom-o.voxel*.5||dz>top_+o.padding+o.voxel*.5)continue;
        closest=std::min(closest,(q-p).head<2>().norm());
      }
    }
  const double occupied_radius=half_length_>0?std::min(half_length_,half_width_):radius_;
  if(closest<=occupied_radius)return -1;
  return std::pow((reach-closest)/o.avoidance_distance,2);
}
bool CollisionOverlay::motionClear(const Point &a,const Point &b,double first_yaw,double last_yaw) const {
  if(obstacles_.empty())return true;
  const Point pad(radius_,radius_,0);
  const Point low=a.cwiseMin(b)-pad,high=a.cwiseMax(b)+pad;
  for(int x=int(std::floor(low.x()/bin_));x<=int(std::floor(high.x()/bin_));++x)
    for(int y=int(std::floor(low.y()/bin_));y<=int(std::floor(high.y()/bin_));++y) {
      const auto it=obstacles_.find({x,y});if(it==obstacles_.end())continue;
      for(const auto &q:it->second)
        if(half_length_>0?
           bool(dog_terrain::sweptTurningBoxContact(a,b,q,first_yaw,last_yaw,half_length_,half_width_,
             options_.obstacle_bottom-options_.voxel*.5-1e-9,top_+options_.padding+options_.voxel*.5+1e-9)):
           bool(dog_terrain::sweptCylinderContact(a,b,q,radius_,
             options_.obstacle_bottom-options_.voxel*.5-1e-9,top_+options_.padding+options_.voxel*.5+1e-9)))return false;
    }
  return true;
}
double CollisionOverlay::edgeRisk(const Point &a,const Point &b) const {
  if(obstacles_.empty())return 0;
  const double yaw=std::atan2(b.y()-a.y(),b.x()-a.x());
  if(!motionClear(a,b,yaw,yaw))return -1;
  const int count=std::max(1,int(std::ceil((b-a).norm()/(options_.voxel*.5))));double total=0;
  for(int i=0;i<=count;++i) {
    // Rectangular collision was checked with the actual heading above. The
    // circumscribed distance remains only a soft clearance preference.
    const double r=risk(a+(b-a)*(double(i)/count));
    if(r<0&&half_length_<=0)return -1;
    total+=std::max(0.,r);
  }
  return total/(count+1);
}
}  // namespace dog_navigation
