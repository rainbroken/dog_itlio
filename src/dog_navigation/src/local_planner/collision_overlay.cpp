#include "collision_overlay.hpp"
#include "dog_terrain/collision_geometry.hpp"
#include <algorithm>
#include <cmath>

namespace dog_navigation {
CollisionOverlay::CollisionOverlay(const Points &points,Options options,double body_radius,double body_top)
    :options_(options),radius_(body_radius+options.padding+options.voxel*.866025404),top_(body_top) {
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
  if(closest<=radius_)return -1;
  return std::pow((reach-closest)/o.avoidance_distance,2);
}
double CollisionOverlay::edgeRisk(const Point &a,const Point &b) const {
  if(obstacles_.empty())return 0;
  const Point pad(radius_,radius_,0);
  const Point low=a.cwiseMin(b)-pad,high=a.cwiseMax(b)+pad;
  for(int x=int(std::floor(low.x()/bin_));x<=int(std::floor(high.x()/bin_));++x)
    for(int y=int(std::floor(low.y()/bin_));y<=int(std::floor(high.y()/bin_));++y) {
      const auto it=obstacles_.find({x,y});if(it==obstacles_.end())continue;
      for(const auto &q:it->second)
        if(dog_terrain::sweptCylinderContact(a,b,q,radius_,
            options_.obstacle_bottom-options_.voxel*.5-1e-9,
            top_+options_.padding+options_.voxel*.5+1e-9))return -1;
    }
  const int count=std::max(1,int(std::ceil((b-a).norm()/(options_.voxel*.5))));double total=0;
  for(int i=0;i<=count;++i) {
    const double r=risk(a+(b-a)*(double(i)/count));if(r<0)return -1;total+=r;
  }
  return total/(count+1);
}
}  // namespace dog_navigation
