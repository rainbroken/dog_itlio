#pragma once
#include "dog_navigation/braking_sweep.hpp"
#include "dog_terrain/terrain_graph.hpp"
#include <functional>

namespace dog_navigation {
// A body-frame planar velocity over the certified 3D terrain route. The caller
// supplies current live-obstacle checks; no Z velocity is generated.
inline bool certifyBraking(const dog_terrain::TerrainGraph &graph,const Point &ground,double yaw,
                           const Velocity &velocity,const Points &route,
                           double deceleration,double yaw_deceleration,
                           const std::function<bool(const Point &,const Point &,double,double)> &live_clear) {
  Point previous=ground;double previous_yaw=yaw;
  const auto sweep=brakingSweep(velocity,deceleration,yaw_deceleration);
  for(size_t sample_index=1;sample_index<sweep.size();++sample_index){
    const auto &sample=sweep[sample_index];
    Point next=ground+Point(std::cos(yaw)*sample.point.x()-std::sin(yaw)*sample.point.y(),
                            std::sin(yaw)*sample.point.x()+std::cos(yaw)*sample.point.y(),0);
    next.z()=previous.z();double best=std::numeric_limits<double>::infinity(),height=next.z();
    if(sample.point.head<2>().norm()>1e-9)for(size_t i=1;i<route.size();++i){
      const Point a=route[i-1],d=route[i]-a;const double n=d.head<2>().squaredNorm();if(n<1e-12)continue;
      const double t=std::clamp((next-a).head<2>().dot(d.head<2>())/n,0.,1.);
      const Point p=a+t*d;const double distance=(next-p).squaredNorm();
      if(distance<best){best=distance;height=p.z();}
    }
    next.z()=height;const double next_yaw=yaw+sample.yaw;
    if(!graph.motionClear(previous,next,previous_yaw,next_yaw)||
       !live_clear(previous,next,previous_yaw,next_yaw))return false;
    previous=next;previous_yaw=next_yaw;
  }
  return true;
}
// Velocity scaling changes the shape of a curved braking path, so its swept
// region does not contain the unscaled one. Reserve headroom AND certify the
// exact command that the independent guard will receive.
inline bool certifyTrackingBraking(const dog_terrain::TerrainGraph &graph,const Point &ground,double yaw,
                                  const Velocity &velocity,const Points &route,
                                  double deceleration,double yaw_deceleration,
                                  const std::function<bool(const Point &,const Point &,double,double)> &live_clear) {
  const Velocity reserve{velocity.x*1.12,velocity.y*1.12,velocity.yaw*1.12};
  return certifyBraking(graph,ground,yaw,reserve,route,deceleration,yaw_deceleration,live_clear)&&
         certifyBraking(graph,ground,yaw,velocity,route,deceleration,yaw_deceleration,live_clear);
}
}
