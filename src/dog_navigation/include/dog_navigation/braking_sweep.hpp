#pragma once
#include "dog_navigation/execution_types.hpp"
#include <vector>
#include <stdexcept>

namespace dog_navigation {
struct BrakingPose {Point point;double yaw;};
// Planar numerical braking model: reaction interval, then independent linear
// and yaw deceleration. Heights are assigned from the certified terrain route
// by the caller; these samples are never sent as Z velocity commands.
inline std::vector<BrakingPose> brakingSweep(const Velocity &v,double linear_deceleration,
                                            double yaw_deceleration){
  if(!std::isfinite(speed(v))||!std::isfinite(linear_deceleration)||linear_deceleration<=0||
     !std::isfinite(yaw_deceleration)||yaw_deceleration<=0)throw std::invalid_argument("Invalid braking input");
  constexpr double reaction=.15,period=.02;
  const double linear=std::hypot(v.x,v.y),angular=std::abs(v.yaw);
  const double end=reaction+std::max(linear/linear_deceleration,angular/yaw_deceleration);
  if(!std::isfinite(end)||end>151.)throw std::invalid_argument("Braking horizon exceeds model bound");
  const auto integral=[](double value,double deceleration,double t){
    const double coast=std::min(t,.15),brake=std::min(std::max(0.,t-.15),value/deceleration);
    return value*coast+value*brake-.5*deceleration*brake*brake;
  };
  std::vector<BrakingPose> result{{Point::Zero(),0}};Point point=Point::Zero();
  for(double t=0;t<end;){
    const double next=std::min(end,t+period),middle=(t+next)*.5;
    const double angle=std::copysign(integral(angular,yaw_deceleration,middle),v.yaw);
    const double distance=integral(linear,linear_deceleration,next)-integral(linear,linear_deceleration,t);
    if(linear>1e-9){
      point.x()+=(v.x*std::cos(angle)-v.y*std::sin(angle))*distance/linear;
      point.y()+=(v.x*std::sin(angle)+v.y*std::cos(angle))*distance/linear;
    }
    const double yaw=std::copysign(integral(angular,yaw_deceleration,next),v.yaw);
    if((point-result.back().point).norm()>=.08||std::abs(yaw-result.back().yaw)>=.06||next==end)
      result.push_back({point,yaw});
    t=next;
  }
  return result;
}
}
