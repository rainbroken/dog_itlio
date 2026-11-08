#pragma once
#include "dog_navigation/obstacle_memory.hpp"
#include <geometry_msgs/msg/transform.hpp>
#include <chrono>
#include <stdexcept>

namespace dog_navigation {
using Clock=std::chrono::steady_clock;
inline double elapsed(Clock::time_point t) {return std::chrono::duration<double>(Clock::now()-t).count();}
inline Pose transform(const geometry_msgs::msg::Transform &t) {
  Eigen::Quaterniond q(t.rotation.w,t.rotation.x,t.rotation.y,t.rotation.z);
  if(!q.coeffs().allFinite()||std::abs(q.norm()-1)>.001)throw std::invalid_argument("Invalid TF quaternion");
  Pose p=Pose::Identity();p.linear()=q.normalized().toRotationMatrix();
  p.translation()=Point(t.translation.x,t.translation.y,t.translation.z);
  if(!validPose(p))throw std::invalid_argument("Invalid TF pose");
  return p;
}
}  // namespace dog_navigation
