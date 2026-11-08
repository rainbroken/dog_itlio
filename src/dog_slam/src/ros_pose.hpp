#pragma once
#include "dog_slam/map_localization.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <stdexcept>

namespace dog_slam {
inline Pose3 pose(const geometry_msgs::msg::Pose &p) {
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (!q.coeffs().allFinite() || std::abs(q.norm() - 1) > .001)
    throw std::invalid_argument("Initial orientation must be a unit quaternion");
  Pose3 out = Pose3::Identity(); out.linear() = q.normalized().toRotationMatrix();
  out.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  if (!validPose(out)) throw std::invalid_argument("Initial pose is not finite SE(3)");
  return out;
}
inline Pose3 transform(const geometry_msgs::msg::Transform &t) {
  geometry_msgs::msg::Pose p;
  p.position.x=t.translation.x; p.position.y=t.translation.y; p.position.z=t.translation.z; p.orientation=t.rotation;
  return pose(p);
}
inline geometry_msgs::msg::Pose messagePose(const Pose3 &p) {
  geometry_msgs::msg::Pose result;
  result.position.x=p.translation().x(); result.position.y=p.translation().y(); result.position.z=p.translation().z();
  Eigen::Quaterniond q(p.linear()); result.orientation.x=q.x(); result.orientation.y=q.y();
  result.orientation.z=q.z(); result.orientation.w=q.w(); return result;
}
}  // namespace dog_slam
