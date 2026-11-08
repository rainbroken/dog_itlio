#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <optional>

namespace dog_terrain {
// Exact slab intersection for a point against a translating, fixed-yaw box.
// The vertical envelope selects obstacles relative to the support surface.
inline std::optional<double> sweptBoxContact(
    const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &obstacle,
    double yaw, double half_length, double half_width, double bottom, double top) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  const auto local = [&](const Eigen::Vector3d &v) -> Eigen::Vector3d {
    return {c*v.x()+s*v.y(), -s*v.x()+c*v.y(), v.z()};
  };
  const Eigen::Vector3d point = local(obstacle-a), motion = local(b-a);
  const Eigen::Vector3d lower(-half_length,-half_width,bottom), upper(half_length,half_width,top);
  double enter = 0, leave = 1;
  for (int axis=0; axis<3; ++axis) {
    if (std::abs(motion[axis])<1e-12) {
      if (axis==2 ? (point[axis]<=lower[axis] || point[axis]>=upper[axis]) :
                    (point[axis]<lower[axis] || point[axis]>upper[axis])) return std::nullopt;
    } else {
      double first=(point[axis]-upper[axis])/motion[axis];
      double last=(point[axis]-lower[axis])/motion[axis];
      if(first>last)std::swap(first,last);
      enter=std::max(enter,first);leave=std::min(leave,last);
      if(enter>leave || (axis==2 && enter==leave))return std::nullopt;
    }
  }
  return enter;
}

// Continuous sweep of a vertical circular body along a 3D ground segment.
// All coordinates must be finite, radius positive, bottom < top. Ground support
// is a separate constraint: this function cannot certify a floor or a stair.
inline std::optional<double> sweptCylinderContact(
    const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &obstacle,
    double radius, double bottom, double top) {
  const Eigen::Vector3d motion = b-a;
  double low = 0, high = 1;
  const double height = obstacle.z()-a.z();
  if (std::abs(motion.z()) < 1e-12) {
    if (height <= bottom || height >= top) return std::nullopt;
  } else {
    double enter = (height-top)/motion.z(), leave = (height-bottom)/motion.z();
    if (enter > leave) std::swap(enter,leave);
    low = std::max(low,enter); high = std::min(high,leave);
    if (low >= high) return std::nullopt;
    // The vertical interval is open, matching the point envelope. Move inside
    // it before testing XY so a single height-boundary touch is not a hit.
    low = std::nextafter(low,high); high = std::nextafter(high,low);
  }
  const double horizontal = motion.head<2>().squaredNorm();
  const double t = horizontal > 1e-20 ? std::clamp(
      (obstacle-a).head<2>().dot(motion.head<2>())/horizontal,low,high) : low;
  if ((obstacle-a-t*motion).head<2>().squaredNorm() <= radius*radius) return t;
  return std::nullopt;
}
}  // namespace dog_terrain
