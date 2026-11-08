#pragma once

#include <Eigen/Geometry>

namespace dog_slam {
// T_AB maps coordinates in B into A. Keep full SE(3), including mounting tilt.
inline Eigen::Isometry3d basePose(const Eigen::Isometry3d & odom_to_imu,
                                const Eigen::Isometry3d & base_to_imu)
{
  return odom_to_imu * base_to_imu.inverse();
}
}  // namespace dog_slam
