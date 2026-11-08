// Copyright 2026 rainbroken
// Licensed under the Apache License, Version 2.0

#ifndef ITERA_LIO__COMMON__TRANSFORMS_HPP_
#define ITERA_LIO__COMMON__TRANSFORMS_HPP_

#include <Eigen/Core>

#include "itera_lio/common/geometry.hpp"

namespace itera_lio::common
{

inline Eigen::Vector3d lidarToImu(
  const Eigen::Vector3d & point_L, const FixedExtrinsics & extrinsics)
{
  return extrinsics.R_IL * point_L + extrinsics.p_IL;
}

inline Eigen::Vector3d imuToWorld(
  const Eigen::Vector3d & point_I, const Pose & pose)
{
  return pose.R_WI * point_I + pose.p_WI;
}

inline Eigen::Vector3d lidarToWorld(
  const Eigen::Vector3d & point_L, const Pose & pose, const FixedExtrinsics & extrinsics)
{
  return imuToWorld(lidarToImu(point_L, extrinsics), pose);
}

inline Eigen::Vector3d worldToImu(
  const Eigen::Vector3d & point_W, const Pose & pose)
{
  return pose.R_WI.transpose() * (point_W - pose.p_WI);
}

}  // namespace itera_lio::common

#endif  // ITERA_LIO__COMMON__TRANSFORMS_HPP_
