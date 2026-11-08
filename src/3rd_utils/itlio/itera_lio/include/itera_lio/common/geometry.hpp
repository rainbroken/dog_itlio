// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ITERA_LIO__COMMON__GEOMETRY_HPP_
#define ITERA_LIO__COMMON__GEOMETRY_HPP_

#include <Eigen/Core>
#include <Eigen/LU>

#include <cmath>

namespace itera_lio::common
{

// A lightweight pose view shared by sensor geometry, frontend and system code.
// R_WI maps IMU-frame vectors into the world frame; p_WI is IMU origin in W.
struct Pose
{
  Eigen::Matrix3d R_WI{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d p_WI{Eigen::Vector3d::Zero()};

  bool isValid(double rotation_tolerance = 1e-9) const
  {
    if (!R_WI.allFinite() || !p_WI.allFinite() || !std::isfinite(rotation_tolerance) ||
      rotation_tolerance < 0.0)
    {
      return false;
    }
    const Eigen::Matrix3d error = R_WI.transpose() * R_WI - Eigen::Matrix3d::Identity();
    return error.norm() <= rotation_tolerance &&
           std::abs(R_WI.determinant() - 1.0) <= rotation_tolerance;
  }
};

struct ImuPose
{
  double timestamp{0.0};
  Eigen::Matrix3d R_WI{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d p_WI{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_WI{Eigen::Vector3d::Zero()};
};

// Fixed LiDAR-to-IMU extrinsics: p_I = R_IL * p_L + p_IL.
struct FixedExtrinsics
{
  Eigen::Matrix3d R_IL{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d p_IL{Eigen::Vector3d::Zero()};

  bool isValid(double rotation_tolerance = 1e-9) const
  {
    if (!R_IL.allFinite() || !p_IL.allFinite() || !std::isfinite(rotation_tolerance) ||
      rotation_tolerance < 0.0)
    {
      return false;
    }
    const Eigen::Matrix3d error = R_IL.transpose() * R_IL - Eigen::Matrix3d::Identity();
    return error.norm() <= rotation_tolerance &&
           std::abs(R_IL.determinant() - 1.0) <= rotation_tolerance;
  }
};

}  // namespace itera_lio::common

#endif  // ITERA_LIO__COMMON__GEOMETRY_HPP_
