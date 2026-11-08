#include "dog_slam/pose_conversion.hpp"
#include <gtest/gtest.h>

TEST(PoseConversion, RecoversBodyWithTiltedOffsetSensor)
{
  Eigen::Isometry3d base_to_imu = Eigen::Isometry3d::Identity();
  base_to_imu.linear() = (Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitX())).toRotationMatrix();
  base_to_imu.translation() = Eigen::Vector3d(0.25, 0.023, 0.12);
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.linear() = (Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX())).toRotationMatrix();
  expected.translation() = Eigen::Vector3d(4.0, -2.0, 1.7);
  const Eigen::Isometry3d measured = expected * base_to_imu;
  const auto actual = dog_slam::basePose(measured, base_to_imu);
  EXPECT_TRUE(actual.matrix().isApprox(expected.matrix(), 1e-12));
  // In particular, body z/roll/pitch are retained for stairs and slopes.
  EXPECT_NEAR(actual.translation().z(), 1.7, 1e-12);
}
