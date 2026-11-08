// Copyright 2026 rainbroken
#include "itera_lio/sensor/measurement_synchronizer.hpp"
namespace itera_lio::sensor
{
MeasurementSynchronizer::MeasurementSynchronizer(const std::size_t lidar_capacity,
  const std::size_t imu_capacity, const double imu_lookahead_sec)
: buffer_(lidar_capacity, imu_capacity, imu_lookahead_sec) {}
}  // namespace itera_lio::sensor
