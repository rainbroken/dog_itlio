// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ITERA_LIO__COMMON__MEASUREMENT_TYPES_HPP_
#define ITERA_LIO__COMMON__MEASUREMENT_TYPES_HPP_

#include "itera_lio/sensor/imu_data.hpp"
#include "itera_lio/sensor/lidar_frame.hpp"

namespace itera_lio::common
{

enum class MeasurementType
{
  kLidar,
  kImu,
};

struct Measurement
{
  MeasurementType type{MeasurementType::kImu};
  sensor::LidarFrame lidar;
  sensor::ImuData imu;
};

}  // namespace itera_lio::common

#endif  // ITERA_LIO__COMMON__MEASUREMENT_TYPES_HPP_
