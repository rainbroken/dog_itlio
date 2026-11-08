// Copyright 2026 rainbroken
#ifndef ITERA_LIO__SENSOR__MEASUREMENT_SYNCHRONIZER_HPP_
#define ITERA_LIO__SENSOR__MEASUREMENT_SYNCHRONIZER_HPP_

#include <chrono>
#include <cstddef>
#include <utility>
#include "itera_lio/sensor/sensor_buffer.hpp"

namespace itera_lio::sensor
{
class MeasurementSynchronizer
{
public:
  MeasurementSynchronizer(std::size_t lidar_capacity, std::size_t imu_capacity,
    double imu_lookahead_sec = 0.0);
  PushResult pushLidar(LidarFrame frame) {return buffer_.pushLidar(std::move(frame));}
  PushResult pushImu(ImuData sample) {return buffer_.pushImu(std::move(sample));}
  bool waitAndPopNext(BufferedMeasurement & measurement, std::chrono::milliseconds timeout)
  {return buffer_.waitAndPopNext(measurement, timeout);}
  void stop() {buffer_.stop();}
  SensorBufferStats stats() const {return buffer_.stats();}
private:
  SensorBuffer buffer_;
};
}  // namespace itera_lio::sensor
#endif  // ITERA_LIO__SENSOR__MEASUREMENT_SYNCHRONIZER_HPP_
