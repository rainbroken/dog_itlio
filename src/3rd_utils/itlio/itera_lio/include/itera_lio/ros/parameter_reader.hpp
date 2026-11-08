// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__ROS__PARAMETER_READER_HPP_
#define ITERA_LIO__ROS__PARAMETER_READER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "itera_lio/estimator/esikf.hpp"
#include "rclcpp/rclcpp.hpp"

namespace itera_lio::ros
{

class ParameterReader
{
public:
  explicit ParameterReader(rclcpp::Node & node)
  : node_(node) {}

  std::size_t positiveSize(const std::string & name, std::int64_t default_value) const;
  // Zero is a valid value for optional preview limits and means unlimited.
  std::size_t nonNegativeSize(const std::string & name, std::int64_t default_value) const;
  estimator::FixedExtrinsics lidarExtrinsics() const;

private:
  rclcpp::Node & node_;
};

}  // namespace itera_lio::ros

#endif  // ITERA_LIO__ROS__PARAMETER_READER_HPP_
