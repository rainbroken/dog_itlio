// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__OUTPUT__ARCHIVE_POSE_CORRECTION_HPP_
#define ITERA_LIO__OUTPUT__ARCHIVE_POSE_CORRECTION_HPP_

#include "itera_lio/backend/pose_graph.hpp"

namespace itera_lio::output::detail {

common::Pose archiveCorrection(double timestamp,
                               const std::vector<backend::Keyframe> &raw_keyframes,
                               const std::vector<backend::Keyframe> &optimized_keyframes);
} // namespace itera_lio::output::detail

#endif // ITERA_LIO__OUTPUT__ARCHIVE_POSE_CORRECTION_HPP_
