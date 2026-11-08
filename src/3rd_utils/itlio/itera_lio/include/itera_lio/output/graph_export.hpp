// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__OUTPUT__GRAPH_EXPORT_HPP_
#define ITERA_LIO__OUTPUT__GRAPH_EXPORT_HPP_

#include "itera_lio/backend/pose_graph.hpp"
#include "itera_lio/output/pcd_result.hpp"
#include <vector>

namespace itera_lio::output {

PcdWriteResult writeTrajectoryCsv(const std::vector<backend::Keyframe> &keyframes,
                                  const std::string &output_directory,
                                  const std::string &filename_suffix);

PcdWriteResult writePoseGraphConstraintsCsv(const backend::PoseGraph &graph,
                                            const std::string &output_directory);

} // namespace itera_lio::output

#endif // ITERA_LIO__OUTPUT__GRAPH_EXPORT_HPP_
