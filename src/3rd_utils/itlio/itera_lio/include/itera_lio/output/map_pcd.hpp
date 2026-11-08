// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__OUTPUT__MAP_PCD_HPP_
#define ITERA_LIO__OUTPUT__MAP_PCD_HPP_

#include "itera_lio/frontend/global_map.hpp"
#include "itera_lio/mapping/color/colored_global_map.hpp"
#include "itera_lio/output/pcd_result.hpp"
#include <vector>

namespace itera_lio::output {

PcdWriteResult writeMapPcd(const frontend::GlobalMap &map, const std::string &output_directory);

PcdWriteResult writeMapPcd(const std::vector<common::MapPoint> &points,
                           const std::string &output_directory, const std::string &filename_suffix);

PcdWriteResult writeColoredMapPcd(const std::vector<mapping::color::ColoredPoint> &points,
                                  const std::string &output_directory);

PcdWriteResult writeColoredMapPcd(const mapping::color::ColoredGlobalMap &map,
                                  const std::string &output_directory);

} // namespace itera_lio::output

#endif // ITERA_LIO__OUTPUT__MAP_PCD_HPP_
