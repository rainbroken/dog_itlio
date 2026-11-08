// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__OUTPUT__PCD_RESULT_HPP_
#define ITERA_LIO__OUTPUT__PCD_RESULT_HPP_

#include <cstddef>
#include <string>

namespace itera_lio::output {

struct PcdWriteResult {
  bool success{false};
  std::string output_path;
  std::string error;
  std::size_t points_written{0U};
  double conversion_time_ms{0.0};
  double file_write_time_ms{0.0};
  double total_time_ms{0.0};
};

} // namespace itera_lio::output

#endif // ITERA_LIO__OUTPUT__PCD_RESULT_HPP_
