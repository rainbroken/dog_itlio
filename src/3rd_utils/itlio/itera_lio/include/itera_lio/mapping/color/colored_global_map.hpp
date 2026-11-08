// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__MAPPING__COLOR__COLORED_GLOBAL_MAP_HPP_
#define ITERA_LIO__MAPPING__COLOR__COLORED_GLOBAL_MAP_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "itera_lio/common/voxel.hpp"
#include "itera_lio/mapping/color/colorizer.hpp"

namespace itera_lio::mapping::color {

struct ColoredGlobalMapUpdate {
  std::size_t inserted_count{0U};
  std::size_t replaced_count{0U};
  std::vector<ColoredPoint> upserted_points;
};

class ColoredGlobalMap {
public:
  explicit ColoredGlobalMap(float voxel_size, std::size_t max_points = 0U,
                            std::size_t max_resident_chunks = 0U,
                            const std::string &spill_directory = {});
  ~ColoredGlobalMap();

  ColoredGlobalMap(const ColoredGlobalMap &) = delete;
  ColoredGlobalMap &operator=(const ColoredGlobalMap &) = delete;

  std::size_t addPoints(const std::vector<ColoredPoint> &points,
                        ColoredGlobalMapUpdate *update = nullptr);
  bool
  forEachPoint(const std::function<void(const ColoredPoint &)> &visitor) const;
  bool getAllPoints(std::vector<ColoredPoint> &points) const;
  // Fetch one 5 m archive chunk, including chunks spilled to disk.
  bool getChunkPoints(std::int64_t x, std::int64_t y, std::int64_t z,
                      std::size_t max_points,
                      std::vector<ColoredPoint> &points) const;
  std::size_t size() const;
  std::size_t residentChunkCount() const;
  std::size_t spilledChunkCount() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace itera_lio::mapping::color

#endif // ITERA_LIO__MAPPING__COLOR__COLORED_GLOBAL_MAP_HPP_
