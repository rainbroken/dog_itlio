#pragma once
#include "dog_slam/map_localization.hpp"

namespace dog_slam {
// Deterministic gravity-constrained map-wide search. No ROS, random sampling,
// unknown-space creation, or dependence on the terrain/navigation packages.
class GlobalRelocalizer {
public:
  GlobalRelocalizer(const MapMatcher &matcher, LocalizationOptions options,
                    std::vector<Pose3> keyframes = {});
  ~GlobalRelocalizer();
  LocalizationMatch search(LocalizationCloud::ConstPtr scan_base, const Pose3 &odom_to_base,
                          const MapMatcher &matcher) const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace dog_slam
