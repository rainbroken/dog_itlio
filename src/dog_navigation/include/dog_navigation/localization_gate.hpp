#pragma once
#include <chrono>
#include <filesystem>
#include <string>

namespace dog_navigation {
std::filesystem::path canonicalMapPath(std::string path);
// Boundary adapter for the current SLAM wire protocol. Planning and collision
// code do not depend on ROS messages, localization internals, or JSON fields.
class LocalizationGate {
public:
  using Time = std::chrono::steady_clock::time_point;
  LocalizationGate(std::filesystem::path project, double timeout);
  std::string updateJson(const std::string &payload, Time receipt);
  bool ready(Time now) const;
  double remaining(Time now) const;
private:
  std::filesystem::path project_;
  double timeout_;
  bool valid_ = false;
  Time receipt_{};
};
}  // namespace dog_navigation
