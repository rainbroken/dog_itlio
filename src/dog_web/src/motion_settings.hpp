#pragma once
#include <array>
#include <filesystem>
#include <mutex>
#include <nlohmann/json.hpp>

// Selected Web speeds are persisted independently of maps and never raise the
// robot driver's hard limits. A missing file starts with the previous speeds.
class MotionSettings {
public:
  explicit MotionSettings(std::filesystem::path path = {});
  nlohmann::json snapshot() const;
  std::array<double,3> limits() const;
  nlohmann::json save(const nlohmann::json &request);
private:
  void persist(const std::array<double,3> &values) const;
  std::filesystem::path path_;
  mutable std::mutex mutex_;
  std::array<double,3> values_{.30,.20,.40};
};
