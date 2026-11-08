#pragma once
#include <nlohmann/json.hpp>
#include <chrono>
#include <filesystem>
#include <map>
#include <mutex>
#include <cstdint>

class ResourceMonitor {
public:
  explicit ResourceMonitor(std::filesystem::path proc="/proc"):proc_(std::move(proc)){}
  nlohmann::json snapshot();
private:
  struct Ticks{uint64_t total=0,idle=0;};
  std::filesystem::path proc_;std::mutex mutex_;std::map<std::string,Ticks> previous_;
  std::chrono::steady_clock::time_point sampled_{};nlohmann::json cached_;
};
