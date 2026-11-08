#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <nlohmann/json.hpp>
#include <vector>

namespace dog_navigation::benchmark {
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
inline double milliseconds(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start)
      .count();
}
inline Json distribution(std::vector<double> samples, const char *unit) {
  std::sort(samples.begin(), samples.end());
  auto quantile = [&](double p) {
    return samples.at(size_t(std::ceil(p * samples.size())) - 1);
  };
  return {
      {"unit", unit},        {"samples", samples},   {"count", samples.size()},
      {"p50", quantile(.5)}, {"p95", quantile(.95)}, {"max", samples.back()}};
}
// Percentiles describe batch means, not a worst-case execution-time bound.
template <class Work>
Json kernel(Work work, int windows = 30, int batch = 100) {
  std::vector<double> samples;
  volatile double checksum = 0;
  for (int window = 0; window < windows; ++window) {
    auto start = Clock::now();
    for (int i = 0; i < batch; ++i)
      checksum += work();
    samples.push_back(milliseconds(start) * 1000 / batch);
  }
  auto report = distribution(samples, "microseconds_per_call_batch_mean");
  report["calls_per_window"] = batch;
  report["checksum"] = checksum;
  return report;
}
Json controlKernels(const std::vector<Eigen::Vector3d> &path,
                    const std::vector<bool> &stairs);
Json perceptionKernels();
Json successfulPlanningKernels();
} // namespace dog_navigation::benchmark
