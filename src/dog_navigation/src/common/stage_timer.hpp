#pragma once
#include <chrono>
namespace dog_navigation {
class StageTimer {
public:
  explicit StageTimer(double &milliseconds)
      : value_(milliseconds), start_(Clock::now()) {}
  ~StageTimer() {
    value_ += std::chrono::duration<double, std::milli>(Clock::now() - start_)
                  .count();
  }

private:
  using Clock = std::chrono::steady_clock;
  double &value_;
  Clock::time_point start_;
};
} // namespace dog_navigation
