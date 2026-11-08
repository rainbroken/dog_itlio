#pragma once
#include <string>

namespace dog_navigation {
class TaskSupervisor {
public:
  void start(double now);
  void pause();
  void resume(double now);
  void cancel();
  void abort(const std::string &reason);
  bool update(double now, bool localization, bool route, bool robot,
              bool progress);
  void succeed();
  bool active() const;
  bool paused() const { return state_ == "paused"; }
  const std::string &state() const { return state_; }
  const std::string &reason() const { return reason_; }
  bool retryDue(double now);

private:
  std::string state_ = "idle", reason_;
  double started_ = 0, blocked_since_ = -1, last_progress_ = 0,
         last_retry_ = -100;
  unsigned retries_ = 0;
};
} // namespace dog_navigation
