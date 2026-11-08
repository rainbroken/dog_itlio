#pragma once
#include <cstdint>
#include <deque>

// Estimate the source sampling rate, independent of DDS callback scheduling.
// Averaging 1/dt magnifies queued bursts; count intervals over a time window.
class StreamFrequency {
public:
  void observe(int64_t stamp_ns) {
    if(stamp_ns<=0){stamps_.clear();return;}
    if(!stamps_.empty()){
      if(stamp_ns==stamps_.back())return;
      if(stamp_ns<stamps_.back())stamps_.clear();
    }
    stamps_.push_back(stamp_ns);
    while(stamps_.size()>1&&stamp_ns-stamps_.front()>2000000000LL)stamps_.pop_front();
    while(stamps_.size()>16384)stamps_.pop_front();
  }
  bool available() const {return stamps_.size()>1&&stamps_.back()-stamps_.front()>=500000000LL;}
  double hz() const {return available()?double(stamps_.size()-1)*1e9/double(stamps_.back()-stamps_.front()):0.;}
private:
  std::deque<int64_t> stamps_;
};
