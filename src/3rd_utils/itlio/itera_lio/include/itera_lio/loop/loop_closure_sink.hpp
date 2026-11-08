// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__LOOP__LOOP_CLOSURE_SINK_HPP_
#define ITERA_LIO__LOOP__LOOP_CLOSURE_SINK_HPP_

#include <cstddef>
#include <vector>

#include "itera_lio/loop/loop_types.hpp"

namespace itera_lio::loop
{

class LoopClosureSink
{
public:
  virtual ~LoopClosureSink() = default;
  virtual LoopClosureUpdate addKeyframe(KeyframeCloud cloud) = 0;
  virtual std::vector<LoopVerificationResult> pollCompleted() {return {};}
  virtual LoopWorkerStats workerStats() const {return {};}
  virtual void cancelPending() {}
  virtual std::size_t storedKeyframeClouds() const {return 0U;}
};

}  // namespace itera_lio::loop

#endif  // ITERA_LIO__LOOP__LOOP_CLOSURE_SINK_HPP_
