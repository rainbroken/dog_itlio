// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef ITERA_LIO__BACKEND__BACKEND_SINK_HPP_
#define ITERA_LIO__BACKEND__BACKEND_SINK_HPP_

#include <optional>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "itera_lio/backend/pose_graph.hpp"
#include "itera_lio/common/geometry.hpp"

namespace itera_lio::backend
{

struct BackendUpdate;

struct BackendWorkerStats
{
  std::size_t queue_depth{0U};
  std::size_t max_queue_depth{0U};
  std::uint64_t submitted{0U};
  std::uint64_t completed{0U};
  std::uint64_t dropped{0U};
  std::uint64_t timed_out{0U};
  std::uint64_t cancelled{0U};
  double last_duration_ms{0.0};
  double max_duration_ms{0.0};
};

class BackendSink
{
public:
  virtual ~BackendSink() = default;
  // Scan quality in the frontend chart [right body rotation, world position].
  // This is measurement information, not a relative-pose covariance.
  virtual void observeOdometryQuality(
    const common::Pose &, const PoseInformationMatrix &) {}
  virtual BackendUpdate addOdometryKeyframe(
    double timestamp, const common::Pose & pose_WI) = 0;
  virtual std::optional<BackendUpdate> addLoopConstraint(
    common::KeyframeId from_id, common::KeyframeId to_id,
    const common::Pose & T_Ia_Ib,
    const PoseInformationMatrix & information) = 0;
  virtual bool enqueueLoopConstraint(
    common::KeyframeId from_id, common::KeyframeId to_id,
    const common::Pose & T_Ia_Ib, const PoseInformationMatrix & information) = 0;
  virtual std::vector<BackendUpdate> pollCompleted() = 0;
  virtual BackendWorkerStats workerStats() const {return {};}
  virtual void cancelPending() {}
};

}  // namespace itera_lio::backend

#endif  // ITERA_LIO__BACKEND__BACKEND_SINK_HPP_
