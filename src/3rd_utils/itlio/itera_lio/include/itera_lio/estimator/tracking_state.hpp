// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ITERA_LIO__ESTIMATOR__TRACKING_STATE_HPP_
#define ITERA_LIO__ESTIMATOR__TRACKING_STATE_HPP_

#include <cstddef>

#include "itera_lio/common/observation_types.hpp"

namespace itera_lio::estimator
{

enum class TrackingState
{
  kNormal,
  kDegraded,
  kLost
};

const char * trackingStateName(TrackingState state);

struct TrackingStateOptions
{
  std::size_t minimum_effective_points{20U};
  std::size_t lost_effective_points{10U};
  double maximum_mean_residual{0.25};
  double minimum_whitened_eigenvalue{1e-2};
  double maximum_whitened_condition_number{500.0};
  std::size_t degraded_frames{5U};
  std::size_t lost_frames{5U};
  std::size_t recovery_frames{10U};
  bool allow_automatic_recovery{false};
};

struct TrackingStateReport
{
  TrackingState state{TrackingState::kNormal};
  TrackingState previous_state{TrackingState::kNormal};
  bool transitioned{false};
  bool observation_good{false};
  bool observation_lost{false};
  // Bit mask: 1=no correction, 2=few points, 4=residual,
  // 8=minimum whitened eigenvalue, 16=whitened condition number.
  unsigned int bad_observation_mask{0U};
  std::size_t consecutive_bad{0U};
  std::size_t consecutive_lost{0U};
  std::size_t consecutive_good{0U};
};

class TrackingStateMachine
{
public:
  explicit TrackingStateMachine(const TrackingStateOptions & options = {});

  TrackingStateReport update(
    bool corrected, std::size_t effective_points, double mean_residual,
    const common::ObservationUpdateReport & observation);

  TrackingState state() const {return state_;}

private:
  TrackingStateOptions options_;
  TrackingState state_{TrackingState::kNormal};
  std::size_t consecutive_bad_{0U};
  std::size_t consecutive_lost_{0U};
  std::size_t consecutive_good_{0U};
};

}  // namespace itera_lio::estimator

#endif  // ITERA_LIO__ESTIMATOR__TRACKING_STATE_HPP_
