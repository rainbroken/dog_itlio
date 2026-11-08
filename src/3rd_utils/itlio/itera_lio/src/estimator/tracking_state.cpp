// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "itera_lio/estimator/tracking_state.hpp"

#include <cmath>
#include <stdexcept>

namespace itera_lio::estimator
{

const char * trackingStateName(const TrackingState state)
{
  switch (state) {
    case TrackingState::kNormal: return "normal";
    case TrackingState::kDegraded: return "degraded";
    case TrackingState::kLost: return "lost";
  }
  return "unknown";
}

TrackingStateMachine::TrackingStateMachine(const TrackingStateOptions & options)
: options_(options)
{
  if (options_.minimum_effective_points == 0U ||
    options_.lost_effective_points > options_.minimum_effective_points ||
    !std::isfinite(options_.maximum_mean_residual) ||
    options_.maximum_mean_residual <= 0.0 ||
    !std::isfinite(options_.minimum_whitened_eigenvalue) ||
    options_.minimum_whitened_eigenvalue <= 0.0 ||
    !std::isfinite(options_.maximum_whitened_condition_number) ||
    options_.maximum_whitened_condition_number <= 1.0 ||
    options_.degraded_frames == 0U || options_.lost_frames == 0U ||
    options_.recovery_frames == 0U)
  {
    throw std::invalid_argument("invalid tracking state options");
  }
}

TrackingStateReport TrackingStateMachine::update(
  const bool corrected, const std::size_t effective_points,
  const double mean_residual, const common::ObservationUpdateReport & observation)
{
  TrackingStateReport report;
  report.previous_state = state_;
  const double minimum = observation.whitened_eigenvalues.minCoeff();
  if (!corrected) {report.bad_observation_mask |= 1U;}
  if (effective_points < options_.minimum_effective_points) {
    report.bad_observation_mask |= 2U;
  }
  if (!std::isfinite(mean_residual) || mean_residual > options_.maximum_mean_residual) {
    report.bad_observation_mask |= 4U;
  }
  if (!std::isfinite(minimum) || minimum < options_.minimum_whitened_eigenvalue) {
    report.bad_observation_mask |= 8U;
  }
  if (!std::isfinite(observation.whitened_condition_number) ||
    observation.whitened_condition_number > options_.maximum_whitened_condition_number)
  {
    report.bad_observation_mask |= 16U;
  }
  report.observation_good = corrected &&
    effective_points >= options_.minimum_effective_points &&
    std::isfinite(mean_residual) && mean_residual <= options_.maximum_mean_residual &&
    std::isfinite(minimum) && minimum >= options_.minimum_whitened_eigenvalue &&
    std::isfinite(observation.whitened_condition_number) &&
    observation.whitened_condition_number <= options_.maximum_whitened_condition_number;
  report.observation_lost = !corrected || effective_points < options_.lost_effective_points ||
    !std::isfinite(mean_residual);

  consecutive_good_ = report.observation_good ? consecutive_good_ + 1U : 0U;
  consecutive_bad_ = report.observation_good ? 0U : consecutive_bad_ + 1U;
  consecutive_lost_ = report.observation_lost ? consecutive_lost_ + 1U : 0U;

  if (state_ == TrackingState::kNormal) {
    if (consecutive_lost_ >= options_.lost_frames) {
      state_ = TrackingState::kLost;
    } else if (consecutive_bad_ >= options_.degraded_frames) {
      state_ = TrackingState::kDegraded;
    }
  } else {
    if (state_ == TrackingState::kDegraded) {
      if (consecutive_lost_ >= options_.lost_frames) {
        state_ = TrackingState::kLost;
      } else if (options_.allow_automatic_recovery &&
        consecutive_good_ >= options_.recovery_frames) {
        state_ = TrackingState::kNormal;
      }
    } else if (options_.allow_automatic_recovery &&
      consecutive_good_ >= options_.recovery_frames) {
      state_ = TrackingState::kDegraded;
    }
  }

  report.state = state_;
  report.transitioned = report.previous_state != report.state;
  report.consecutive_bad = consecutive_bad_;
  report.consecutive_lost = consecutive_lost_;
  report.consecutive_good = consecutive_good_;
  return report;
}

}  // namespace itera_lio::estimator
