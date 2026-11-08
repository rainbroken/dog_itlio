// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <gtest/gtest.h>

#include "itera_lio/estimator/tracking_state.hpp"

namespace
{

itera_lio::common::ObservationUpdateReport goodObservation()
{
  itera_lio::common::ObservationUpdateReport observation;
  observation.whitened_eigenvalues.setOnes();
  observation.whitened_condition_number = 1.0;
  return observation;
}

}  // namespace

TEST(TrackingState, AppliesDegradedAndLostHysteresis)
{
  itera_lio::estimator::TrackingStateOptions options;
  options.minimum_effective_points = 10U;
  options.lost_effective_points = 2U;
  options.degraded_frames = 2U;
  options.lost_frames = 2U;
  options.recovery_frames = 2U;
  itera_lio::estimator::TrackingStateMachine machine(options);
  auto weak = goodObservation();
  weak.whitened_eigenvalues[0] = 0.0;
  weak.whitened_condition_number = INFINITY;

  EXPECT_EQ(
    machine.update(true, 10U, 0.1, weak).state,
    itera_lio::estimator::TrackingState::kNormal);
  const auto degraded = machine.update(true, 10U, 0.1, weak);
  EXPECT_TRUE(degraded.transitioned);
  EXPECT_EQ(degraded.bad_observation_mask, 8U | 16U);
  EXPECT_EQ(degraded.state, itera_lio::estimator::TrackingState::kDegraded);
  EXPECT_EQ(
    machine.update(false, 0U, 0.0, weak).state,
    itera_lio::estimator::TrackingState::kDegraded);
  const auto lost = machine.update(false, 0U, 0.0, weak);
  EXPECT_TRUE(lost.transitioned);
  EXPECT_EQ(lost.bad_observation_mask, 1U | 2U | 8U | 16U);
  EXPECT_EQ(lost.state, itera_lio::estimator::TrackingState::kLost);
  EXPECT_EQ(
    machine.update(true, 20U, 0.1, goodObservation()).state,
    itera_lio::estimator::TrackingState::kLost);
}

TEST(TrackingState, AutomaticRecoveryMustBeExplicitlyEnabled)
{
  itera_lio::estimator::TrackingStateOptions options;
  options.minimum_effective_points = 10U;
  options.lost_effective_points = 0U;
  options.degraded_frames = 1U;
  options.lost_frames = 2U;
  options.recovery_frames = 2U;
  options.allow_automatic_recovery = true;
  itera_lio::estimator::TrackingStateMachine machine(options);
  auto weak = goodObservation();
  weak.whitened_eigenvalues[0] = 0.0;
  weak.whitened_condition_number = INFINITY;
  ASSERT_EQ(
    machine.update(true, 10U, 0.1, weak).state,
    itera_lio::estimator::TrackingState::kDegraded);
  machine.update(true, 10U, 0.1, goodObservation());
  EXPECT_EQ(
    machine.update(true, 10U, 0.1, goodObservation()).state,
    itera_lio::estimator::TrackingState::kNormal);
}
