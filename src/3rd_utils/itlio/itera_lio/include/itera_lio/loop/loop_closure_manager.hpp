// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ITERA_LIO__LOOP__LOOP_CLOSURE_MANAGER_HPP_
#define ITERA_LIO__LOOP__LOOP_CLOSURE_MANAGER_HPP_

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <string>
#include <vector>

#include "itera_lio/loop/keyframe_cloud_store.hpp"
#include "itera_lio/loop/loop_detector.hpp"
#include "itera_lio/loop/loop_verifier.hpp"
#include "itera_lio/loop/loop_closure_sink.hpp"

namespace itera_lio::loop
{

struct LoopClosureOptions
{
  bool enabled{false};
  LoopDetectorOptions detector;
  LoopVerifierOptions verifier;
  float keyframe_voxel_size{0.20F};
  std::size_t max_keyframe_points{4000U};
  std::size_t submap_half_window{10U};
  double min_attempt_interval_sec{5.0};
  std::size_t confirmation_count{2U};
  double confirmation_translation_tolerance{2.0};
  double confirmation_rotation_tolerance_rad{0.10};
  StrongLoopEvidenceOptions strong_evidence;
  std::size_t worker_queue_capacity{2U};
  double task_timeout_ms{1000.0};
  std::size_t max_resident_keyframe_clouds{200U};
  std::string archive_directory;
  std::string diagnostic_pair_directory;
  std::size_t diagnostic_pair_limit{128U};
};

class LoopClosureManager : public LoopClosureSink
{
public:
  explicit LoopClosureManager(const LoopClosureOptions & options = {});
  ~LoopClosureManager() override;

  LoopClosureManager(const LoopClosureManager &) = delete;
  LoopClosureManager & operator=(const LoopClosureManager &) = delete;

  LoopClosureUpdate addKeyframe(KeyframeCloud cloud) override;
  std::vector<LoopVerificationResult> pollCompleted() override;
  LoopWorkerStats workerStats() const override;
  void cancelPending() override;
  std::size_t storedKeyframeClouds() const override {return store_.size();}
  const KeyframeCloudStore & store() const {return store_;}
  KeyframeCloudStoreStats storeStats() const {return store_.stats();}
  std::vector<LoopVerificationResult> reverifyOptimized(
    const std::vector<backend::Keyframe> & optimized_keyframes) const;

private:
  struct VerificationTask
  {
    LoopCandidate candidate;
    KeyframeCloud current;
    KeyframeCloud matched;
    std::vector<common::MapPoint> submap;
    std::chrono::steady_clock::time_point deadline;
    std::uint64_t cancellation_generation{0U};
  };

  void workerLoop();
  void submit(VerificationTask task);

  std::size_t diagnostic_pair_count_{0U};
  LoopClosureOptions options_;
  KeyframeCloudStore store_;
  LoopDetector detector_;
  LoopVerifier verifier_;
  double last_attempt_timestamp_{-1.0};
  mutable std::mutex worker_mutex_;
  std::condition_variable worker_condition_;
  std::deque<VerificationTask> pending_tasks_;
  std::deque<LoopVerificationResult> completed_results_;
  LoopWorkerStats worker_stats_;
  std::uint64_t cancellation_generation_{0U};
  bool stopping_{false};
  std::thread worker_;
};

}  // namespace itera_lio::loop

#endif  // ITERA_LIO__LOOP__LOOP_CLOSURE_MANAGER_HPP_
