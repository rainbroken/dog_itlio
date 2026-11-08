// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ITERA_LIO__BACKEND__BACKEND_MANAGER_HPP_
#define ITERA_LIO__BACKEND__BACKEND_MANAGER_HPP_

#include <chrono>
#include <utility>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_set>
#include <vector>

#include "itera_lio/backend/keyframe_manager.hpp"
#include "itera_lio/backend/backend_sink.hpp"
#include "itera_lio/backend/pose_graph_optimizer.hpp"

namespace itera_lio::backend
{

struct BackendOptions
{
  // Backend output is not consumed by the first-stage frontend by default.
  bool enabled{false};
  KeyframeManagerOptions keyframes;
  PoseGraphOptimizerOptions optimizer;
  double odometry_rotation_stddev{0.05};
  double odometry_translation_stddev{0.10};
  bool asynchronous_optimization{false};
  std::size_t worker_queue_capacity{2U};
  double task_timeout_ms{2000.0};
};

struct BackendUpdate
{
  bool keyframe_added{false};
  bool loop_constraint_added{false};
  std::size_t keyframe_count{0U};
  std::size_t odometry_constraint_count{0U};
  std::size_t loop_constraint_count{0U};
  PoseGraphOptimizationReport optimization;
  std::optional<Keyframe> keyframe;
};

// Owns persistent backend state. It deliberately never mutates the frontend ESKF state:
// optimized poses are a remapping product once a loop constraint has been accepted.
class BackendManager : public BackendSink
{
public:
  explicit BackendManager(const BackendOptions & options = {});
  ~BackendManager() override;

  BackendManager(const BackendManager &) = delete;
  BackendManager & operator=(const BackendManager &) = delete;

  void observeOdometryQuality(
    const common::Pose & pose, const PoseInformationMatrix & information) override;
  common::Pose mapToOdometry() const;

  BackendUpdate addOdometryKeyframe(
    double timestamp, const common::Pose & pose_WI) override;
  std::optional<BackendUpdate> addLoopConstraint(
    common::KeyframeId from_id,
    common::KeyframeId to_id,
    const common::Pose & T_Ia_Ib,
    const PoseInformationMatrix & information) override;
  bool enqueueLoopConstraint(
    common::KeyframeId from_id, common::KeyframeId to_id,
    const common::Pose & T_Ia_Ib,
    const PoseInformationMatrix & information) override;
  std::vector<BackendUpdate> pollCompleted() override;
  BackendWorkerStats workerStats() const override;
  void cancelPending() override;

  PoseGraph graph() const;
  std::vector<Keyframe> optimizedKeyframes() const;
  std::pair<std::vector<Keyframe>, std::vector<Keyframe>> previewKeyframes() const;

private:
  struct EdgeKey
  {
    common::KeyframeId first{0U};
    common::KeyframeId second{0U};
    bool operator==(const EdgeKey & other) const
    {return first == other.first && second == other.second;}
  };
  struct EdgeKeyHash
  {
    std::size_t operator()(const EdgeKey & key) const
    {
      return std::hash<common::KeyframeId>{}(key.first) ^
             (std::hash<common::KeyframeId>{}(key.second) << 1U);
    }
  };
  struct OptimizationTask
  {
    PoseGraphConstraint constraint;
    std::chrono::steady_clock::time_point deadline;
    std::uint64_t cancellation_generation{0U};
  };

  BackendUpdate makeUpdateUnlocked() const;
  PoseInformationMatrix odometryInformation(const common::Pose & current_pose) const;
  EdgeKey edgeKey(common::KeyframeId from_id, common::KeyframeId to_id) const;
  std::optional<BackendUpdate> processLoopConstraint(
    const PoseGraphConstraint & constraint,
    std::optional<std::chrono::steady_clock::time_point> deadline,
    bool edge_reserved,
    std::optional<std::uint64_t> cancellation_generation = std::nullopt);
  void workerLoop();

  BackendOptions options_;
  KeyframeManager keyframe_manager_;
  PoseGraphOptimizer optimizer_;
  PoseGraph graph_;
  std::vector<Keyframe> optimized_keyframes_;
  // Average covariance-shaped quality penalties, never a sum of correlated
  // scan information. Stored in world axes to survive device rotation.
  PoseInformationMatrix segment_quality_covariance_{PoseInformationMatrix::Zero()};
  std::size_t segment_quality_samples_{0U};
  mutable std::mutex state_mutex_;
  // Serialize synchronous refinement with the online optimization worker.
  std::mutex optimization_mutex_;
  std::unordered_set<EdgeKey, EdgeKeyHash> loop_edges_;
  mutable std::mutex worker_mutex_;
  std::condition_variable worker_condition_;
  std::deque<OptimizationTask> pending_tasks_;
  std::deque<BackendUpdate> completed_updates_;
  BackendWorkerStats worker_stats_;
  std::atomic<std::uint64_t> cancellation_generation_{0U};
  bool stopping_{false};
  std::thread worker_;
};

}  // namespace itera_lio::backend

#endif  // ITERA_LIO__BACKEND__BACKEND_MANAGER_HPP_
