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

#ifndef ITERA_LIO__SYSTEM__LIO_PIPELINE_HPP_
#define ITERA_LIO__SYSTEM__LIO_PIPELINE_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "itera_lio/common/geometry.hpp"
#include "itera_lio/common/map_types.hpp"
#include "itera_lio/common/observation_types.hpp"
#include "itera_lio/backend/backend_manager.hpp"
#include "itera_lio/estimator/esikf.hpp"
#include "itera_lio/estimator/degeneracy_detector.hpp"
#include "itera_lio/estimator/tracking_state.hpp"
#include "itera_lio/frontend/global_map.hpp"
#include "itera_lio/frontend/ikd_map.hpp"
#include "itera_lio/frontend/scan_to_map.hpp"
#include "itera_lio/frontend/local_map_manager.hpp"
#include "itera_lio/loop/loop_closure_manager.hpp"


namespace itera_lio::system
{

struct LioPipelineOptions
{
  backend::BackendOptions backend;
  frontend::ScanToMapOptions registration;
  estimator::EsikfOptions correction;
  estimator::DegeneracyOptions degeneracy;
  estimator::TrackingStateOptions tracking;
  float scan_voxel_size{0.2F};
  // Set when the caller already sampled the scan using scan_voxel_size.
  // This avoids a second identical voxel pass after deskewing.
  bool input_prevoxelized{false};
  float map_voxel_size{0.3F};
  frontend::IkdRebuildPolicy map_rebuild_policy{frontend::IkdRebuildPolicy::kHybrid};
  std::size_t map_async_rebuild_min_subtree_points{20000U};
  frontend::LocalMapOptions local_map;
  loop::LoopClosureOptions loop_closure;
};

struct PipelineResult
{
  bool initialized_map{false};
  bool corrected{false};
  bool map_updated{false};
  std::string map_insert_reason{"not_corrected"};
  double frame_translation_correction{0.0};
  double frame_rotation_correction{0.0};
  bool temporary_map_updated{false};
  common::RegistrationReport registration;
  estimator::EsikfReport correction;
  estimator::DegeneracyReport degeneracy;
  estimator::TrackingStateReport tracking;
  backend::BackendUpdate backend;
  std::size_t map_size{0U};
  std::size_t temporary_map_size{0U};
  std::size_t global_map_size{0U};
  std::size_t dense_archive_size{0U};
  std::size_t registration_input_points{0U};
  std::size_t map_candidate_points{0U};
  std::size_t map_inserted_points{0U};
  std::size_t map_replaced_points{0U};
  std::size_t map_rejected_points{0U};
  std::size_t map_compacted_points{0U};
  std::size_t temporary_map_inserted_points{0U};
  std::size_t archive_inserted_points{0U};
  std::size_t dense_archive_inserted_points{0U};
  // Actual global-archive representatives inserted or replaced this frame.
  // This is intentionally separate from local-map updates: the archive has
  // its own voxel resolution and replacement policy.
  std::vector<common::MapPoint> archive_updates;
  // World-frame points actually upserted into the local map during this frame.
  std::vector<common::MapPoint> map_updates;
  double registration_time_ms{0.0};
  double correction_time_ms{0.0};
  double local_map_update_time_ms{0.0};
  double map_delete_time_ms{0.0};
  double archive_update_time_ms{0.0};
  double map_insert_time_ms{0.0};
  double backend_time_ms{0.0};
  double loop_closure_time_ms{0.0};
  double map_update_time_ms{0.0};
  double total_time_ms{0.0};
  std::size_t deleted_map_points{0U};
  std::size_t pending_delete_boxes{0U};
  std::size_t loop_candidates{0U};
  std::size_t loop_verifications{0U};
  std::size_t loop_inliers{0U};
  common::KeyframeId loop_current_id{0U};
  common::KeyframeId loop_matched_id{0U};
  std::size_t loop_accepted_verifications{0U};
  std::size_t loop_rejected_verifications{0U};
  std::vector<loop::LoopVerificationResult> loop_verification_results;
  double loop_fitness{0.0};
  double loop_translation_correction{0.0};
  double loop_rotation_correction{0.0};
  const char * loop_icp_initialization{"none"};
  const char * loop_rejection_reason{"none"};
  std::size_t stored_loop_keyframe_clouds{0U};
  loop::LoopWorkerStats loop_worker;
  backend::BackendWorkerStats backend_worker;
  frontend::IkdRebuildStats ikd_rebuild;
  frontend::GlobalMapStats global_map_stats;
};

struct OptimizedLoopRefinementReport
{
  std::string error;
  std::size_t verifications{0U};
  std::size_t accepted{0U};
  std::size_t committed{0U};
  std::vector<loop::LoopVerificationResult> results;
};

class LioPipeline
{
public:
  explicit LioPipeline(const LioPipelineOptions & options = {});
  // Sink pointers are non-owning. A null pointer selects the built-in
  // BackendManager/LoopClosureManager implementations.
  LioPipeline(
    const LioPipelineOptions & options,
    backend::BackendSink * backend_sink,
    loop::LoopClosureSink * loop_sink);

  PipelineResult processScan(
    const sensor::LidarFrame & scan,
    estimator::NominalState & state,
    estimator::StateCovariance & covariance,
    const estimator::FixedExtrinsics & extrinsics = {},
    const sensor::LidarFrame * dense_scan = nullptr);

  const frontend::IkdMap & map() const {return map_;}
  const frontend::IkdMap & temporaryMap() const {return temporary_map_;}
  const frontend::LocalMapManager & localMap() const {return local_map_;}
  const backend::BackendManager & backend() const {return backend_;}
  const loop::LoopClosureManager & loopClosure() const {return loop_closure_;}
  bool getOptimizedMapPoints(
    std::vector<common::MapPoint> & points, float voxel_size) const;
  OptimizedLoopRefinementReport refineOptimizedLoops(const std::string & cloud_archive = {});

private:
  void updateBackendAndLoop(
    const sensor::LidarFrame & scan,
    const estimator::NominalState & state,
    const estimator::FixedExtrinsics & extrinsics,
  PipelineResult & result);

  LioPipelineOptions options_;
  frontend::IkdMap map_;
  frontend::IkdMap temporary_map_;
  frontend::LocalMapManager local_map_;
  frontend::LocalMapManager temporary_local_map_;
  backend::BackendManager backend_;
  loop::LoopClosureManager loop_closure_;
  estimator::TrackingStateMachine tracking_state_;
  backend::BackendSink * backend_sink_{nullptr};
  loop::LoopClosureSink * loop_sink_{nullptr};
  bool map_initialized_{false};
  loop::LoopConfirmationGate loop_confirmation_gate_;
};

}  // namespace itera_lio::system

#endif  // ITERA_LIO__SYSTEM__LIO_PIPELINE_HPP_
