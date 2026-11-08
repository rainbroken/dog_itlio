#include "dog_slam/map_localization.hpp"
#include "dog_slam/global_relocalizer.hpp"
#include <stdexcept>

namespace dog_slam {
const char *localizationStateName(LocalizationState state) {
  switch (state) {
    case LocalizationState::WaitingForPose: return "waiting_initial_pose";
    case LocalizationState::Initializing: return "initializing";
    case LocalizationState::Tracking: return "tracking";
    case LocalizationState::Degraded: return "degraded";
    case LocalizationState::Lost: return "lost";
    case LocalizationState::Relocalizing: return "relocalizing";
    case LocalizationState::GlobalRelocalizing: return "global_relocalizing";
  }
  return "unknown";
}
LocalizationEngine::LocalizationEngine(LocalizationCloud::ConstPtr map, LocalizationOptions options,
                                       std::vector<Pose3> keyframes)
    : matcher_(std::move(map), options), options_(options) {
  if(options.global.enabled)global_=std::make_unique<GlobalRelocalizer>(matcher_,options,std::move(keyframes));
  reset();
}
LocalizationEngine::~LocalizationEngine()=default;
void LocalizationEngine::reset() {
  status_ = LocalizationUpdate{}; pending_map_to_odom_.setIdentity();
  global_requested_=options_.global.enabled&&options_.global.on_startup;
  if(global_requested_)status_.state=LocalizationState::GlobalRelocalizing;
}
void LocalizationEngine::initialize(const Pose3 &map_to_base, const Pose3 &odom_to_base) {
  if (!validPose(map_to_base) || !validPose(odom_to_base)) throw std::invalid_argument("Invalid initial SE(3) pose");
  reset(); status_.has_prior = true; status_.state = LocalizationState::Initializing;
  status_.source="initial_pose";
  global_requested_=false; // An explicit initial pose always takes priority.
  status_.map_to_odom = map_to_base * odom_to_base.inverse(); pending_map_to_odom_ = status_.map_to_odom;
}
void LocalizationEngine::invalidate(const std::string &reason) {
  status_.valid = false; status_.confirmations = 0; status_.failures = options_.failures_to_lost;
  status_.state = status_.has_prior ? LocalizationState::Lost : LocalizationState::WaitingForPose;
  status_.match.accepted = false; status_.match.reason = reason;
  pending_map_to_odom_ = status_.map_to_odom;
}
bool LocalizationEngine::relocalize() {
  if (!status_.has_prior) return false;
  global_requested_=false;
  invalidate("relocalization_requested"); status_.state = LocalizationState::Relocalizing; return true;
}
bool LocalizationEngine::globalRelocalize() {
  if(!global_)return false;
  global_requested_=true;status_.valid=false;status_.confirmations=0;
  status_.state=LocalizationState::GlobalRelocalizing;status_.match.reason="global_relocalization_requested";
  return true;
}
LocalizationUpdate LocalizationEngine::update(LocalizationCloud::ConstPtr scan, const Pose3 &odom_to_base) {
  if (!validPose(odom_to_base)) { invalidate("invalid_odometry"); return status_; }
  const bool fallback=global_&&options_.global.on_lost&&options_.automatic_recovery&&
    status_.state==LocalizationState::Lost&&status_.failures>=options_.failures_to_lost+options_.global.failures_before_global;
  const bool global_search=global_requested_||fallback;
  if(global_search) {
    global_requested_=true;status_.state=LocalizationState::GlobalRelocalizing;
    auto result=global_->search(scan,odom_to_base,matcher_);
    if(!result.accepted) {
      status_.match=result;status_.valid=false;status_.confirmations=0;++status_.failures;return status_;
    }
    initialize(result.map_to_base,odom_to_base);
    status_.source=result.scope=="historical_keyframe_search" ? "historical_keyframes" : "global_map_search";
    status_.state=LocalizationState::Relocalizing;status_.match=result;
  } else if(!status_.has_prior)return status_;
  bool searching = status_.state == LocalizationState::Initializing || status_.state == LocalizationState::Relocalizing ||
                   status_.state == LocalizationState::Lost;
  if (status_.state == LocalizationState::Lost && !options_.automatic_recovery) return status_;
  const Pose3 prior = status_.confirmations > 0 && !status_.valid ? pending_map_to_odom_ : status_.map_to_odom;
  if(!global_search)status_.match = matcher_.match(scan, prior * odom_to_base, searching);
  if (!status_.match.accepted) {
    status_.valid = false; status_.confirmations = 0; ++status_.failures;
    if (status_.failures >= options_.failures_to_lost) status_.state = LocalizationState::Lost;
    else if (!searching) status_.state = LocalizationState::Degraded;
    return status_;
  }
  const Pose3 correction = status_.match.map_to_base * odom_to_base.inverse();
  const bool continuing = status_.valid && status_.state == LocalizationState::Tracking;
  if (!continuing) {
    bool consistent = status_.confirmations > 0 &&
        // Compare the correction at the current body, not at the arbitrary odom
        // origin: a small angular update must not look like a huge translation
        // merely because that origin is far from the robot.
        ((correction*odom_to_base).translation()-(pending_map_to_odom_*odom_to_base).translation()).norm() <= options_.confirmation_translation &&
        rotationDistance(correction, pending_map_to_odom_) <= options_.confirmation_rotation;
    status_.confirmations = consistent ? status_.confirmations + 1 : 1;
    pending_map_to_odom_ = correction;
    status_.valid = status_.confirmations >= options_.confirmations;
  }
  status_.failures = 0;
  if (status_.valid) { status_.map_to_odom = correction; status_.state = LocalizationState::Tracking; }
  return status_;
}
}  // namespace dog_slam
