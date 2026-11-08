#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>
#include <vector>

namespace dog_slam {
using LocalizationCloud = pcl::PointCloud<pcl::PointXYZ>;
using Pose3 = Eigen::Isometry3d;

struct GlobalRelocalizationOptions {
  bool enabled = false;
  bool on_startup = true;
  bool on_lost = true;
  double xy_step = 1.0;
  double height_step = .5;
  double body_height = .65;
  int yaw_steps = 24;
  int max_hypotheses = 30000;
  int scoring_points = 128;
  int refine_candidates = 12;
  double time_budget = 1.0;
  int failures_before_global = 3;
  int keyframe_candidates = 128;
  double min_overlap = .80;
};

struct LocalizationOptions {
  double voxel_size = .15;
  double coarse_voxel_size = .35;
  double crop_radius = 15;
  double crop_half_height = 4;
  double scan_min_range = .4;
  double scan_max_range = 20;
  int min_points = 80;
  int max_scan_points = 4000;
  int max_map_points = 1000000;
  int coarse_iterations = 25;
  int fine_iterations = 35;
  double coarse_correspondence = 1.5;
  double fine_correspondence = .5;
  double inlier_distance = .25;
  double min_overlap = .55;
  double min_reverse_overlap = .30;
  double max_rmse = .15;
  double min_geometry_ratio = .0005;
  double max_tracking_translation = .5;
  double max_tracking_rotation = .25;
  double max_tracking_vertical = .25;
  double max_initial_translation = 1.5;
  double max_initial_rotation = .8;
  double max_initial_vertical = .6;
  double recovery_xy_radius = 1;
  double recovery_z_radius = .3;
  double recovery_yaw_radius = .52;
  int recovery_candidates = 4;
  double ambiguity_margin = .015;
  double confirmation_translation = .15;
  double confirmation_rotation = .10;
  int confirmations = 3;
  int failures_to_lost = 3;
  bool automatic_recovery = true;
  GlobalRelocalizationOptions global;
  void validate() const;
};

struct LocalizationMatch {
  bool accepted = false;
  std::string reason = "not_processed";
  Pose3 map_to_base = Pose3::Identity();
  double rmse = 0;
  double overlap = 0;
  double reverse_overlap = 0;
  double geometry_ratio = 0;
  int inliers = 0;
  int hypotheses = 0;
  double score = 0;
  std::string scope = "bounded_seed_neighborhood";
  double duration_ms = 0;
};

// Stateless registration against an immutable map. All returned transforms map
// body coordinates into map coordinates; no planar projection is performed.
class MapMatcher {
public:
  MapMatcher(LocalizationCloud::ConstPtr map, LocalizationOptions options = {});
  ~MapMatcher();
  LocalizationMatch match(LocalizationCloud::ConstPtr scan_base, const Pose3 &guess,
                          bool recovery) const;
  LocalizationCloud::ConstPtr map() const;
  pcl::PointCloud<pcl::Normal>::ConstPtr normals() const;
  // Refine a single map-wide candidate with the same quality/correction gates.
  LocalizationMatch refine(LocalizationCloud::ConstPtr scan_base, const Pose3 &guess) const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

enum class LocalizationState { WaitingForPose, Initializing, Tracking, Degraded, Lost, Relocalizing, GlobalRelocalizing };
const char *localizationStateName(LocalizationState state);

struct LocalizationUpdate {
  LocalizationState state = LocalizationState::WaitingForPose;
  bool valid = false;
  bool has_prior = false;
  int confirmations = 0;
  int failures = 0;
  Pose3 map_to_odom = Pose3::Identity();
  LocalizationMatch match;
  std::string source = "none";
};

// Single worker owns this engine. ROS lifecycle, freshness and TF lookups live
// outside it, making state transitions testable without a robot or ROS graph.
class GlobalRelocalizer;
class LocalizationEngine {
public:
  LocalizationEngine(LocalizationCloud::ConstPtr map, LocalizationOptions options = {},
                     std::vector<Pose3> keyframes = {});
  ~LocalizationEngine();
  void initialize(const Pose3 &map_to_base, const Pose3 &odom_to_base);
  void reset();
  void invalidate(const std::string &reason);
  bool relocalize();
  bool globalRelocalize();
  LocalizationUpdate update(LocalizationCloud::ConstPtr scan_base, const Pose3 &odom_to_base);
  const LocalizationUpdate &status() const { return status_; }
  LocalizationCloud::ConstPtr map() const { return matcher_.map(); }
private:
  MapMatcher matcher_;
  LocalizationOptions options_;
  LocalizationUpdate status_;
  Pose3 pending_map_to_odom_ = Pose3::Identity();
  std::unique_ptr<GlobalRelocalizer> global_;
  bool global_requested_ = false;
};

LocalizationCloud::Ptr readLocalizationMap(const std::string &path);
// Optional map-frame IMU keyframe poses, used as approximate body search seeds.
std::vector<Pose3> readLocalizationKeyframes(const std::string &path);
bool validPose(const Pose3 &pose);
double rotationDistance(const Pose3 &a, const Pose3 &b);
}  // namespace dog_slam
