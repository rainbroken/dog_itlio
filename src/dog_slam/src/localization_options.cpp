#include "dog_slam/map_localization.hpp"
#include <cmath>
#include <stdexcept>

namespace dog_slam {
bool validPose(const Pose3 &pose) {
  return pose.matrix().allFinite() && pose.translation().cwiseAbs().maxCoeff() < 100000 &&
      (pose.linear().transpose() * pose.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-5) &&
      std::abs(pose.linear().determinant() - 1) < 1e-5 &&
      pose.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-8);
}
double rotationDistance(const Pose3 &a, const Pose3 &b) {
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle();
}

void LocalizationOptions::validate() const {
  const double positive[] = {voxel_size, coarse_voxel_size, crop_radius, crop_half_height,
    scan_min_range, scan_max_range, coarse_correspondence, fine_correspondence, inlier_distance,
    min_overlap, min_reverse_overlap, max_rmse, min_geometry_ratio, max_tracking_translation,
    max_tracking_rotation, max_tracking_vertical, max_initial_translation, max_initial_rotation,
    max_initial_vertical, recovery_xy_radius, recovery_z_radius, recovery_yaw_radius,
    ambiguity_margin, confirmation_translation, confirmation_rotation};
  for (double value : positive) if (!std::isfinite(value) || value <= 0)
    throw std::invalid_argument("Localization limits must be finite and positive");
  if (min_points < 20 || max_scan_points < min_points || max_scan_points > 20000 ||
      max_map_points < min_points || max_map_points > 2000000 || coarse_iterations < 1 ||
      fine_iterations < 1 || coarse_iterations > 100 || fine_iterations > 100 ||
      recovery_candidates < 1 || recovery_candidates > 12 || confirmations < 1 || failures_to_lost < 1 ||
      min_overlap > 1 || min_reverse_overlap > 1 || min_geometry_ratio >= 1 ||
      voxel_size > coarse_voxel_size || scan_min_range >= scan_max_range ||
      fine_correspondence > coarse_correspondence || inlier_distance > fine_correspondence)
    throw std::invalid_argument("Inconsistent localization configuration");
  for(double v:{global.xy_step,global.height_step,global.body_height,global.time_budget})
    if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Invalid global search limits");
  if(!std::isfinite(global.min_overlap)||global.min_overlap<=0||global.min_overlap>1||
      global.xy_step>max_initial_translation||global.height_step>max_initial_vertical||global.body_height>2||
      global.yaw_steps<8||global.yaw_steps>72||global.max_hypotheses<global.yaw_steps||global.max_hypotheses>200000||
      global.scoring_points<32||global.scoring_points>512||global.refine_candidates<2||global.refine_candidates>32||
      global.keyframe_candidates<2||global.keyframe_candidates>256||
      global.failures_before_global<1||global.time_budget>10)
    throw std::invalid_argument("Inconsistent global search configuration");
}

}  // namespace dog_slam
