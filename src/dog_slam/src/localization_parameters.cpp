#include "map_localizer_node.hpp"

namespace dog_slam {
void MapLocalizer::configureParameters() {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    cloud_frame_ = declare_parameter<std::string>("cloud_frame", "livox_frame");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/lio/cloud_deskewed");
    declare_parameter<std::string>("map_path", "");
    auto param = [this](const char *key, auto &value) {
      value = declare_parameter<std::decay_t<decltype(value)>>(key, value);
    };
    param("voxel_size", options_.voxel_size); param("coarse_voxel_size", options_.coarse_voxel_size);
    param("crop_radius", options_.crop_radius); param("crop_half_height", options_.crop_half_height);
    param("scan_min_range", options_.scan_min_range); param("scan_max_range", options_.scan_max_range);
    param("min_points", options_.min_points); param("max_scan_points", options_.max_scan_points);
    param("max_map_points", options_.max_map_points); param("coarse_iterations", options_.coarse_iterations);
    param("fine_iterations", options_.fine_iterations); param("coarse_correspondence", options_.coarse_correspondence);
    param("fine_correspondence", options_.fine_correspondence); param("inlier_distance", options_.inlier_distance);
    param("min_overlap", options_.min_overlap); param("min_reverse_overlap", options_.min_reverse_overlap);
    param("max_rmse", options_.max_rmse); param("min_geometry_ratio", options_.min_geometry_ratio);
    param("max_tracking_translation", options_.max_tracking_translation); param("max_tracking_rotation", options_.max_tracking_rotation);
    param("max_tracking_vertical", options_.max_tracking_vertical); param("max_initial_translation", options_.max_initial_translation);
    param("max_initial_rotation", options_.max_initial_rotation); param("max_initial_vertical", options_.max_initial_vertical);
    param("recovery_xy_radius", options_.recovery_xy_radius); param("recovery_z_radius", options_.recovery_z_radius);
    param("recovery_yaw_radius", options_.recovery_yaw_radius); param("recovery_candidates", options_.recovery_candidates);
    param("ambiguity_margin", options_.ambiguity_margin); param("confirmation_translation", options_.confirmation_translation);
    param("confirmation_rotation", options_.confirmation_rotation); param("confirmations", options_.confirmations);
    param("failures_to_lost", options_.failures_to_lost); param("automatic_recovery", options_.automatic_recovery);
    auto &g=options_.global;
    param("global_relocalization.keyframe_candidates",g.keyframe_candidates);
    param("global_relocalization.min_overlap",g.min_overlap);
    param("global_relocalization.enabled",g.enabled);param("global_relocalization.on_startup",g.on_startup);
    param("global_relocalization.on_lost",g.on_lost);param("global_relocalization.xy_step",g.xy_step);
    param("global_relocalization.height_step",g.height_step);param("global_relocalization.body_height",g.body_height);
    param("global_relocalization.yaw_steps",g.yaw_steps);param("global_relocalization.max_hypotheses",g.max_hypotheses);
    param("global_relocalization.scoring_points",g.scoring_points);param("global_relocalization.refine_candidates",g.refine_candidates);
    param("global_relocalization.time_budget",g.time_budget);param("global_relocalization.failures_before_global",g.failures_before_global);
    options_.validate();
    rate_ = declare_parameter<double>("registration_rate", 2.0);
    timeout_ = declare_parameter<double>("data_timeout", 1.5);
    max_scan_age_ = declare_parameter<double>("max_scan_age", 1.0);
    max_result_age_ = declare_parameter<double>("max_result_age", 2.0);
    recovery_interval_ = declare_parameter<double>("recovery_interval", 1.0);
    global_retry_interval_ = declare_parameter<double>("global_relocalization.retry_interval",3.0);
    if(!std::isfinite(global_retry_interval_)||global_retry_interval_<=0||
       (options_.global.enabled&&options_.global.time_budget>=max_result_age_))
      throw std::invalid_argument("Global search budget must be shorter than max_result_age");
    odom_jump_ = declare_parameter<double>("odom_jump_tolerance", .3);
    odom_speed_ = declare_parameter<double>("max_odom_speed", 3.0);
    odom_rotation_ = declare_parameter<double>("max_odom_rotation_rate", 4.0);
    for (double value : {rate_, timeout_, max_scan_age_, max_result_age_, recovery_interval_, odom_jump_, odom_speed_, odom_rotation_})
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("Invalid timing/odometry limit");
    if (map_frame_.empty() || odom_frame_.empty() || base_frame_.empty() || cloud_frame_.empty() ||
        map_frame_ == odom_frame_ || map_frame_ == base_frame_ || odom_frame_ == base_frame_)
      throw std::invalid_argument("Localization frames must be distinct and nonempty");
}
}  // namespace dog_slam
