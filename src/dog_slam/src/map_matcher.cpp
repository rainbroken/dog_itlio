#include "dog_slam/map_localization.hpp"

#include <Eigen/Eigenvalues>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <array>

namespace dog_slam {
namespace {
Eigen::Vector3d xyz(const pcl::PointXYZ &p) { return {p.x, p.y, p.z}; }
LocalizationCloud::Ptr voxel(LocalizationCloud::ConstPtr input, double size, int cap = 0) {
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(size, size, size); filter.setInputCloud(input);
  auto out = std::make_shared<LocalizationCloud>(); filter.filter(*out);
  if (cap > 0 && out->size() > static_cast<size_t>(cap)) {
    auto sampled = std::make_shared<LocalizationCloud>(); sampled->reserve(cap);
    for (int i = 0; i < cap; ++i) sampled->push_back((*out)[static_cast<size_t>(i) * out->size() / cap]);
    return sampled;
  }
  return out;
}
Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d out;
  out << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return out;
}
}

struct MapMatcher::Impl {
  LocalizationOptions options;
  LocalizationCloud::Ptr map;
  pcl::PointCloud<pcl::Normal>::Ptr normals;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;

  explicit Impl(LocalizationCloud::ConstPtr input, LocalizationOptions config) : options(config) {
    options.validate();
    if (!input || input->size() < static_cast<size_t>(options.min_points) || input->size() > 8000000)
      throw std::invalid_argument("Map has too few or too many points");
    auto clean = std::make_shared<LocalizationCloud>(); clean->reserve(input->size());
    for (const auto &p : *input) {
      if (!xyz(p).allFinite()) continue;
      if (xyz(p).cwiseAbs().maxCoeff() > 100000) throw std::invalid_argument("Use local map coordinates in meters");
      clean->push_back(p);
    }
    map = voxel(clean, options.voxel_size);
    if (map->size() < static_cast<size_t>(options.min_points) || map->size() > static_cast<size_t>(options.max_map_points))
      throw std::invalid_argument("Localization map size outside configured limits after voxel filtering");
    tree.setInputCloud(map);
    normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> estimator;
    estimator.setInputCloud(map); estimator.setKSearch(15); estimator.compute(*normals);
  }

  LocalizationCloud::Ptr target(const Pose3 &guess) const {
    std::vector<int> indices; std::vector<float> distances;
    const Eigen::Vector3d p = guess.translation();
    pcl::PointXYZ center(p.x(), p.y(), p.z());
    tree.radiusSearch(center, std::hypot(options.crop_radius, options.crop_half_height), indices, distances);
    auto crop = std::make_shared<LocalizationCloud>(); crop->reserve(indices.size());
    for (int i : indices) {
      const Eigen::Vector3d delta = xyz((*map)[i]) - p;
      if (delta.head<2>().norm() <= options.crop_radius && std::abs(delta.z()) <= options.crop_half_height)
        crop->push_back((*map)[i]);
    }
    return crop;
  }

  LocalizationMatch evaluate(LocalizationCloud::ConstPtr scan, const Pose3 &pose, const Pose3 &guess,
                             bool recovery, bool converged) const {
    LocalizationMatch result; result.map_to_base = pose;
    if (!converged || !validPose(pose)) { result.reason = "registration_not_converged"; return result; }
    const Eigen::Vector3d delta = pose.translation() - guess.translation();
    if (delta.norm() > (recovery ? options.max_initial_translation : options.max_tracking_translation) ||
        std::abs(delta.z()) > (recovery ? options.max_initial_vertical : options.max_tracking_vertical) ||
        rotationDistance(pose, guess) > (recovery ? options.max_initial_rotation : options.max_tracking_rotation)) {
      result.reason = "correction_exceeds_limits"; return result;
    }
    LocalizationCloud::Ptr transformed(new LocalizationCloud());
    pcl::transformPointCloud(*scan, *transformed, pose.matrix().cast<float>());
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
    double squared_error = 0, radius_squared = 0;
    for (const auto &p : *scan) radius_squared += xyz(p).squaredNorm();
    const double scale = std::max(1.0, std::sqrt(radius_squared / scan->size()));
    int normal_count = 0;
    const double threshold = options.inlier_distance * options.inlier_distance;
    std::vector<int> index(1); std::vector<float> distance(1);
    for (const auto &p : *transformed) {
      if (tree.nearestKSearch(p, 1, index, distance) != 1 || distance[0] > threshold) continue;
      ++result.inliers; squared_error += distance[0];
      const auto &normal = (*normals)[index[0]];
      Eigen::Vector3d n(normal.normal_x, normal.normal_y, normal.normal_z);
      if (!n.allFinite() || n.norm() < .8) continue;
      n.normalize();
      Eigen::Matrix<double, 1, 6> jacobian;
      jacobian.head<3>() = -n.transpose() * skew((xyz(p) - pose.translation()) / scale);
      jacobian.tail<3>() = n.transpose();
      information.noalias() += jacobian.transpose() * jacobian; ++normal_count;
    }
    result.overlap = static_cast<double>(result.inliers) / scan->size();
    result.rmse = result.inliers ? std::sqrt(squared_error / result.inliers) : options.fine_correspondence;
    if (result.inliers < options.min_points || result.overlap < options.min_overlap || result.rmse > options.max_rmse) {
      result.reason = "insufficient_overlap_or_large_residual"; return result;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(information);
    if (normal_count < options.min_points || eigen.info() != Eigen::Success || eigen.eigenvalues()[5] <= 0) {
      result.reason = "insufficient_geometry"; return result;
    }
    result.geometry_ratio = std::max(0.0, eigen.eigenvalues()[0]) / eigen.eigenvalues()[5];
    if (result.geometry_ratio < options.min_geometry_ratio) { result.reason = "degenerate_geometry"; return result; }
    // Reverse overlap includes only observed angular bins up to their first
    // return. A map AABB alone also counts occluded/unobserved surfaces and
    // rejects sparse real scans even at the correct pose.
    constexpr double pi=3.14159265358979323846;
    auto angularBin=[&](const Eigen::Vector3d &p) {
      const double r=p.norm();
      if(r<1e-6)return -1;
      const int az=std::clamp(int((std::atan2(p.y(),p.x())+pi)*180/pi),0,359);
      const int el=std::clamp(int((std::asin(std::clamp(p.z()/r,-1.,1.))+pi/2)*180/pi),0,179);
      return el*360+az;
    };
    std::array<double,360*180> depth;depth.fill(std::numeric_limits<double>::infinity());
    for(const auto &p:*scan) {
      const auto v=xyz(p);const int bin=angularBin(v);
      if(bin>=0)depth[bin]=std::min(depth[bin],v.norm());
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> scan_tree; scan_tree.setInputCloud(transformed);
    pcl::PointXYZ low, high; pcl::getMinMax3D(*transformed, low, high);
    int covered = 0, local_points = 0;
    auto local = target(pose);
    const auto inverse_pose=pose.inverse();
    for (const auto &p : *local) {
      if (p.x < low.x || p.x > high.x || p.y < low.y || p.y > high.y || p.z < low.z || p.z > high.z) continue;
      const Eigen::Vector3d v=inverse_pose*xyz(p);const int bin=angularBin(v);
      if(bin<0||!std::isfinite(depth[bin])||v.norm()>depth[bin]+options.inlier_distance)continue;
      ++local_points;
      if (scan_tree.nearestKSearch(p, 1, index, distance) == 1 && distance[0] <= threshold) ++covered;
    }
    result.reverse_overlap = local_points ? static_cast<double>(covered) / local_points : 0;
    if (result.reverse_overlap < options.min_reverse_overlap) { result.reason = "low_reverse_overlap"; return result; }
    result.score = result.rmse + .2 * (1 - result.overlap) + .1 * (1 - result.reverse_overlap);
    result.accepted = true; result.reason = "accepted"; return result;
  }

  LocalizationMatch align(LocalizationCloud::ConstPtr scan, LocalizationCloud::ConstPtr crop,
                          const Pose3 &hypothesis, const Pose3 &prior, bool recovery) const {
    LocalizationMatch rejected; rejected.reason = "local_map_too_small";
    if (crop->size() < static_cast<size_t>(options.min_points)) return rejected;
    // Center the target before float ICP; odom/map origins need not coincide.
    const Eigen::Vector3d origin = prior.translation();
    Pose3 shift = Pose3::Identity(); shift.translation() = -origin;
    auto target_local = std::make_shared<LocalizationCloud>();
    pcl::transformPointCloud(*crop, *target_local, shift.matrix().cast<float>());
    auto coarse_scan = voxel(scan, options.coarse_voxel_size);
    auto coarse_target = voxel(target_local, options.coarse_voxel_size);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(coarse_scan); icp.setInputTarget(coarse_target);
    icp.setMaximumIterations(options.coarse_iterations); icp.setMaxCorrespondenceDistance(options.coarse_correspondence);
    icp.setTransformationEpsilon(1e-7); icp.setEuclideanFitnessEpsilon(1e-7);
    LocalizationCloud aligned;
    icp.align(aligned, (shift * hypothesis).matrix().cast<float>());
    if (!icp.hasConverged()) { rejected.reason = "coarse_not_converged"; return rejected; }
    Eigen::Matrix4f estimate = icp.getFinalTransformation();
    icp.setInputSource(scan); icp.setInputTarget(target_local);
    icp.setMaximumIterations(options.fine_iterations); icp.setMaxCorrespondenceDistance(options.fine_correspondence);
    icp.align(aligned, estimate);
    Pose3 pose = Pose3::Identity(); pose.matrix() = icp.getFinalTransformation().cast<double>();
    pose = shift.inverse() * pose;
    // Quaternion normalization removes float orthogonality drift.
    Eigen::Quaterniond q(pose.linear()); pose.linear() = q.normalized().toRotationMatrix();
    return evaluate(scan, pose, prior, recovery, icp.hasConverged());
  }
};

MapMatcher::MapMatcher(LocalizationCloud::ConstPtr map, LocalizationOptions options)
    : impl_(std::make_unique<Impl>(std::move(map), options)) {}
MapMatcher::~MapMatcher() = default;
LocalizationCloud::ConstPtr MapMatcher::map() const { return impl_->map; }
pcl::PointCloud<pcl::Normal>::ConstPtr MapMatcher::normals() const { return impl_->normals; }

LocalizationMatch MapMatcher::refine(LocalizationCloud::ConstPtr input,const Pose3 &guess) const {
  auto clean=std::make_shared<LocalizationCloud>();
  const auto &o=impl_->options;
  if(input)for(const auto &p:*input) {
    const auto v=xyz(p);
    if(v.allFinite()&&v.norm()>=o.scan_min_range&&v.norm()<=o.scan_max_range)clean->push_back(p);
  }
  auto scan=voxel(clean,o.voxel_size,o.max_scan_points);
  if(!validPose(guess)||scan->size()<size_t(o.min_points)) {
    LocalizationMatch rejected;rejected.reason="scan_too_small_or_invalid_pose";return rejected;
  }
  return impl_->align(scan,impl_->target(guess),guess,guess,true);
}

LocalizationMatch MapMatcher::match(LocalizationCloud::ConstPtr input, const Pose3 &guess, bool recovery) const {
  const auto &o = impl_->options;
  LocalizationMatch rejected;
  if (!input || !validPose(guess)) { rejected.reason = "invalid_input_or_pose"; return rejected; }
  auto clean = std::make_shared<LocalizationCloud>(); clean->reserve(input->size());
  for (const auto &p : *input) {
    const auto v = xyz(p); const double range = v.norm();
    if (v.allFinite() && range >= o.scan_min_range && range <= o.scan_max_range) clean->push_back(p);
  }
  if (clean->size() < static_cast<size_t>(o.min_points)) { rejected.reason = "scan_too_small"; return rejected; }
  auto scan = voxel(clean, o.voxel_size, o.max_scan_points);
  if (scan->size() < static_cast<size_t>(o.min_points)) { rejected.reason = "scan_too_small"; return rejected; }
  auto crop = impl_->target(guess);
  if (!recovery) return impl_->align(scan, crop, guess, guess, false);
  // Bounded XYZ/yaw hypotheses around the seed or the last odometry prediction.
  // This deliberately does not claim unknown-position global localization.
  std::vector<Pose3> hypotheses;
  for (double x : {0., -o.recovery_xy_radius, o.recovery_xy_radius})
    for (double y : {0., -o.recovery_xy_radius, o.recovery_xy_radius})
      for (double yaw : {0., -o.recovery_yaw_radius, o.recovery_yaw_radius}) {
        Pose3 p = guess; p.translation() += Eigen::Vector3d(x, y, 0);
        p.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * guess.linear();
        hypotheses.push_back(p);
      }
  for (double z : {-o.recovery_z_radius, o.recovery_z_radius})
    for (double yaw : {0., -o.recovery_yaw_radius, o.recovery_yaw_radius}) {
      Pose3 p = guess; p.translation().z() += z;
      p.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * guess.linear();
      hypotheses.push_back(p);
    }
  auto scoring_scan = voxel(scan, o.coarse_voxel_size, 500);
  std::vector<std::pair<double, size_t>> ranked;
  std::vector<int> indices(1); std::vector<float> distances(1);
  for (size_t i = 0; i < hypotheses.size(); ++i) {
    double score = 0;
    for (const auto &p : *scoring_scan) {
      const auto v = hypotheses[i] * xyz(p); pcl::PointXYZ q(v.x(), v.y(), v.z());
      if (impl_->tree.nearestKSearch(q, 1, indices, distances) != 1) score += o.coarse_correspondence;
      else score += std::min(o.coarse_correspondence, std::sqrt(static_cast<double>(distances[0])));
    }
    ranked.emplace_back(score / scoring_scan->size(), i);
  }
  std::sort(ranked.begin(), ranked.end());
  std::vector<LocalizationMatch> accepted;
  for (int i = 0; i < std::min(o.recovery_candidates, static_cast<int>(ranked.size())); ++i) {
    auto result = impl_->align(scan, crop, hypotheses[ranked[i].second], guess, true);
    if (!result.accepted) { rejected = result; continue; }
    bool duplicate = false;
    for (auto &other : accepted) if ((other.map_to_base.translation() - result.map_to_base.translation()).norm() < .25 &&
                                    rotationDistance(other.map_to_base, result.map_to_base) < .12) {
      if (result.score < other.score) other = result;
      duplicate = true; break;
    }
    if (!duplicate) accepted.push_back(result);
  }
  if (accepted.empty()) { rejected.hypotheses = o.recovery_candidates; return rejected; }
  std::sort(accepted.begin(), accepted.end(), [](const auto &a, const auto &b) { return a.score < b.score; });
  auto result = accepted.front(); result.hypotheses = o.recovery_candidates;
  if (accepted.size() > 1 && accepted[1].score - result.score < o.ambiguity_margin) {
    result.accepted = false; result.reason = "ambiguous_relocalization";
  }
  return result;
}
}  // namespace dog_slam
