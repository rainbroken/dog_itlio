#pragma once
#include <Eigen/Geometry>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace dog_navigation {
using Point = Eigen::Vector3d;
using Points = std::vector<Point>;
using Pose = Eigen::Isometry3d;
struct Options {
  double voxel = .10;
  double range = 8.;
  double vertical_range = 4.;
  double min_range = .4;
  double padding = .12;
  double obstacle_bottom = .12;
  double static_tolerance = .08;
  double avoidance_distance = .45;
  double avoidance_weight = 8.;
  double local_length = 3.;
  int max_rays = 20000;
  int max_voxels = 120000;
  void validate() const;
};

// Stored in continuous odom coordinates. Obstacles disappear only with positive
// ray evidence, or when leaving the bounded local window; no TTL-to-free rule.
class ObstacleMemory {
public:
  explicit ObstacleMemory(Options options = {});
  void update(const Point &sensor, const Points &returns, double stamp, const Point &body);
  void clear();
  Points points() const;
  size_t size() const { return cells_.size(); }
  double stamp() const { return stamp_; }
private:
  using Key = std::array<int,3>;
  Key key(const Point &p) const;
  Options options_;
  std::map<Key,Point> cells_;
  double stamp_ = 0;
};

bool validPose(const Pose &p);
}  // namespace dog_navigation
