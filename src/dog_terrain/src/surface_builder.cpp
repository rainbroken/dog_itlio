#include "dog_terrain/terrain_graph.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <Eigen/Eigenvalues>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>

namespace dog_terrain {
namespace {
constexpr double pi = 3.14159265358979323846;
Eigen::Vector3d xyz(const pcl::PointXYZ &p) { return {p.x, p.y, p.z}; }
}
void TerrainGraph::build() {
  const auto &c = config_;
  indexRawPoints();
  // Extract local plane seeds. A vertical riser otherwise joins
  // adjacent treads into one single-link cluster and erases their surfaces.
  // Keep raw_cells_ unchanged: walls/risers still participate in body collision.
  auto normal_cloud = std::make_shared<Cloud>();
  const double normal_voxel = std::min(c.resolution * .15, c.min_step * .25);
  {
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> filter(normal_voxel);
    filter.setInputCloud(cloud_);
    filter.addPointsFromInputCloud();
    filter.getVoxelCentroids(normal_cloud->points);
  }
  normal_cloud->width = normal_cloud->size(); normal_cloud->height = 1;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(normal_cloud);
  const double normal_radius = c.normal_radius;
  const double normal_cosine = std::cos(c.max_slope_deg * pi / 180);
  std::vector<int> neighbors;
  std::vector<float> distances;
  std::vector<Eigen::Vector3d> normals(normal_cloud->size(), Eigen::Vector3d::Zero());
  std::vector<Eigen::Vector3d> centers(normal_cloud->size(), Eigen::Vector3d::Zero());
  for (size_t id = 0; id < normal_cloud->size(); ++id) {
    // Keep successful small-scale estimates at tread edges. A larger bounded
    // neighborhood is only a retry for sparse/noisy patches; it is never used
    // to replace a valid fine-scale plane or fill unobserved map geometry.
    for (double radius : {normal_radius, c.normal_fallback_radius}) {
      if(radius<=0||normals[id].squaredNorm()>.5)break;
      if (tree.radiusSearch((*normal_cloud)[id], radius, neighbors, distances, 256) < std::max(6, c.min_points)) continue;
      Eigen::Vector3d center = Eigen::Vector3d::Zero();
      for (int neighbor : neighbors) center += xyz((*normal_cloud)[neighbor]);
      center /= static_cast<double>(neighbors.size());
      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (int neighbor : neighbors) {
        const Eigen::Vector3d delta = xyz((*normal_cloud)[neighbor]) - center;
        covariance.noalias() += delta * delta.transpose();
      }
      covariance /= static_cast<double>(neighbors.size());
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
      if (solver.info() == Eigen::Success &&
          solver.eigenvalues()[1] > std::pow(radius * .05, 2) &&
          std::abs(solver.eigenvectors()(2, 0)) >= normal_cosine &&
          solver.eigenvalues()[0] <= std::pow(std::min(c.normal_thickness, c.max_roughness), 2)) {
        normals[id] = solver.eigenvectors().col(0);
        centers[id] = center;
      }
    }
  }
  std::vector<Eigen::Vector3d> node_normals;
  for (const auto &[key, raw_ids] : raw_cells_) {
    std::vector<int> seeds;
    for (int id : raw_ids) {
      // Grow a measured plane into its edge points. Their own neighborhoods
      // mix tread and riser, so rejecting by their normals alone erases the
      // tread precisely where the planner must cross to the next step.
      tree.radiusSearch((*cloud_)[id], normal_radius * 2, neighbors, distances, 256);
      for (int neighbor : neighbors) {
        if (normals[neighbor].squaredNorm() < .5) continue;
        if (std::abs(normals[neighbor].dot(xyz((*cloud_)[id]) - centers[neighbor])) <= std::min(c.plane_distance, c.max_roughness)) {
          seeds.push_back(neighbor);
          // Plane evidence comes from the larger, validated neighborhood. A
          // cell with only one or two measured floor points may not define its
          // own node, but those observations still support nearby footprints.
          // Do not erase them because of an unrelated per-cell node threshold.
          support_cells_[key].push_back(id);
          break;
        }
      }
    }
    std::sort(seeds.begin(), seeds.end());
    seeds.erase(std::unique(seeds.begin(), seeds.end()), seeds.end());
    auto remaining = raw_ids;
    const size_t first_node = nodes_.size();
    // Partition by measured planes, not by a chain of Z gaps. Estimate the
    // normal on the larger seed neighborhood rather than on a thin tread edge.
    while (remaining.size() >= static_cast<size_t>(c.min_points)) {
      std::vector<int> best;
      int best_seed = -1;
      for (int seed : seeds) {
        std::vector<int> inliers;
        for (int id : remaining) {
          const Eigen::Vector3d delta = xyz((*cloud_)[id]) - centers[seed];
          if (delta.norm() <= normal_radius * 3 &&
              std::abs(normals[seed].dot(delta)) <= std::min(c.plane_distance, c.max_roughness)) inliers.push_back(id);
        }
        if (inliers.size() > best.size()) { best = std::move(inliers); best_seed = seed; }
      }
      if (best.size() < static_cast<size_t>(c.min_points)) break;
      Eigen::Vector3d center = Eigen::Vector3d::Zero();
      for (int id : best) center += xyz((*cloud_)[id]);
      center /= static_cast<double>(best.size());
      Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
      double roughness = 0;
      for (int id : best) {
        const Eigen::Vector3d delta = xyz((*cloud_)[id]) - center;
        covariance.noalias() += delta.head<2>() * delta.head<2>().transpose();
        roughness += std::pow(normals[best_seed].dot(delta), 2);
      }
      covariance /= static_cast<double>(best.size());
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
      bool duplicate = false;
      for (size_t i = first_node; i < nodes_.size(); ++i)
        duplicate |= std::abs(nodes_[i].point.z() - center.z()) < c.height_cluster;
      if (solver.info() == Eigen::Success &&
          solver.eigenvalues()[0] > std::pow(c.resolution * .02, 2)) {
        auto &support = support_cells_[key];
        support.insert(support.end(), best.begin(), best.end());
        if (!duplicate) {
          const double slope = std::acos(std::clamp(std::abs(normals[best_seed].z()), 0.0, 1.0));
          nodes_.push_back({center, slope, std::sqrt(roughness / best.size()), false, -1});
          node_normals.push_back(normals[best_seed]);
          if (nodes_.size() > static_cast<size_t>(c.max_nodes))
            throw std::invalid_argument("Too many surface nodes; crop the PCD or increase resolution");
        }
      }
      std::sort(best.begin(), best.end());
      remaining.erase(std::remove_if(remaining.begin(), remaining.end(),
          [&](int id) { return std::binary_search(best.begin(), best.end(), id); }), remaining.end());
    }
    auto support=support_cells_.find(key);
    if(support!=support_cells_.end()) {
      auto &ids=support->second;
      std::sort(ids.begin(),ids.end());ids.erase(std::unique(ids.begin(),ids.end()),ids.end());
    }
  }
  if (nodes_.empty()) throw std::invalid_argument("No surface found; check density, units and Z-up alignment");
  indexNodes();
  auto evaluate=[&](Node &node) {
    if(c.body_length==0) {
      node.support_valid=supported(node.point);node.body_clear=bodyClear(node.point);
      node.valid=node.support_valid&&node.body_clear;return;
    }
    node.support_valid=node.body_clear=node.valid=false;
    for(int heading=0;heading<16&&!node.valid;++heading) {
      const double yaw=heading*pi/16;
      const bool support=supportedAtYaw(node.point,yaw);
      const bool clear=(support||!node.body_clear) ? rectangleClear(node.point,node.point,yaw) : false;
      node.support_valid|=support;node.body_clear|=clear;node.valid=support&&clear;
    }
  };
  for (size_t id=0;id<nodes_.size();++id) {
    auto &node=nodes_[id];evaluate(node);
    if(node.valid||c.body_length==0)continue;
    // A density-weighted centroid can sit against a wall or an observed-data
    // boundary while the middle of the same cell is feasible. Try that pose on
    // the measured plane, without adding points or accepting an unsupported pose.
    const Cell key=cell(node.point);Node alternative=node;
    alternative.point.x()=(key.first+.5)*c.resolution;
    alternative.point.y()=(key.second+.5)*c.resolution;
    const auto &normal=node_normals[id];
    alternative.point.z()=node.point.z()-(normal.head<2>().dot(
        alternative.point.head<2>()-node.point.head<2>()))/normal.z();
    bool observed=false;
    auto support=support_cells_.find(key);
    if(support!=support_cells_.end())for(int point_id:support->second) {
      const auto point=xyz((*cloud_)[point_id]);
      if((point-alternative.point).head<2>().norm()<=c.resolution*.35&&
          std::abs(point.z()-alternative.point.z())<=c.plane_distance){observed=true;break;}
    }
    if(observed) {
      evaluate(alternative);
      if(alternative.valid)node=alternative;
    }
  }
  edges_.resize(nodes_.size());
  std::vector<Cell> offsets{{1,0},{0,1},{1,1},{1,-1}};
  if(c.connection_distance>0) {
    offsets.clear();
    const int reach=int(std::ceil(c.connection_distance/c.resolution));
    for(int dx=0;dx<=reach;++dx)for(int dy=-reach;dy<=reach;++dy)
      if(dx>0||dy>0)offsets.emplace_back(dx,dy);
    // Multiple measured surfaces in a single cell may also be connected.
    offsets.emplace_back(0,0);
  }
  for (const auto &[key, ids] : cells_) {
    for (const auto &[dx, dy] : offsets) {
      auto neighbor = cells_.find({key.first + dx, key.second + dy});
      if (neighbor == cells_.end()) continue;
      for (int i : ids) for (int j : neighbor->second) {
        if(dx==0&&dy==0&&j<=i)continue;
        const auto &a = nodes_[i]; const auto &b = nodes_[j];
        if (!a.valid || !b.valid) continue;
        const double dz = std::abs(b.point.z() - a.point.z());
        const double horizontal = (b.point - a.point).head<2>().norm();
        if(c.connection_distance>0&&horizontal>c.connection_distance)continue;
        const bool step = dz >= c.min_step;
        if (horizontal < 1e-6 || dz > c.max_step) continue;
        // Both surfaces already passed the configured slope limit. Do not add
        // an unrelated hard-coded 12 degree limit only at a height transition.
        if (!step && dz > horizontal * std::tan(c.max_slope_deg * pi / 180) + .015) continue;
        // Rectangular edges check the footprint along the entire segment;
        // requiring extra candidate centroids in both corner cells is redundant
        // and makes graph connectivity depend on incidental grid alignment.
        if (c.body_length==0 && dx && dy && (!cornerSupported({key.first + dx, key.second}, a.point, b.point) ||
                         !cornerSupported({key.first, key.second + dy}, a.point, b.point))) continue;
        if (!edgeClear(a.point, b.point)) continue;
        const double cost = (b.point - a.point).norm() * (1 + (step ? c.step_penalty : 0));
        edges_[i].push_back({j, cost, step}); edges_[j].push_back({i, cost, step});
      }
    }
  }
  applyClearanceCosts();
  int component = 0;
  for (size_t i = 0; i < nodes_.size(); ++i) {
    if (!nodes_[i].valid || nodes_[i].component >= 0) continue;
    std::vector<int> pending{static_cast<int>(i)};
    nodes_[i].component = component;
    while (!pending.empty()) {
      int current = pending.back(); pending.pop_back();
      for (const auto &edge : edges_[current]) if (nodes_[edge.target].component < 0) {
        nodes_[edge.target].component = component; pending.push_back(edge.target);
      }
    }
    ++component;
  }
}

}  // namespace dog_terrain
