#pragma once

#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <array>

namespace dog_terrain {
using Json = nlohmann::json;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Cell = std::pair<int, int>;

struct Config {
  double resolution = 0.15;
  double height_cluster = 0.055;
  int min_points = 3;
  double max_roughness = 0.035;
  double max_slope_deg = 25.0;
  double max_step = 0.22;
  double min_step = 0.075;
  double body_radius = 0.22;
  // Zero dimensions retain legacy cylinder projects. Robot projects explicitly
  // configure the rectangular footprint; body_radius then has no geometric role.
  double body_length = 0;
  double body_width = 0;
  double normal_radius = 0.0825;
  double normal_fallback_radius = 0;  // only retry failed local plane estimates
  double normal_thickness = 0.01;
  double plane_distance = 0.008;
  double support_distance = 0;  // zero: legacy resolution / 2
  double connection_distance = 0;  // zero: legacy adjacent cells only
  double terrain_tolerance = 0;  // zero: fixed vertical body; otherwise follow observed connected treads
  double footprint_padding = 0.0;
  double body_bottom = 0.30;
  double body_top = 0.85;
  double snap_xy = 0.30;
  double snap_z = 0.25;
  double step_penalty = 2.0;
  double max_tread_gap = 0.65;
  double clearance_weight = 6.0;
  double clearance_distance = 0.60;
  int max_nodes = 100000;
  void validate() const;
  Json json() const;
  static Config parse(const Json &value);
};

struct Node {
  Eigen::Vector3d point;
  double slope = 0;
  double roughness = 0;
  bool valid = false;
  int component = -1;
  double boundary_distance = 0;
  bool support_valid = false;
  bool body_clear = false;
};
struct Edge { int target; double cost; bool stairs; };

// Immutable after build/load. This graph produces previews, never robot commands.
class TerrainGraph {
public:
  explicit TerrainGraph(Cloud::Ptr cloud, Config config = {});
  const Cloud &cloud() const { return *cloud_; }
  const std::vector<Node> &nodes() const { return nodes_; }
  const std::vector<std::vector<Edge>> &edges() const { return edges_; }
  const Config &config() const { return config_; }
  const std::string &revision() const { return revision_; }
  int snap(const Eigen::Vector3d &point) const;
  Json plan(const Eigen::Vector3d &start, const Eigen::Vector3d &goal) const;
  Json inspectSurface(const Eigen::Vector3d &point) const;
  bool turnClear(int previous, int current, int next) const;
  bool motionClear(const Eigen::Vector3d &a,const Eigen::Vector3d &b,double first_yaw,double last_yaw) const;
  Json inspectConnection(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const;
  Json summary() const;
  Json cache() const;
  static std::shared_ptr<TerrainGraph> fromCache(Cloud::Ptr cloud, const Json &cache);

private:
  TerrainGraph() = default;
  Cell cell(const Eigen::Vector3d &p) const;
  bool supported(const Eigen::Vector3d &p) const;
  // Reusable only for the exact same support centre. The terrain neighbourhood
  // is independent of yaw; sharing it does not change footprint samples.
  struct SupportContext {std::vector<int> terrain;bool built=false;};
  // A fixed-centre rotation may reuse localTerrain only when its exact cell
  // bounds match. Collision poses and raw-point contact tests remain distinct.
  struct RotationContext {std::map<std::array<int,4>,std::vector<int>> terrain;};
  bool supportedAtYaw(const Eigen::Vector3d &p, double yaw,SupportContext *context=nullptr) const;
  std::vector<int> localTerrain(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                                const Eigen::Vector3d &padding) const;
  bool rectangleClear(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                      double yaw, double padding = 0, Eigen::Vector3d *obstacle = nullptr,
                      RotationContext *context=nullptr) const;
  bool bodyClear(const Eigen::Vector3d &p) const;
  bool bodySweepClear(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                      Eigen::Vector3d *obstacle = nullptr, double *fraction = nullptr) const;
  double obstacleClearance(const Eigen::Vector3d &p) const;
  bool cornerSupported(const Cell &key, const Eigen::Vector3d &a, const Eigen::Vector3d &b) const;
  bool edgeClear(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const;
  void build();
  void indexNodes();
  void indexRawPoints();
  void applyClearanceCosts();
  Config config_;
  Cloud::Ptr cloud_;
  std::vector<Node> nodes_;
  std::vector<std::vector<Edge>> edges_;
  std::map<Cell, std::vector<int>> cells_;
  std::map<Cell, std::vector<int>> raw_cells_;
  std::map<Cell, std::vector<int>> support_cells_;
  std::string revision_;
};

Cloud::Ptr readPcd(const std::filesystem::path &path);
Cloud::Ptr switchbackCloud();
std::string sha256(const std::string &data);
std::string fileSha256(const std::filesystem::path &path);
std::filesystem::path mapPath(const std::filesystem::path &root, const std::string &name);
void saveProject(const std::filesystem::path &root, const std::string &name, const TerrainGraph &graph,
                 const Json &source = Json::object());
void saveProject(const std::filesystem::path &root, const std::string &name, const TerrainGraph &graph,
                 const Json &source, const std::filesystem::path &keyframes);
void saveProject(const std::filesystem::path &root, const std::string &name, const TerrainGraph &graph,
                 const Json &source, const std::filesystem::path &keyframes,
                 const std::filesystem::path &localization);
std::shared_ptr<TerrainGraph> loadProject(const std::filesystem::path &root, const std::string &name);
}  // namespace dog_terrain
