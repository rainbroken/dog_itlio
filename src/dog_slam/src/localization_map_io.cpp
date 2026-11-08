#include "dog_slam/map_localization.hpp"
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <cmath>

namespace dog_slam {
std::vector<Pose3> readLocalizationKeyframes(const std::string &input) {
  namespace fs = std::filesystem;
  fs::path path(input);
  if (input.rfind("~/", 0) == 0 && std::getenv("HOME"))
    path = fs::path(std::getenv("HOME")) / input.substr(2);
  if(!fs::is_directory(path) && path.filename()!="cloud.pcd" &&
     path.filename()!="localization.pcd" && path.filename()!="cloud_static.pcd" && path.filename()!="cloud_optimized.pcd")return {};
  const auto filename=path.filename()=="cloud_optimized.pcd" ?
      "localization_optimized_keyframes.csv" : "localization_keyframes.csv";
  path = (fs::is_directory(path) ? path : path.parent_path()) / filename;
  if (!fs::exists(path)) return {};
  if (!fs::is_regular_file(path) || fs::is_symlink(path) || fs::file_size(path)>16*1024*1024)
    throw std::invalid_argument("Invalid localization keyframe artifact");
  std::ifstream stream(path); std::string line;
  std::getline(stream,line);
  if(line!="id,timestamp,px,py,pz,qx,qy,qz,qw")
    throw std::invalid_argument("Invalid localization keyframe header");
  std::vector<Pose3> poses;
  while(std::getline(stream,line)) {
    if(poses.size()>=100000) throw std::invalid_argument("Too many localization keyframes");
    std::replace(line.begin(),line.end(),',',' '); std::istringstream row(line);
    double id,t,x,y,z,qx,qy,qz,qw; std::string extra;
    if(!(row>>id>>t>>x>>y>>z>>qx>>qy>>qz>>qw) || row>>extra ||
       !std::isfinite(id)||id<0||std::floor(id)!=id||!std::isfinite(t))
      throw std::invalid_argument("Malformed localization keyframe row");
    Eigen::Quaterniond q(qw,qx,qy,qz);
    if(!q.coeffs().allFinite() || std::abs(q.norm()-1)>.001)
      throw std::invalid_argument("Invalid keyframe quaternion");
    Pose3 p=Pose3::Identity();p.linear()=q.normalized().toRotationMatrix();p.translation()=Eigen::Vector3d(x,y,z);
    if(!validPose(p))throw std::invalid_argument("Invalid keyframe pose");
    poses.push_back(p);
  }
  if(!stream.eof() || poses.empty())throw std::invalid_argument("Empty or unreadable localization keyframes");
  return poses;
}
LocalizationCloud::Ptr readLocalizationMap(const std::string &input) {
  namespace fs = std::filesystem;
  fs::path path(input);
  if (input.rfind("~/", 0) == 0) {
    const char *home = std::getenv("HOME");
    if (!home) throw std::invalid_argument("HOME unavailable; use an absolute map path");
    path = fs::path(home) / input.substr(2);
  }
  if (fs::is_directory(path)) {
    if (fs::is_regular_file(path / "localization.pcd")) path /= "localization.pcd";
    else path /= "cloud.pcd";
  }
  if (!fs::is_regular_file(path) || fs::file_size(path) > 256 * 1024 * 1024)
    throw std::invalid_argument("Map must be a PCD file (<=256 MiB) or a map project directory");
  pcl::PCDReader reader; pcl::PCLPointCloud2 header; Eigen::Vector4f origin;
  Eigen::Quaternionf orientation; int version = 0, data_type = 0; unsigned int offset = 0;
  if (reader.readHeader(path.string(), header, origin, orientation, version, data_type, offset) < 0 ||
      static_cast<uint64_t>(header.width) * header.height > 8000000 || header.data.size() > 512 * 1024 * 1024)
    throw std::invalid_argument("Invalid or oversized PCD header");
  for (const char *name : {"x", "y", "z"}) {
    auto field = std::find_if(header.fields.begin(), header.fields.end(),
        [&](const auto &f) { return f.name == name; });
    if (field == header.fields.end() || field->count != 1 || field->datatype != pcl::PCLPointField::FLOAT32)
      throw std::invalid_argument("Map requires scalar FLOAT32 XYZ fields");
  }
  auto map = std::make_shared<LocalizationCloud>();
  if (pcl::io::loadPCDFile(path.string(), *map) < 0) throw std::invalid_argument("Cannot read localization PCD");
  return map;
}

}  // namespace dog_slam
