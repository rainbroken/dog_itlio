#include "dog_terrain/terrain_graph.hpp"

#include <openssl/evp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace dog_terrain {
namespace fs = std::filesystem;
namespace {
class Digest {
public:
  Digest() : context_(EVP_MD_CTX_new(), EVP_MD_CTX_free) {
    if (!context_ || EVP_DigestInit_ex(context_.get(), EVP_sha256(), nullptr) != 1)
      throw std::runtime_error("Cannot initialize SHA256");
  }
  void update(const char *data, size_t count) {
    if (EVP_DigestUpdate(context_.get(), data, count) != 1) throw std::runtime_error("SHA256 update failed");
  }
  std::string finish() {
    std::array<unsigned char, EVP_MAX_MD_SIZE> bytes{};
    unsigned int count = 0;
    if (EVP_DigestFinal_ex(context_.get(), bytes.data(), &count) != 1) throw std::runtime_error("SHA256 finalize failed");
    std::ostringstream text;
    for (unsigned int i = 0; i < count; ++i) text << std::hex << std::setw(2) << std::setfill('0') << int(bytes[i]);
    return text.str();
  }
private:
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context_;
};

void writeJson(const fs::path &path, const Json &value) {
  std::ofstream stream(path);
  stream << value.dump(2);
  stream.close();
  if (!stream) throw std::runtime_error("Failed writing " + path.string());
}

Json readJson(const fs::path &path) {
  if (fs::is_symlink(path) || fs::file_size(path) > 512 * 1024 * 1024)
    throw std::invalid_argument("Invalid map artifact: " + path.string());
  std::ifstream stream(path);
  if (!stream) throw std::invalid_argument("Cannot read " + path.string());
  return Json::parse(stream);
}
}  // namespace

std::string sha256(const std::string &data) {
  Digest digest; digest.update(data.data(), data.size()); return digest.finish();
}

std::string fileSha256(const fs::path &path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::invalid_argument("Cannot read map artifact: " + path.string());
  Digest digest;
  std::array<char, 65536> buffer{};
  while (stream.read(buffer.data(), buffer.size()) || stream.gcount())
    digest.update(buffer.data(), static_cast<size_t>(stream.gcount()));
  if (!stream.eof()) throw std::runtime_error("Failed reading " + path.string());
  return digest.finish();
}

fs::path mapPath(const fs::path &root, const std::string &name) {
  static const std::regex allowed("[A-Za-z0-9][A-Za-z0-9_-]{0,63}");
  if (!std::regex_match(name, allowed))
    throw std::invalid_argument("Map ID must be 1..64 letters, digits, underscores or hyphens");
  fs::path result = fs::absolute(root) / name;
  if (fs::is_symlink(result)) throw std::invalid_argument("Symlink map projects are not supported");
  return result;
}

void saveProject(const fs::path &root, const std::string &name, const TerrainGraph &graph, const Json &source) {
  saveProject(root,name,graph,source,{});
}

void saveProject(const fs::path &root, const std::string &name, const TerrainGraph &graph, const Json &source,
                 const fs::path &keyframes) {
  saveProject(root,name,graph,source,keyframes,{});
}

void saveProject(const fs::path &root, const std::string &name, const TerrainGraph &graph, const Json &source,
                 const fs::path &keyframes, const fs::path &localization_source) {
  fs::create_directories(root);
  const fs::path target = mapPath(root, name);
  if (fs::exists(target)) throw std::invalid_argument("Map ID already exists; use a new ID for a revision");
  const auto token = std::chrono::steady_clock::now().time_since_epoch().count();
  const fs::path staging = fs::absolute(root) / (".import-" + std::to_string(token));
  if (!fs::create_directory(staging)) throw std::runtime_error("Cannot create map staging directory");
  try {
    if (pcl::io::savePCDFileBinary((staging / "cloud.pcd").string(), graph.cloud()) < 0)
      throw std::runtime_error("Failed saving source PCD");
    if (!localization_source.empty()) {
      if (fs::is_symlink(localization_source) || !fs::is_regular_file(localization_source) ||
          readPcd(localization_source)->empty()) throw std::invalid_argument("Invalid localization PCD source");
      fs::copy_file(localization_source, staging/"localization.pcd");
    } else {
      pcl::VoxelGrid<pcl::PointXYZ> filter;
      filter.setInputCloud(graph.cloud().makeShared()); filter.setLeafSize(.20f, .20f, .20f);
      Cloud localization; filter.filter(localization);
      if (pcl::io::savePCDFileBinary((staging / "localization.pcd").string(), localization) < 0)
        throw std::runtime_error("Failed saving localization PCD");
    }
    writeJson(staging / "navigation.json", graph.cache());
    Json hashes = Json::object();
    for (const char *file : {"cloud.pcd", "localization.pcd", "navigation.json"}) hashes[file] = fileSha256(staging / file);
    if(!keyframes.empty()) {
      if(fs::is_symlink(keyframes)||!fs::is_regular_file(keyframes)||fs::file_size(keyframes)>16*1024*1024)
        throw std::invalid_argument("Invalid localization keyframe source");
      fs::copy_file(keyframes,staging/"localization_keyframes.csv");
      hashes["localization_keyframes.csv"]=fileSha256(staging/"localization_keyframes.csv");
    }
    writeJson(staging / "map.json", {{"format_version", 1}, {"map_id", name}, {"revision", graph.revision()},
              {"frame_id", "map"}, {"units", "meters"}, {"up_axis", "z"},
              {"representation", "multi_surface_graph"}, {"preview_only", true},
              {"config", graph.config().json()}, {"files", hashes}, {"summary", graph.summary()},
              {"source", source}});
    fs::rename(staging, target);
  } catch (...) { fs::remove_all(staging); throw; }
}

std::shared_ptr<TerrainGraph> loadProject(const fs::path &root, const std::string &name) {
  const fs::path directory = mapPath(root, name);
  const auto metadata = readJson(directory / "map.json");
  if (metadata.at("format_version") != 1 || metadata.at("representation") != "multi_surface_graph" ||
      metadata.at("map_id") != name || metadata.at("frame_id") != "map" || metadata.at("up_axis") != "z")
    throw std::invalid_argument("Unsupported map project format");
  for (const char *file : {"cloud.pcd", "localization.pcd", "navigation.json"})
    if (fs::is_symlink(directory / file) || fileSha256(directory / file) != metadata.at("files").at(file))
      throw std::invalid_argument(std::string("Map artifact changed: ") + file + "; re-import source map");
  const auto cache = readJson(directory / "navigation.json");
  if(metadata.at("files").contains("localization_keyframes.csv")) {
    const auto file=directory/"localization_keyframes.csv";
    if(fs::is_symlink(file)||!fs::is_regular_file(file)||fs::file_size(file)>16*1024*1024||
       fileSha256(file)!=metadata.at("files").at("localization_keyframes.csv"))
      throw std::invalid_argument("Map keyframes changed; re-import source map");
  } else if(fs::exists(directory/"localization_keyframes.csv")) {
    throw std::invalid_argument("Untracked localization keyframes; re-import source map");
  }
  if (cache.at("revision") != metadata.at("revision") || cache.at("config") != metadata.at("config"))
    throw std::invalid_argument("Map metadata does not match navigation cache");
  return TerrainGraph::fromCache(readPcd(directory / "cloud.pcd"), cache);
}

}  // namespace dog_terrain
