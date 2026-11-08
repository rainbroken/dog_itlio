#include "dog_terrain/terrain_graph.hpp"
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace dog_terrain {
namespace fs=std::filesystem;
Cloud::Ptr readPcd(const fs::path &path) {
  // Bound the declared decoded size before PCL allocates/decompresses the file.
  if (fs::file_size(path) > 128 * 1024 * 1024) throw std::invalid_argument("PCD exceeds 128 MiB");
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::invalid_argument("Cannot open PCD");
  size_t declared = 0, width = 0, height = 1, bytes_per_point = 0;
  std::vector<size_t> sizes, counts;
  bool found_data = false;
  for (int line_number = 0; line_number < 100; ++line_number) {
    std::string line;
    if (!std::getline(input, line) || line.size() > 8192) break;
    std::istringstream row(line); std::string key; row >> key;
    size_t value;
    if (key == "POINTS") row >> declared;
    else if (key == "WIDTH") row >> width;
    else if (key == "HEIGHT") row >> height;
    else if (key == "SIZE") { while (row >> value) sizes.push_back(value); }
    else if (key == "COUNT") { while (row >> value) counts.push_back(value); }
    else if (key == "DATA") { found_data = true; break; }
  }
  if (!declared && width <= 8000000 && height <= 8000000) declared = width * height;
  if (counts.empty()) counts.assign(sizes.size(), 1);
  if (!found_data || !declared || declared > 8000000 || sizes.empty() || sizes.size() != counts.size())
    throw std::invalid_argument("Invalid PCD header or point count (limit 8000000)");
  for (size_t i = 0; i < sizes.size(); ++i) {
    if (!sizes[i] || sizes[i] > 8 || !counts[i] || counts[i] > 4096)
      throw std::invalid_argument("Invalid PCD field size/count");
    bytes_per_point += sizes[i] * counts[i];
  }
  if (bytes_per_point > 512 * 1024 * 1024 / declared)
    throw std::invalid_argument("Decoded PCD exceeds 512 MiB");
  pcl::PCLPointCloud2 blob;
  if (pcl::io::loadPCDFile(path.string(), blob) < 0) throw std::invalid_argument("PCL failed to read PCD");
  for (const auto *name : {"x", "y", "z"}) {
    auto it = std::find_if(blob.fields.begin(), blob.fields.end(), [&](const auto &f) { return f.name == name; });
    if (it == blob.fields.end() || it->count != 1 || it->datatype != pcl::PCLPointField::FLOAT32)
      throw std::invalid_argument("PCD requires scalar FLOAT32 x/y/z fields");
  }
  auto cloud = std::make_shared<Cloud>();
  pcl::fromPCLPointCloud2(blob, *cloud);
  Cloud clean;
  clean.reserve(cloud->size());
  for (const auto &p : *cloud) if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) clean.push_back(p);
  if (clean.size() < 3 || clean.size() > 8000000) throw std::invalid_argument("Not enough finite PCD points");
  *cloud = std::move(clean);
  return cloud;
}

}  // namespace dog_terrain
