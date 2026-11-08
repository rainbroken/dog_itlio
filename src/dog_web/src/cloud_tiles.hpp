#pragma once
#include <array>
#include <map>
#include <string>
#include <vector>

// Indices reference the immutable CloudPacket bytes, rather than retaining a
// second point cloud. Reordered source snapshots have identical tile versions.
struct CloudTile {
  std::string version;
  std::array<double,3> center{};
  std::vector<size_t> indices;
};
using CloudTiles=std::map<std::string,CloudTile>;
CloudTiles buildCloudTiles(const std::string &xyzi);
std::string cloudTilesVersion(const CloudTiles &);
std::string cloudTileBytes(const std::string &xyzi,const CloudTile &,size_t offset,size_t count);
