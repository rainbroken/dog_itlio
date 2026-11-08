#include "cloud_tiles.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

CloudTiles buildCloudTiles(const std::string &bytes){
  if(bytes.size()%16)throw std::invalid_argument("Invalid XYZI packet");
  std::map<std::array<int64_t,3>,std::vector<size_t>> cells;
  for(size_t i=0;i<bytes.size()/16;++i){
    float p[4];std::memcpy(p,bytes.data()+i*16,16);
    if(!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2]))continue;
    // Reject impossible coordinates before converting to integer tile keys.
    if(std::abs(p[0])>1e9||std::abs(p[1])>1e9||std::abs(p[2])>1e9)continue;
    cells[{int64_t(std::floor(p[0]/2.0)),int64_t(std::floor(p[1]/2.0)),int64_t(std::floor(p[2]/2.0))}].push_back(i);
  }
  CloudTiles result;
  for(auto &[key,indices]:cells){
    std::sort(indices.begin(),indices.end(),[&](size_t a,size_t b){return std::memcmp(bytes.data()+a*16,bytes.data()+b*16,16)<0;});
    const auto prefix=std::to_string(key[0])+","+std::to_string(key[1])+","+std::to_string(key[2])+":";
    for(size_t begin=0;begin<indices.size();begin+=512){
      CloudTile tile;tile.center={(key[0]+.5)*2,(key[1]+.5)*2,(key[2]+.5)*2};
      const auto end=std::min(indices.size(),begin+512);
      tile.indices.assign(indices.begin()+begin,indices.begin()+end);
      uint64_t hash=14695981039346656037ULL;
      for(auto index:tile.indices)for(size_t j=0;j<16;++j){hash^=static_cast<unsigned char>(bytes[index*16+j]);hash*=1099511628211ULL;}
      tile.version=std::to_string(hash);result.emplace(prefix+std::to_string(begin/512),std::move(tile));
    }
  }
  return result;
}
std::string cloudTileBytes(const std::string &bytes,const CloudTile &tile,size_t offset,size_t count){
  if(offset>tile.indices.size()||count>tile.indices.size()-offset)throw std::invalid_argument("Invalid tile slice");
  std::string result;result.reserve(count*16);
  for(size_t i=offset;i<offset+count;++i)result.append(bytes,tile.indices[i]*16,16);
  return result;
}
std::string cloudTilesVersion(const CloudTiles &tiles){
  uint64_t hash=14695981039346656037ULL;
  for(const auto &[id,tile]:tiles)for(const auto &part:{id,std::string("="),tile.version,std::string(";")})
    for(unsigned char c:part){hash^=c;hash*=1099511628211ULL;}
  return std::to_string(hash);
}
