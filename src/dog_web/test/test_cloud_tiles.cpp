#include "cloud_tiles.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>

namespace {
std::string points(size_t n,float shift=0){
  std::string bytes;for(size_t i=0;i<n;++i){const float p[]={shift+float(i%100)*.01F,float(i/100)*.01F,0,float(i%256)};bytes.append(reinterpret_cast<const char *>(p),16);}return bytes;
}
}
TEST(CloudTiles, ReorderingDoesNotInvalidateTilesAndAllPointsArePreserved){
  auto bytes=points(70000);const auto tiles=buildCloudTiles(bytes);
  size_t count=0;std::vector<std::string> reconstructed,original;
  for(const auto &[id,tile]:tiles){EXPECT_LE(tile.indices.size(),512U);count+=tile.indices.size();
    for(size_t offset=0;offset<tile.indices.size();offset+=32){const auto part=cloudTileBytes(bytes,tile,offset,std::min(size_t(32),tile.indices.size()-offset));for(size_t i=0;i<part.size();i+=16)reconstructed.push_back(part.substr(i,16));}
  }
  EXPECT_EQ(count,70000U);
  std::string reversed;for(size_t i=bytes.size();i;i-=16){original.push_back(bytes.substr(i-16,16));reversed+=original.back();}
  std::sort(original.begin(),original.end());std::sort(reconstructed.begin(),reconstructed.end());EXPECT_EQ(original,reconstructed);
  EXPECT_EQ(cloudTilesVersion(tiles),cloudTilesVersion(buildCloudTiles(reversed)));
}
TEST(CloudTiles, ChangesAreSpatiallyIsolatedAndRemovalsChangeManifest){
  auto left=points(20),right=points(20,10);const auto before=buildCloudTiles(left+right),after=buildCloudTiles(left+points(21,10));
  for(const auto &[id,tile]:before)if(id.rfind("0,",0)==0)EXPECT_EQ(tile.version,after.at(id).version);
  EXPECT_NE(cloudTilesVersion(before),cloudTilesVersion(after));EXPECT_NE(cloudTilesVersion(before),cloudTilesVersion(buildCloudTiles(left)));
  EXPECT_THROW(cloudTileBytes(left,before.begin()->second,100,1),std::invalid_argument);
}
