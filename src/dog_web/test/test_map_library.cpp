#include "map_library.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <chrono>

namespace fs=std::filesystem;
class MapLibraryTest:public ::testing::Test{
protected:
 void SetUp() override{root=fs::temp_directory_path()/("map-library-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));fs::create_directories(root/"maps/a");std::ofstream(root/"maps/a/map.json")<<"{}";fs::create_directories(root/"slam/session/time");std::ofstream(root/"slam/session/time/cloud.pcd")<<"original PCD bytes";}
 void TearDown() override{fs::remove_all(root);}
 fs::path root;
};
TEST_F(MapLibraryTest, NamesPersistWithoutChangingArtifactIdentity){
 PcdCatalog catalog(root/"slam");
 {MapLibrary library(root/"maps",catalog);library.rename("map","a"," 一楼大厅 ");library.rename("pcd","session/time/cloud.pcd","大厅扫描");}
 MapLibrary reopened(root/"maps",catalog);EXPECT_EQ(reopened.projects("a")["projects"][0]["name"],"一楼大厅");
 EXPECT_EQ(reopened.savedPcds()["files"][0]["name"],"大厅扫描");EXPECT_TRUE(fs::exists(root/"maps/a/map.json"));EXPECT_TRUE(fs::exists(root/"slam/session/time/cloud.pcd"));
 EXPECT_THROW(reopened.rename("map","a","../escape"),std::invalid_argument);
 EXPECT_THROW(reopened.remove("map","../escape"),std::invalid_argument);
}
TEST_F(MapLibraryTest, RemovalMovesOnlySelectedItemAndKeepsRecoverableBytes){
 PcdCatalog catalog(root/"slam");MapLibrary library(root/"maps",catalog);
 auto removed=library.remove("pcd","session/time/cloud.pcd");const fs::path recovery=removed["recovery_path"].get<std::string>();
 EXPECT_TRUE(library.savedPcds()["files"].empty());EXPECT_TRUE(fs::exists(recovery/"record.json"));
 std::ifstream input(recovery/"payload");std::string content;std::getline(input,content);EXPECT_EQ(content,"original PCD bytes");EXPECT_TRUE(fs::exists(root/"maps/a/map.json"));
 removed=library.remove("map","a");EXPECT_TRUE(library.projects("")["maps"].empty());EXPECT_TRUE(fs::exists(fs::path(removed["recovery_path"].get<std::string>())/"payload/map.json"));
}
TEST_F(MapLibraryTest, MetadataWriteFailureRollsBackRemovalAndRejectsLinkedFiles){
 PcdCatalog catalog(root/"slam");MapLibrary library(root/"maps",catalog);
 fs::create_directory(root/"maps/.library-names.json");
 EXPECT_THROW(library.remove("pcd","session/time/cloud.pcd"),fs::filesystem_error);EXPECT_TRUE(fs::exists(root/"slam/session/time/cloud.pcd"));
 fs::create_symlink(root/"slam/session/time/cloud.pcd",root/"slam/linked.pcd");EXPECT_THROW(library.remove("pcd","linked.pcd"),std::invalid_argument);
}
