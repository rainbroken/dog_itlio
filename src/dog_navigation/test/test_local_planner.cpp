#include "dog_navigation/local_planner.hpp"
#include "dog_navigation/localization_gate.hpp"
#include "dog_terrain/terrain_graph.hpp"
#include "../src/collision_overlay.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <pcl/io/pcd_io.h>
using namespace dog_navigation;
namespace {
std::shared_ptr<dog_terrain::TerrainGraph> floorGraph(bool upper=false,bool room=false) {
  auto cloud=std::make_shared<dog_terrain::Cloud>();
  for(double x=-2;x<2;x+=.035)for(double y=-1.3;y<1.3;y+=.035) {
    cloud->push_back(pcl::PointXYZ(x,y,0));
    if(upper)cloud->push_back(pcl::PointXYZ(x,y,2.4));
  }
  if(room) {
    for(double z=0;z<2.5;z+=.035) {
      for(double y=-1.3;y<1.3;y+=.035)cloud->push_back(pcl::PointXYZ(-2,y,z));
      for(double x=-2;x<2;x+=.035) {
        cloud->push_back(pcl::PointXYZ(x,-1.3,z));cloud->push_back(pcl::PointXYZ(x,1.3,z));
      }
    }
  }
  return std::make_shared<dog_terrain::TerrainGraph>(cloud);
}
Points obstacle() {Points out;for(double z=.2;z<.9;z+=.1)out.emplace_back(0,0,z);return out;}
}
TEST(LocalPlanner, LiveOverlayRejectsGrazingObstacleBetweenRiskSamples) {
  Options options;options.voxel=.06;options.padding=0;
  const double radius=.22+options.voxel*.866025404;
  CollisionOverlay overlay({Point(.015,radius-.00005,.4)},options,.22,.85);
  for(int i=0;i<=5;++i)EXPECT_GE(overlay.risk(Point(i*.03,0,0)),0);
  EXPECT_LT(overlay.edgeRisk(Point(0,0,0),Point(.15,0,0)),0);
  EXPECT_LT(overlay.edgeRisk(Point(.15,0,0),Point(0,0,0)),0);
}
TEST(LocalPlanner, ReplansAroundNewObstacleAndKeepsHeights) {
  auto graph=floorGraph(true);LocalPlanner planner(graph);
  const Point start(-1.5,0,0),goal(1.5,0,0);
  auto baseline=planner.plan(start,goal,{},Pose::Identity());ASSERT_TRUE(baseline.valid)<<baseline.reason;
  auto route=planner.plan(start,goal,obstacle(),Pose::Identity());ASSERT_TRUE(route.valid)<<route.reason;
  EXPECT_GT(route.cost,baseline.cost);EXPECT_GT(route.obstacle_count,0u);
  for(const auto &p:route.points) {
    EXPECT_NEAR(p.z(),0,1e-5);
    EXPECT_GT(p.head<2>().norm(),.42);
  }
  for(size_t i=1;i<route.points.size();++i) {
    const Point a=route.points[i-1],b=route.points[i],delta=b-a;
    const double t=std::clamp(-a.dot(delta)/delta.squaredNorm(),0.,1.);
    EXPECT_GT((a+t*delta).head<2>().norm(),.42);
  }
  EXPECT_LT((route.local.back()-route.local.front()).norm(),3.3);
  auto upper=planner.plan({-1.5,0,2.4},{1.5,0,2.4},obstacle(),Pose::Identity());
  ASSERT_TRUE(upper.valid)<<upper.reason;
  for(const auto &p:upper.points)EXPECT_NEAR(p.z(),2.4,1e-5);
  EXPECT_FALSE(planner.plan(start,{1.5,0,2.4},{},Pose::Identity()).valid);
}
TEST(LocalPlanner, BlockageNeverReturnsOldRoute) {
  auto graph=floorGraph();LocalPlanner planner(graph);Points wall;
  for(double y=-1.4;y<1.4;y+=.08)for(double z=.2;z<.9;z+=.12)wall.emplace_back(0,y,z);
  auto route=planner.plan({-1.5,0,0},{1.5,0,0},wall,Pose::Identity());
  EXPECT_FALSE(route.valid);EXPECT_TRUE(route.points.empty());
  EXPECT_EQ(route.reason,"blocked_no_route");
  EXPECT_TRUE(planner.plan({-1.5,0,0},{1.5,0,0},{},Pose::Identity()).valid);
  Points start_obstacle{{-1.5,0,.4}};
  EXPECT_EQ(planner.plan({-1.5,0,0},{1.5,0,0},start_obstacle,Pose::Identity()).reason,"start_blocked");
}
TEST(LocalPlanner, UsesFullSE3OdomCorrectionAndIgnoresStaticStairs) {
  auto graph=std::make_shared<dog_terrain::TerrainGraph>(dog_terrain::switchbackCloud());
  LocalPlanner planner(graph);Pose map_from_odom=Pose::Identity();
  map_from_odom.translation()=Point(3,-2,1);
  map_from_odom.linear()=Eigen::AngleAxisd(.15,Point::UnitY()).toRotationMatrix();
  Points scan;for(const auto &p:graph->cloud())scan.push_back(map_from_odom.inverse()*Point(p.x,p.y,p.z));
  auto route=planner.plan({-1,0,0},{-1,0,2.4},scan,map_from_odom);
  ASSERT_TRUE(route.valid)<<route.reason;EXPECT_EQ(route.obstacle_count,0u);
  EXPECT_NEAR(route.points.back().z(),2.4,1e-5);
  EXPECT_NE(std::find(route.stairs.begin(),route.stairs.end(),true),route.stairs.end());
}
TEST(ObstacleMemory, PositiveRaysClearAndOcclusionRetains) {
  ObstacleMemory memory;const Point sensor(-2,0,.5),body(-2,0,.5);
  Points hits{{0,0,.5},{0,1,.5},{0,-1,.5}};memory.update(sensor,hits,1,body);
  ASSERT_EQ(memory.size(),3u);
  Points occluded{{-1,0,.5},{-1,1,.5},{-1,-1,.5}};memory.update(sensor,occluded,2,body);
  auto retained=memory.points();EXPECT_TRUE(std::any_of(retained.begin(),retained.end(),[](const auto &p){return p.isApprox(Point(0,0,.5));}));
  Points clear{{2,0,.5},{2,2,.5},{2,-2,.5}};memory.update(sensor,clear,3,body);
  auto cleared=memory.points();EXPECT_FALSE(std::any_of(cleared.begin(),cleared.end(),[](const auto &p){return p.isApprox(Point(0,0,.5));}));
  EXPECT_THROW(memory.update(sensor,clear,3,body),std::invalid_argument);
  EXPECT_DOUBLE_EQ(memory.stamp(),3);
  EXPECT_THROW(memory.update(sensor,{},4,body),std::invalid_argument);
  EXPECT_DOUBLE_EQ(memory.stamp(),3); // Failed scan must not partially clear memory.
  memory.clear();EXPECT_EQ(memory.size(),0u);
}

TEST(ObstacleMemory, OccupiedEndpointsWinRegardlessOfFreeRayOrder) {
  const Point sensor(0,0,0);Points first{{1,0,0},{2,0,0},{2,1,0},{2,-1,0}};
  Points reverse(first.rbegin(),first.rend());ObstacleMemory a,b;
  a.update(sensor,first,1,sensor);b.update(sensor,reverse,1,sensor);
  auto left=a.points(),right=b.points();ASSERT_EQ(left.size(),right.size());
  for(size_t i=0;i<left.size();++i)EXPECT_TRUE(left[i].isApprox(right[i]));
  EXPECT_TRUE(std::any_of(left.begin(),left.end(),[](const Point &p){return p.isApprox(Point(1,0,0));}));
}

// Test fixture export for the DDS test; validates exactly the same persistent
// artifact format (hashes/revision) that the running node will load.
TEST(LocalPlanner, ProjectFixture) {
  const char *root=std::getenv("DOG_NAV_TEST_MAP_ROOT");
  auto path=root?std::filesystem::path(root):std::filesystem::temp_directory_path()/
    ("dog-nav-artifact-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  auto graph=floorGraph(false,std::getenv("DOG_NAV_TEST_ROOM")!=nullptr);
  if(std::getenv("DOG_NAV_TEST_RECT")) {
    dog_terrain::Config c;c.body_length=.66;c.body_width=.58;c.body_top=.5;c.body_bottom=.3;
    graph=std::make_shared<dog_terrain::TerrainGraph>(graph->cloud().makeShared(),c);
  }
  dog_terrain::saveProject(path,"scene",*graph);
  EXPECT_EQ(dog_terrain::loadProject(path,"scene")->revision(),graph->revision());
  if(!root)std::filesystem::remove_all(path);
}

TEST(LocalPlanner, RecordedExecutionRouteAudit) {
  const char *root=std::getenv("DOG_EXECUTION_MAP_ROOT");if(!root)GTEST_SKIP();
  auto graph=dog_terrain::loadProject(root,std::getenv("DOG_EXECUTION_MAP_ID"));LocalPlanner planner(graph);
  const Point start(11.627,-2.626,-.809),goal(12.527,-4.440,-5.245);
  auto before=std::chrono::steady_clock::now();auto route=planner.plan(start,goal,{},Pose::Identity());
  const double cold=std::chrono::duration<double>(std::chrono::steady_clock::now()-before).count();
  // A dataset audit reports feasibility; a failed certificate must never
  // publish points. Do not turn an observed unsafe route into a passing one.
  if(!route.valid) {
    EXPECT_TRUE(route.points.empty());EXPECT_TRUE(route.ids.empty());
    if(const char *out=std::getenv("DOG_EXECUTION_PLANNED_ROUTE")) {
      std::ofstream file(out);file<<dog_terrain::Json({{"valid",false},{"reason",route.reason},
        {"cold_seconds",cold},{"revision",graph->revision()},
        {"physical_robot_verified",false}}).dump(2)<<'\n';
    }
    return;
  }
  before=std::chrono::steady_clock::now();auto cached=planner.plan(start,goal,{},Pose::Identity());
  const double warm=std::chrono::duration<double>(std::chrono::steady_clock::now()-before).count();
  ASSERT_TRUE(cached.valid);EXPECT_EQ(route.ids,cached.ids);
  dog_terrain::Json points=dog_terrain::Json::array(),sections=dog_terrain::Json::array();
  for(const auto &p:route.points)points.push_back({p.x(),p.y(),p.z()});
  for(size_t i=0;i<route.stairs.size();++i)if(route.stairs[i])sections.push_back({{"mode","stairs"},{"start_index",i},{"end_index",i+1}});
  if(const char *out=std::getenv("DOG_EXECUTION_PLANNED_ROUTE")) {
    std::ofstream file(out);file<<dog_terrain::Json({{"valid",true},{"route",{{"points",points},{"sections",sections}}},
      {"cold_seconds",cold},{"cached_seconds",warm},{"revision",graph->revision()}}).dump(2)<<'\n';
  }
}

TEST(LocalizationGate, WireBoundaryRejectsWrongMapStaleAndMalformedStatus) {
  LocalizationGate gate("/tmp/dog-nav-gate",.8);
  const auto t=LocalizationGate::Time{}+std::chrono::seconds(10);
  EXPECT_FALSE(gate.ready(t));
  EXPECT_EQ(gate.updateJson(R"({"map_path":"/tmp/dog-nav-gate/localization.pcd","valid":true,"map_ready":true})",t),"");
  EXPECT_TRUE(gate.ready(t+std::chrono::milliseconds(700)));
  EXPECT_FALSE(gate.ready(t+std::chrono::milliseconds(900)));
  EXPECT_FALSE(gate.ready(t-std::chrono::milliseconds(1)));
  EXPECT_EQ(gate.updateJson(R"({"map_path":"/tmp/other","valid":true,"map_ready":true})",t),"map_mismatch");
  EXPECT_FALSE(gate.ready(t));
  EXPECT_EQ(gate.updateJson("broken json",t),"invalid_localization_status");
  EXPECT_FALSE(gate.ready(t));
}
