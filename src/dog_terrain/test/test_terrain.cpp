#include "dog_terrain/terrain_graph.hpp"
#include "dog_terrain/graph_search.hpp"
#include "dog_terrain/collision_geometry.hpp"
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <fstream>
#include <limits>

using namespace dog_terrain;
namespace fs = std::filesystem;

TEST(Collision, OrientedBoxUsesFullCornersAndContinuousTranslation) {
  const Eigen::Vector3d zero=Eigen::Vector3d::Zero();
  EXPECT_TRUE(sweptBoxContact(zero,zero,{.32,.28,.4},0,.33,.29,.3,.5));
  EXPECT_FALSE(sweptCylinderContact(zero,zero,{.32,.28,.4},.29,.3,.5));
  EXPECT_FALSE(sweptBoxContact(zero,zero,{.34,0,.4},0,.33,.29,.3,.5));
  EXPECT_FALSE(sweptBoxContact(zero,zero,{.32,0,2.4},0,.33,.29,.3,.5));
  EXPECT_TRUE(sweptBoxContact({-1,0,0},{1,0,0},{.031,.28,.4},0,.33,.29,.3,.5));
  constexpr double pi=3.14159265358979323846;
  EXPECT_FALSE(sweptBoxContact(zero,zero,{.4,0,.4},0,.33,.29,.3,.5));
  EXPECT_FALSE(sweptBoxContact(zero,zero,{.4,0,.4},pi/2,.33,.29,.3,.5));
  EXPECT_TRUE(sweptBoxContact(zero,zero,{.4,0,.4},pi/4,.33,.29,.3,.5));
}

namespace {
void plane(Cloud &cloud, double x0, double x1, double z = 0, double y0 = -.6, double y1 = .6) {
  for (double x = x0 + .015; x < x1; x += .035)
    for (double y = y0 + .015; y < y1; y += .035)
      cloud.push_back(pcl::PointXYZ(x, y, z));
}

class Switchback : public ::testing::Test {
protected:
  static void SetUpTestSuite() { graph = std::make_shared<TerrainGraph>(switchbackCloud()); }
  static std::shared_ptr<TerrainGraph> graph;
};
std::shared_ptr<TerrainGraph> Switchback::graph;

TEST(RectangularTerrain, SwitchbackRouteAndSupportSurviveCacheReload) {
  Config c;c.body_length=.66;c.body_width=.58;c.connection_distance=.25;
  TerrainGraph graph(switchbackCloud(),c);
  const auto route=graph.plan({-1,0,0},{-1,0,2.4});
  EXPECT_TRUE(route["collision_check"]["turns_checked"].get<bool>());
  auto reloaded=TerrainGraph::fromCache(graph.cloud().makeShared(),graph.cache());
  EXPECT_EQ(route,reloaded->plan({-1,0,0},{-1,0,2.4}));
  EXPECT_EQ(route["collision_check"]["body_width"],.58);
}

TEST(RectangularTerrain, ExecutionChecksFullForwardRotationAndSweptTranslation) {
  auto cloud=std::make_shared<Cloud>();plane(*cloud,-2,2,0,-1,1);
  cloud->push_back(pcl::PointXYZ(.4,0,.4));
  Config c;c.body_length=.66;c.body_width=.58;c.body_top=.5;c.body_bottom=.3;
  TerrainGraph graph(cloud,c);const Eigen::Vector3d origin=Eigen::Vector3d::Zero();
  EXPECT_TRUE(graph.motionClear(origin,origin,0,0));
  // Identical end footprints do not imply that a forward half-turn is safe.
  EXPECT_FALSE(graph.motionClear(origin,origin,0,3.14159265358979323846));
  EXPECT_FALSE(graph.motionClear({-1,0,0},{1,0,0},0,0));
  EXPECT_FALSE(graph.motionClear(origin,origin,std::numeric_limits<double>::quiet_NaN(),0));
}

TEST(RectangularTerrain, SearchWorkspacePreservesRoutesAndReevaluatesOverlayWeights) {
  Config c;c.body_length=.66;c.body_width=.58;c.connection_distance=.25;
  TerrainGraph graph(switchbackCloud(),c);GraphSearchWorkspace workspace(graph);
  const int source=graph.snap({-1,0,0}),target=graph.snap({-1,0,2.4});
  const auto expected=searchGraph(graph,source,target);
  const auto first=searchGraph(graph,source,target,{},&workspace);
  EXPECT_EQ(first.ids,expected.ids);EXPECT_EQ(first.stairs,expected.stairs);EXPECT_DOUBLE_EQ(first.cost,expected.cost);
  const auto before=workspace.counters();
  const auto again=searchGraph(graph,source,target,{},&workspace);const auto after=workspace.counters();
  EXPECT_EQ(again.ids,first.ids);EXPECT_EQ(after.turn_evaluations,before.turn_evaluations);
  EXPECT_GT(after.turn_cache_hits,before.turn_cache_hits);EXPECT_EQ(after.lower_bound_builds,1u);
  EdgeCost blocked=[&](int current,const Edge &e){return current==source?std::numeric_limits<double>::infinity():e.cost;};
  EXPECT_THROW(searchGraph(graph,source,target,blocked,&workspace),std::invalid_argument);
  EXPECT_EQ(searchGraph(graph,source,target,{},&workspace).ids,expected.ids);
  const int platform=graph.snap({2.8,1.2,1.2});
  EXPECT_EQ(searchGraph(graph,source,platform,{},&workspace).ids,searchGraph(graph,source,platform).ids);
  EXPECT_EQ(workspace.counters().lower_bound_builds,2u);
  auto other=TerrainGraph::fromCache(graph.cloud().makeShared(),graph.cache());
  EXPECT_THROW(searchGraph(*other,source,target,{},&workspace),std::invalid_argument);
}

TEST(RectangularTerrain, RejectedLazyTurnDoesNotHideAnotherArrivalAtSameEdge) {
  Config c;c.body_length=.66;c.body_width=.58;c.body_top=.5;
  auto cloud=std::make_shared<Cloud>();plane(*cloud,-2,2,0,-2,2);
  cloud->push_back(pcl::PointXYZ(.4,0,.4));
  TerrainGraph measured(cloud,c);
  auto cache=measured.cache();
  // Two arrivals at C share the final C->D state. The cheap east-to-north
  // turn collides; the more costly north-to-north arrival must remain usable.
  cache["nodes"]=Json::array();
  for(const auto &p:std::vector<Eigen::Vector3d>{{-.8,-.8,0},{-.6,0,0},{0,-.6,0},{0,0,0},{0,.6,0}})
    cache["nodes"].push_back({p.x(),p.y(),p.z(),0,0,true,0,0,true,true});
  cache["edges"]=Json::array({{0,1,1.,false},{0,2,3.,false},
    {1,3,1.,false},{2,3,1.,false},{3,4,1.,false}});
  auto graph=TerrainGraph::fromCache(cloud,cache);
  EXPECT_FALSE(graph->turnClear(1,3,4));EXPECT_TRUE(graph->turnClear(2,3,4));
  const auto result=searchGraph(*graph,0,4);
  EXPECT_EQ(result.ids,(std::vector<int>{0,2,3,4}));EXPECT_DOUBLE_EQ(result.cost,5.);
  EXPECT_THROW(searchGraph(*graph,0,4,[](int from,const Edge &edge) {
    return from==0&&edge.target==2?std::numeric_limits<double>::infinity():edge.cost;
  }),std::invalid_argument);
}

class TemporaryDirectory {
public:
  TemporaryDirectory() {
    path = fs::temp_directory_path() / ("dog-terrain-test-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directory(path);
  }
  ~TemporaryDirectory() { std::error_code ec; fs::remove_all(path, ec); }
  fs::path path;
};
}

TEST_F(Switchback, SearchOverlayCanOnlyRemoveOrPenalizeExistingEdges) {
  int a=graph->snap({-1,0,0}),b=graph->snap({-1,0,2.4});
  const auto baseline=searchGraph(*graph,a,b);
  auto expensive=searchGraph(*graph,a,b,[](int,const Edge &e){return e.cost*2;});
  EXPECT_NEAR(expensive.cost,baseline.cost*2,1e-8);
  EXPECT_THROW(searchGraph(*graph,a,b,[](int,const Edge &){return std::numeric_limits<double>::infinity();}),std::invalid_argument);
  EXPECT_THROW(searchGraph(*graph,a,b,[](int,const Edge &e){return e.cost-.01;}),std::invalid_argument);
  EXPECT_THROW(searchGraph(*graph,-1,b),std::invalid_argument);
}

TEST_F(Switchback, CrossFloorRouteUsesBothFlightsAndLanding) {
  auto result = graph->plan({-1, 0, 0}, {-1, 0, 2.4});
  double max_x = -100;
  bool first_flight = false, second_flight = false;
  for (const auto &p : result["points"]) {
    double x = p[0], y = p[1], z = p[2];
    max_x = std::max(max_x, x);
    first_flight |= y < .5 && z > .3 && z < 1.1;
    second_flight |= y > 1 && z > 1.4 && z < 2.2;
  }
  EXPECT_GT(max_x, 2.4); EXPECT_TRUE(first_flight); EXPECT_TRUE(second_flight);
  EXPECT_NEAR(result["ascent"].get<double>(), 2.4, 1e-5);
  EXPECT_GT(result["length"].get<double>(), 8);
  int flights = 0;
  for (const auto &section : result["sections"]) if (section["mode"] == "stairs") {
    ++flights; EXPECT_EQ(section["direction"], "up");
  }
  EXPECT_EQ(flights, 2);
  auto reverse = graph->plan({-1, 0, 2.4}, {-1, 0, 0});
  EXPECT_NEAR(reverse["descent"].get<double>(), 2.4, 1e-5);
  for (const auto &section : reverse["sections"]) if (section["mode"] == "stairs") EXPECT_EQ(section["direction"], "down");
}

TEST_F(Switchback, SameXYSelectsTheRequestedFloor) {
  int low = graph->snap({-1, 0, 0}), high = graph->snap({-1, 0, 2.4});
  EXPECT_NE(low, high);
  EXPECT_NEAR(graph->nodes()[low].point.z(), 0, 1e-5);
  EXPECT_NEAR(graph->nodes()[high].point.z(), 2.4, 1e-5);
  EXPECT_THROW(graph->snap({-1, 0, 1.2}), std::invalid_argument);
  auto same = graph->plan({-1, 0, 0}, {-1, 0, 0});
  EXPECT_EQ(same["points"].size(), 1u); EXPECT_EQ(same["length"], 0);
}

TEST(Terrain, OverlapWithoutStairsRemainsDisconnected) {
  auto cloud = std::make_shared<Cloud>(); plane(*cloud, -1, 1); plane(*cloud, -1, 1, 2.4);
  TerrainGraph graph(cloud);
  EXPECT_THROW(graph.plan({0, 0, 0}, {0, 0, 2.4}), std::invalid_argument);
}

TEST(Terrain, UnknownGapCannotBeBridged) {
  auto cloud = std::make_shared<Cloud>(); plane(*cloud, -1.2, -.25); plane(*cloud, .25, 1.2);
  TerrainGraph graph(cloud);
  EXPECT_THROW(graph.plan({-.8, 0, 0}, {.8, 0, 0}), std::invalid_argument);
}

TEST(Terrain, ObservedPlaneSupportDoesNotRequireANodeInEveryCell) {
  auto dense=std::make_shared<Cloud>();plane(*dense,-1.2,1.2,0,-1.2,1.2);
  auto cloud=std::make_shared<Cloud>();int sparse_point=-1;
  for(const auto &p:*dense) {
    const bool sparse=p.x>=.15&&p.x<.30&&p.y>=.15&&p.y<.30;
    if(sparse&&sparse_point>=0)continue;
    if(sparse)sparse_point=int(cloud->size());
    cloud->push_back(p);
  }
  ASSERT_GE(sparse_point,0);
  const int isolated=int(cloud->size());cloud->push_back(pcl::PointXYZ(4,4,0));
  TerrainGraph graph(cloud);
  bool sparse_has_node=false;
  for(const auto &n:graph.nodes())
    sparse_has_node|=n.point.x()>=.15&&n.point.x()<.30&&n.point.y()>=.15&&n.point.y()<.30;
  EXPECT_FALSE(sparse_has_node);
  const auto support=graph.cache()["support_points"].get<std::vector<int>>();
  EXPECT_NE(std::find(support.begin(),support.end(),sparse_point),support.end());
  EXPECT_EQ(std::find(support.begin(),support.end(),isolated),support.end());
  EXPECT_NO_THROW(graph.plan({-.8,0,0},{.8,0,0}));
}

TEST(RectangularTerrain, SparseObservedFloorUsesBoundedNormalFallback) {
  auto cloud=std::make_shared<Cloud>();
  for(double x=-1.5+.025;x<1.5;x+=.075)for(double y=-1+.025;y<1;y+=.075)
    cloud->push_back(pcl::PointXYZ(x,y,.003*std::sin(3*x+2*y)));
  Config c;c.body_length=.66;c.body_width=.58;c.normal_radius=.10;c.support_distance=.12;
  c.connection_distance=.25;c.normal_thickness=.02;c.plane_distance=.015;
  EXPECT_THROW(TerrainGraph(cloud,c),std::invalid_argument);
  c.normal_fallback_radius=.15;
  TerrainGraph graph(cloud,c);
  EXPECT_NO_THROW(graph.plan({-1,0,0},{1,0,0}));
  EXPECT_THROW(graph.snap({4,0,0}),std::invalid_argument);
}

TEST(RectangularTerrain, MeasuredCellCentreCanRescueDensityBiasedCentroidNearWall) {
  auto dense=std::make_shared<Cloud>();plane(*dense,-1.2,1.2,0,-1.2,1.2);
  auto cloud=std::make_shared<Cloud>();
  for(const auto &p:*dense) {
    if(p.x>=0&&p.x<.105&&p.y>=0&&p.y<.15)continue;
    cloud->push_back(p);
  }
  for(double y=-1.2;y<1.2;y+=.035)for(double z=0;z<.8;z+=.035)
    cloud->push_back(pcl::PointXYZ(.4,y,z));
  Config c;c.body_length=.66;c.body_width=.58;c.body_top=.5;c.support_distance=.12;
  TerrainGraph graph(cloud,c);
  bool rescued=false;
  for(const auto &n:graph.nodes())if(n.point.x()>=0&&n.point.x()<.15&&n.point.y()>=0&&n.point.y()<.15) {
    rescued|=n.valid&&std::abs(n.point.x()-.075)<1e-8&&std::abs(n.point.y()-.075)<1e-8;
  }
  EXPECT_TRUE(rescued);
  // The rescue must not make the wall traversable.
  EXPECT_THROW(graph.plan({-.8,0,0},{.8,0,0}),std::invalid_argument);
}

TEST(RectangularTerrain, TerrainFollowingKeepsWallsGapsAndStackedFloorsBlocked) {
  for (double resolution : {.10, .15}) {
    SCOPED_TRACE(resolution);
    Config c;c.body_length=.66;c.body_width=.58;c.connection_distance=.25;
    c.resolution=resolution;
    c.normal_radius=.10;c.normal_fallback_radius=.15;c.normal_thickness=.02;c.plane_distance=.015;
    c.support_distance=.12;c.terrain_tolerance=.02;c.body_top=.5;
    auto gap=std::make_shared<Cloud>();plane(*gap,-1.2,-.25);plane(*gap,.25,1.2);
    TerrainGraph gap_graph(gap,c);
    EXPECT_THROW(gap_graph.plan({-.8,0,0},{.8,0,0}),std::invalid_argument);
    auto wall=std::make_shared<Cloud>();plane(*wall,-1.2,1.2);
    for(double y=-.6;y<.6;y+=.035)for(double z=0;z<1.1;z+=.035)
      wall->push_back(pcl::PointXYZ(0,y,z));
    TerrainGraph wall_graph(wall,c);
    EXPECT_THROW(wall_graph.plan({-.8,0,0},{.8,0,0}),std::invalid_argument);
    auto floors=std::make_shared<Cloud>();plane(*floors,-1.2,1.2);plane(*floors,-1.2,1.2,1.5);
    TerrainGraph floors_graph(floors,c);
    EXPECT_THROW(floors_graph.plan({-.8,0,0},{.8,0,1.5}),std::invalid_argument);
    auto narrow=std::make_shared<Cloud>();plane(*narrow,-1.2,1.2,0,-.25,.25);
    for(double x=-1.2;x<1.2;x+=.035)for(double z=0;z<.8;z+=.035) {
      narrow->push_back(pcl::PointXYZ(x,-.25,z));narrow->push_back(pcl::PointXYZ(x,.25,z));
    }
    TerrainGraph narrow_graph(narrow,c);
    EXPECT_THROW(narrow_graph.plan({-.8,0,0},{.8,0,0}),std::invalid_argument);
    auto tall_step=std::make_shared<Cloud>();plane(*tall_step,-1.2,0);plane(*tall_step,0,1.2,.35);
    TerrainGraph step_graph(tall_step,c);
    EXPECT_THROW(step_graph.plan({-.8,0,0},{.8,0,.35}),std::invalid_argument);
  }
}

TEST(RectangularTerrain, LongFootprintDoesNotTreatConnectedTreadsAsBodyObstacles) {
  auto stairs=std::make_shared<Cloud>();plane(*stairs,-1.2,0,0,-.8,.8);
  for(int step=0;step<8;++step) {
    const double x=step*.22,z=(step+1)*.17;
    plane(*stairs,x,x+.22,z,-.8,.8);
    for(double y=-.8;y<.8;y+=.035)for(double h=z-.17;h<z;h+=.025)
      stairs->push_back(pcl::PointXYZ(x,y,h));
  }
  plane(*stairs,1.76,2.8,1.36,-.8,.8);
  Config c;c.body_length=.66;c.body_width=.58;c.connection_distance=.25;
  c.support_distance=.12;c.terrain_tolerance=.02;c.body_top=.5;
  TerrainGraph graph(stairs,c);
  EXPECT_NO_THROW(graph.plan({-.8,0,0},{2.4,0,1.36}));
}

TEST(RectangularTerrain, TurnChecksSweepBetweenIndividuallyClearHeadings) {
  Config c;c.body_length=.66;c.body_width=.58;c.body_top=.5;
  auto cloud=std::make_shared<Cloud>();plane(*cloud,-1.2,1.2,0,-1.2,1.2);
  TerrainGraph baseline(cloud,c);
  const auto centre=baseline.nodes()[baseline.snap({0,0,0})].point;
  cloud->push_back(pcl::PointXYZ(centre.x()+.4,centre.y(),.4));
  TerrainGraph graph(cloud,c);
  const int current=graph.snap(centre);
  const int previous=graph.snap(centre+Eigen::Vector3d(-.6,0,0));
  const int next=graph.snap(centre+Eigen::Vector3d(0,.6,0));
  const auto a=graph.nodes()[previous].point,b=graph.nodes()[current].point,d=graph.nodes()[next].point;
  EXPECT_TRUE(graph.inspectConnection(a,b)["body_clear"].get<bool>());
  EXPECT_TRUE(graph.inspectConnection(b,d)["body_clear"].get<bool>());
  EXPECT_FALSE(graph.turnClear(previous,current,next));
}

TEST(Terrain, WallBlocksTravel) {
  auto cloud = std::make_shared<Cloud>(); plane(*cloud, -1.2, 1.2);
  for (double y = -.6; y < .6; y += .035) for (double z = 0; z < 1.1; z += .035)
    cloud->push_back(pcl::PointXYZ(0, y, z));
  TerrainGraph graph(cloud);
  EXPECT_THROW(graph.plan({-.8, 0, 0}, {.8, 0, 0}), std::invalid_argument);
}

TEST(Terrain, LowCeilingBlocksFloor) {
  auto cloud = std::make_shared<Cloud>(); plane(*cloud, -.8, .8); plane(*cloud, -.8, .8, .6);
  TerrainGraph graph(cloud);
  EXPECT_THROW(graph.snap({0, 0, 0}), std::invalid_argument);
}

TEST(Terrain, TooHighStepCannotConnect) {
  auto cloud = std::make_shared<Cloud>(); plane(*cloud, -1.2, 0); plane(*cloud, 0, 1.2, .5);
  TerrainGraph graph(cloud);
  EXPECT_THROW(graph.plan({-.8, 0, 0}, {.8, 0, .5}), std::invalid_argument);
}

TEST(Terrain, DenseRisersPreserveTreadsAcrossCellAlignments) {
  for (double offset : {0., .037, .081}) {
    auto cloud = std::make_shared<Cloud>();
    plane(*cloud, offset - 1.2, offset, 0, -.75, .75);
    constexpr int steps = 10;
    constexpr double tread = .28, rise = .16;
    for (int step = 0; step < steps; ++step) {
      const double front = offset + step * tread, top = (step + 1) * rise;
      plane(*cloud, front, front + tread, top, -.75, .75);
      for (double y = -.735; y < .75; y += .035)
        for (double z = top - rise + .005; z < top; z += .012)
          cloud->push_back(pcl::PointXYZ(front + .005, y, z));
    }
    plane(*cloud, offset + steps * tread, offset + steps * tread + 1.2, steps * rise, -.75, .75);
    TerrainGraph graph(cloud);
    const Eigen::Vector3d start{offset - .7, 0, 0}, goal{offset + steps * tread + .7, 0, steps * rise};
    const auto up = graph.plan(start, goal), down = graph.plan(goal, start);
    EXPECT_NEAR(up["ascent"].get<double>(), steps * rise, .03);
    EXPECT_NEAR(down["descent"].get<double>(), steps * rise, .03);
    EXPECT_GT(up["length"].get<double>(), steps * tread + 1.4);
  }
}

TEST(Terrain, StepTransitionsUseTheConfiguredSurfaceSlope) {
  auto cloud = std::make_shared<Cloud>();
  plane(*cloud, -1.2, 0, 0, -.75, .75);
  plane(*cloud, 0, 1.2, .16, -.75, .75);
  for (double y = -.735; y < .75; y += .035)
    for (double z = .005; z < .16; z += .012)
      cloud->push_back(pcl::PointXYZ(.005, y, z));
  for (auto &point : *cloud) point.z += .27 * point.y;
  Config config;
  config.max_slope_deg = 25;
  TerrainGraph graph(cloud, config);
  const auto path = graph.plan({-.7, 0, 0}, {.7, 0, .16});
  EXPECT_GT(path["ascent"].get<double>(), .1);
  config.max_slope_deg = 10;
  EXPECT_THROW(TerrainGraph(cloud, config), std::invalid_argument);
}

TEST(Terrain, GentleRampIsWalkableAndSteepRampRejected) {
  auto make_ramp = [](double slope) {
    auto cloud = std::make_shared<Cloud>(); plane(*cloud, -1.2, 1.2, 0, -.8, .8);
    for (auto &p : *cloud) p.z = p.x * slope;
    return cloud;
  };
  TerrainGraph ramp(make_ramp(.2));
  auto path = ramp.plan({-.8, 0, -.16}, {.8, 0, .16});
  EXPECT_GT(path["ascent"].get<double>(), .25);
  for (const auto &section : path["sections"]) EXPECT_EQ(section["mode"], "walk");
  EXPECT_THROW(TerrainGraph(make_ramp(1)), std::invalid_argument);
}

TEST(Terrain, ClearanceCostPullsWallSideEndpointsTowardCorridorMiddle) {
  auto cloud = std::make_shared<Cloud>(); plane(*cloud, 0, 8, 0, -.75, .75);
  for (double x = .015; x < 8; x += .04) for (double z = .35; z < 1; z += .04) {
    cloud->push_back(pcl::PointXYZ(x, -.75, z));
    cloud->push_back(pcl::PointXYZ(x, .75, z));
  }
  Config shortest_config; shortest_config.clearance_weight = 0;
  TerrainGraph shortest(cloud, shortest_config), centered(cloud);
  auto baseline = shortest.plan({.6, -.45, 0}, {7.3, -.45, 0});
  auto improved = centered.plan({.6, -.45, 0}, {7.3, -.45, 0});
  auto middle_deviation = [](const Json &path) {
    double total = 0; int count = 0;
    for (const auto &p : path["points"]) if (p[0].get<double>() > 2.5 && p[0].get<double>() < 5.5) {
      total += std::abs(p[1].get<double>()); ++count;
    }
    EXPECT_GT(count, 0);
    return total / std::max(1, count);
  };
  EXPECT_GT(middle_deviation(baseline), .30);
  EXPECT_LT(middle_deviation(improved), .15);
  EXPECT_GE(improved["length"].get<double>(), baseline["length"].get<double>());
  // A soft preference must not relocate the requested endpoints.
  EXPECT_EQ(improved["points"].front(), baseline["points"].front());
  EXPECT_EQ(improved["points"].back(), baseline["points"].back());
}

TEST(Terrain, ClearanceCostKeepsUMiddleAwayFromInnerCorner) {
  auto cloud = std::make_shared<Cloud>();
  plane(*cloud, 0, 1.2, 0, 0, 5);
  plane(*cloud, 2.4, 3.6, 0, 0, 5);
  plane(*cloud, 0, 3.6, 0, -1.2, 0);
  Config shortest_config; shortest_config.clearance_weight = 0;
  TerrainGraph shortest(cloud, shortest_config), centered(cloud);
  auto baseline = shortest.plan({.6, 4.5, 0}, {3, 4.5, 0});
  auto improved = centered.plan({.6, 4.5, 0}, {3, 4.5, 0});
  auto leg_deviation = [](const Json &path) {
    double total = 0; int count = 0;
    for (const auto &p : path["points"]) {
      double x = p[0], y = p[1];
      if (y > 1 && y < 3.5) { total += std::abs(x - (x < 1.8 ? .6 : 3.0)); ++count; }
    }
    EXPECT_GT(count, 0);
    return total / std::max(1, count);
  };
  EXPECT_LT(leg_deviation(improved), .15);
  EXPECT_LT(leg_deviation(improved), leg_deviation(baseline) * .6);
}

TEST(Terrain, ClearanceRemainsLocalToEachFloorAndDoesNotBlockNarrowPassages) {
  auto lower = std::make_shared<Cloud>(); plane(*lower, -1.5, 1.5, 0, -.4, .4);
  TerrainGraph single(lower);
  auto stacked = std::make_shared<Cloud>(*lower); plane(*stacked, -1.5, 1.5, 2.4, -.4, .4);
  TerrainGraph multiple(stacked);
  auto first = single.plan({-1, 0, 0}, {1, 0, 0});
  auto second = multiple.plan({-1, 0, 0}, {1, 0, 0});
  EXPECT_EQ(first["points"], second["points"]);
  EXPECT_EQ(first["min_boundary_distance"], second["min_boundary_distance"]);
  EXPECT_NEAR(first["cost"].get<double>(), second["cost"].get<double>(), 1e-9);
}

TEST_F(Switchback, ClearanceCostKeepsStairFlightsNearTheirMiddle) {
  auto path = graph->plan({-1, 0, 0}, {-1, 0, 2.4});
  double deviation = 0; int count = 0;
  for (const auto &p : path["points"]) {
    double y = p[1], z = p[2];
    if (z > .3 && z < 1.05) { deviation += std::abs(y); ++count; }
    if (z > 1.5 && z < 2.1) { deviation += std::abs(y - 1.5); ++count; }
  }
  ASSERT_GT(count, 0);
  EXPECT_LT(deviation / count, .17);
}

TEST_F(Switchback, LegacyProjectIsUpgradedWithoutChangingOriginalFiles) {
  TemporaryDirectory directory;
  saveProject(directory.path, "legacy", *graph);
  auto project = directory.path / "legacy";
  auto read_json = [](const fs::path &path) { std::ifstream file(path); return Json::parse(file); };
  auto old_cache = graph->cache();
  old_cache["algorithm_version"] = 1;
  old_cache["revision"] = "legacy-revision";
  old_cache["config"].erase("clearance_weight"); old_cache["config"].erase("clearance_distance");
  for (auto &n : old_cache["nodes"]) n.erase(n.begin() + 7);
  for (auto &edge : old_cache["edges"]) edge[2] = 1;  // Stale old costs must be replaced.
  { std::ofstream file(project / "navigation.json"); file << old_cache.dump(); }
  auto metadata = read_json(project / "map.json");
  metadata["revision"] = "legacy-revision"; metadata["config"] = old_cache["config"];
  metadata["files"]["navigation.json"] = fileSha256(project / "navigation.json");
  { std::ofstream file(project / "map.json"); file << metadata.dump(); }
  auto old_hash = fileSha256(project / "navigation.json");
  auto loaded = loadProject(directory.path, "legacy");
  EXPECT_NE(loaded->revision(), "legacy-revision");
  EXPECT_EQ(loaded->cache()["algorithm_version"], 5);
  EXPECT_EQ(loaded->plan({-1, 0, 0}, {-1, 0, 2.4}), graph->plan({-1, 0, 0}, {-1, 0, 2.4}));
  EXPECT_EQ(fileSha256(project / "navigation.json"), old_hash);
}

TEST_F(Switchback, CacheRoundTripPreservesRouteAndDetectsChangedArtifacts) {
  TemporaryDirectory directory;
  saveProject(directory.path, "building", *graph);
  auto loaded = loadProject(directory.path, "building");
  EXPECT_EQ(loaded->plan({-1, 0, 0}, {-1, 0, 2.4}), graph->plan({-1, 0, 0}, {-1, 0, 2.4}));
  EXPECT_THROW(saveProject(directory.path, "building", *graph), std::invalid_argument);
  { std::ofstream file(directory.path / "building/cloud.pcd", std::ios::app); file << "changed"; }
  EXPECT_THROW(loadProject(directory.path, "building"), std::invalid_argument);
}

TEST_F(Switchback, KeyframesAreSavedWithProjectAndCheckedOnSameNameLoad) {
  TemporaryDirectory directory;
  const auto csv=directory.path/"poses.csv";
  {std::ofstream out(csv);out<<"id,timestamp,px,py,pz,qx,qy,qz,qw\n0,1,0,0,.65,0,0,0,1\n";}
  saveProject(directory.path,"history",*graph,Json::object(),csv);
  EXPECT_EQ(fileSha256(csv),fileSha256(directory.path/"history/localization_keyframes.csv"));
  EXPECT_NO_THROW(loadProject(directory.path,"history"));
  EXPECT_THROW(saveProject(directory.path,"history",*graph,Json::object(),csv),std::invalid_argument);
  {std::ofstream out(directory.path/"history/localization_keyframes.csv",std::ios::app);out<<"changed";}
  EXPECT_THROW(loadProject(directory.path,"history"),std::invalid_argument);
}

TEST(Pcd, AllPclEncodingsAndExtraFields) {
  TemporaryDirectory directory;
  auto cloud = switchbackCloud();
  for (const std::string encoding : {"ascii", "binary", "compressed"}) {
    fs::path path = directory.path / (encoding + ".pcd");
    if (encoding == "ascii") pcl::io::savePCDFileASCII(path.string(), *cloud);
    else if (encoding == "binary") pcl::io::savePCDFileBinary(path.string(), *cloud);
    else pcl::io::savePCDFileBinaryCompressed(path.string(), *cloud);
    auto loaded = readPcd(path);
    ASSERT_EQ(loaded->size(), cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) EXPECT_NEAR((*loaded)[i].z, (*cloud)[i].z, 1e-5);
  }
  pcl::PointCloud<pcl::PointXYZI> intensity;
  for (const auto &p : *cloud) { pcl::PointXYZI q; q.x=p.x; q.y=p.y; q.z=p.z; q.intensity=42; intensity.push_back(q); }
  pcl::io::savePCDFileBinary((directory.path / "intensity.pcd").string(), intensity);
  EXPECT_EQ(readPcd(directory.path / "intensity.pcd")->size(), cloud->size());
  { std::ofstream file(directory.path / "bad.pcd"); file << "POINTS 999999999\nSIZE 4 4 4\nDATA binary\n"; }
  EXPECT_THROW(readPcd(directory.path / "bad.pcd"), std::invalid_argument);
}

TEST(Input, InvalidConfigAndMapIdsRejected) {
  for (const std::string id : {"../escape", "/tmp/escape", "", "a/b", ".hidden"})
    EXPECT_THROW(mapPath("/tmp/maps", id), std::invalid_argument);
  EXPECT_THROW(Config::parse({{"max_step", -1}}), std::invalid_argument);
  EXPECT_THROW(Config::parse({{"max_step", "wrong"}}), std::invalid_argument);
  EXPECT_THROW(Config::parse({{"min_points", 3.5}}), std::invalid_argument);
  EXPECT_THROW(Config::parse({{"unknown_parameter", 2}}), std::invalid_argument);
  EXPECT_THROW(Config::parse({{"resolution", .0001}}), std::invalid_argument);
  EXPECT_THROW(Config::parse({{"clearance_weight", -1}}), std::invalid_argument);
  EXPECT_THROW(Config::parse({{"clearance_distance", 0}}), std::invalid_argument);
  EXPECT_NO_THROW(Config::parse({{"clearance_weight", 0}}));
  EXPECT_NO_THROW(Config::parse({{"footprint_padding", 0}}));
  EXPECT_THROW(Config::parse({{"footprint_padding", -.01}}),std::invalid_argument);
  Config config; config.resolution = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(config.validate(), std::invalid_argument);
}

TEST(Collision, ContinuousSweepCatchesGrazingContactBetweenSamplePoses) {
  const Eigen::Vector3d a(0,0,0),b(.15,0,0),obstacle(.025,.2199,.4);
  for (double x : {0.,.05,.10,.15}) {
    const Eigen::Vector3d pose(x,0,0);
    EXPECT_FALSE(sweptCylinderContact(pose,pose,obstacle,.22,.30,.50));
  }
  EXPECT_TRUE(sweptCylinderContact(a,b,obstacle,.22,.30,.50));
  EXPECT_TRUE(sweptCylinderContact(b,a,obstacle,.22,.30,.50));
  EXPECT_FALSE(sweptCylinderContact(a,b,{.025,.2199,2.4},.22,.30,.50));
}

TEST(Collision, SweepIntersectsHorizontalAndVerticalIntervalsTogether) {
  const Eigen::Vector3d a(0,0,0),b(1,0,1),obstacle(.25,0,.8);
  EXPECT_TRUE(sweptCylinderContact(a,b,obstacle,.06,.3,.5));
  EXPECT_TRUE(sweptCylinderContact(b,a,obstacle,.06,.3,.5));
  EXPECT_FALSE(sweptCylinderContact(a,b,obstacle,.02,.3,.5));
}

TEST(Terrain, ExplicitPaddingAndCollisionWitnessUseActualBodyHeight) {
  auto cloud=std::make_shared<Cloud>();plane(*cloud,-1.2,1.2,0,-1.2,1.2);
  Config config;config.body_top=.5;
  TerrainGraph floor(cloud,config);
  const auto center=floor.nodes()[floor.snap({0,0,0})].point;
  cloud->push_back(pcl::PointXYZ(center.x(),center.y()+.245,.4));
  TerrainGraph unpadded(cloud,config);
  EXPECT_TRUE(unpadded.inspectSurface(center)["body_clear"].get<bool>());
  config.footprint_padding=.05;
  TerrainGraph padded(cloud,config);
  const auto diagnosis=padded.inspectSurface(center);
  EXPECT_FALSE(diagnosis["body_clear"].get<bool>());
  EXPECT_TRUE(diagnosis.contains("collision_point"));
  EXPECT_TRUE(diagnosis["support_valid"].get<bool>());
  EXPECT_EQ(diagnosis["reason"],"body_collision");
  EXPECT_EQ(padded.inspectSurface({0,0,10})["reason"],"no_observed_surface_at_height");
  auto reloaded=TerrainGraph::fromCache(cloud,padded.cache());
  EXPECT_EQ(reloaded->inspectSurface(center),diagnosis);
}

TEST(Terrain, LegacySampledCollisionCacheIsRebuiltWithoutChangingSourceCloud) {
  auto cloud=std::make_shared<Cloud>();plane(*cloud,-1.2,1.2,0,-1.2,1.2);
  TerrainGraph graph(cloud);
  auto legacy=graph.cache();legacy["algorithm_version"]=2;
  legacy["config"].erase("footprint_padding");
  for(auto &node:legacy["nodes"]) {node.erase(node.begin()+8,node.end());}
  const auto loaded=TerrainGraph::fromCache(cloud,legacy);
  EXPECT_EQ(loaded->cache(),graph.cache());
}
