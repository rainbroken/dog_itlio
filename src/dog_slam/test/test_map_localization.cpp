#include "dog_slam/map_localization.hpp"
#include "dog_slam/global_relocalizer.hpp"
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <fstream>
#include <unistd.h>

using namespace dog_slam;
namespace {
LocalizationCloud::Ptr room() {
  auto c = std::make_shared<LocalizationCloud>();
  for (int i=0;i<=60;++i) for(int j=0;j<=40;++j) {
    const float x=-3.f+i*.1f, y=-2.f+j*.1f;
    c->push_back(pcl::PointXYZ(x,y,0));
    c->push_back(pcl::PointXYZ(x,-2,j*.075f));
    c->push_back(pcl::PointXYZ(-3,x*.65f,j*.075f));
    if(i<20) c->push_back(pcl::PointXYZ(1.f+i*.05f,.7f,j*.05f));
  }
  return c;
}
Pose3 pose(double x,double y,double z,double yaw=.0) {
  Pose3 p=Pose3::Identity(); p.translation()=Eigen::Vector3d(x,y,z);
  p.linear()=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())*
    Eigen::AngleAxisd(.06,Eigen::Vector3d::UnitY())).toRotationMatrix(); return p;
}
LocalizationCloud::Ptr scan(LocalizationCloud::ConstPtr c,const Pose3 &p) {
  auto out=std::make_shared<LocalizationCloud>();
  const Eigen::Matrix4f m=p.inverse().matrix().cast<float>();
  pcl::transformPointCloud(*c,*out,m); return out;
}
}
TEST(MapLocalization, FullSE3Alignment) {
  auto map=room(); MapMatcher matcher(map);
  const auto truth=pose(.2,.1,.65,.12);
  auto guess=truth; guess.translation()+=Eigen::Vector3d(.15,-.1,.08);
  auto result=matcher.match(scan(map,truth),guess,false);
  ASSERT_TRUE(result.accepted)<<result.reason<<" geometry="<<result.geometry_ratio;
  EXPECT_LT((result.map_to_base.translation()-truth.translation()).norm(),.04);
  EXPECT_LT(rotationDistance(result.map_to_base,truth),.025);
}
TEST(MapLocalization, ConfirmationLossAndRecovery) {
  auto map=room(); LocalizationOptions options; options.recovery_candidates=2;
  LocalizationEngine engine(map,options);
  auto truth=pose(.2,.1,.65,.12), odom=pose(4,-2,1.2,-.3);
  auto cloud=scan(map,truth);
  EXPECT_FALSE(engine.update(cloud,odom).valid);
  engine.initialize(truth,odom);
  for(int i=0;i<2;++i) EXPECT_FALSE(engine.update(cloud,odom).valid);
  auto state=engine.update(cloud,odom); ASSERT_TRUE(state.valid)<<state.match.reason;
  EXPECT_LT((state.map_to_odom*odom).matrix().operator-(truth.matrix()).norm(),.06);
  // Climbing changes both height and body pitch: correction stays in full SE(3).
  auto moved=pose(.45,.18,1.05,.2);
  moved.linear()=moved.linear()*Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitY()).toRotationMatrix();
  auto moved_odom=state.map_to_odom.inverse()*moved;
  auto climb=engine.update(scan(map,moved),moved_odom);
  ASSERT_TRUE(climb.valid)<<climb.match.reason;
  EXPECT_LT(((climb.map_to_odom*moved_odom).translation()-moved.translation()).norm(),.04);
  EXPECT_LT(rotationDistance(climb.map_to_odom*moved_odom,moved),.025);
  auto empty=std::make_shared<LocalizationCloud>();
  for(int i=0;i<3;++i) EXPECT_FALSE(engine.update(empty,odom).valid);
  EXPECT_EQ(engine.status().state,LocalizationState::Lost);
  for(int i=0;i<3;++i) state=engine.update(cloud,odom);
  EXPECT_TRUE(state.valid)<<state.match.reason;
  engine.reset(); EXPECT_FALSE(engine.status().has_prior);
  EXPECT_FALSE(engine.relocalize());
}
TEST(MapLocalization, RejectsUnobservablePlaneAndWrongFloor) {
  auto plane=std::make_shared<LocalizationCloud>();
  for(int x=-30;x<30;++x) for(int y=-30;y<30;++y)
    plane->push_back(pcl::PointXYZ(x*.1f,y*.1f,0));
  MapMatcher flat(plane); const auto truth=pose(0,0,.7);
  EXPECT_FALSE(flat.match(scan(plane,truth),truth,false).accepted);
  auto map=room(); MapMatcher matcher(map); auto wrong=truth; wrong.translation().z()+=3;
  EXPECT_FALSE(matcher.match(scan(map,truth),wrong,false).accepted);
}
TEST(MapLocalization, InvalidConfigurationAndPose) {
  LocalizationOptions o; o.voxel_size=0; EXPECT_THROW(o.validate(),std::invalid_argument);
  auto p=Pose3::Identity(); p.linear()(0,0)=2; EXPECT_FALSE(validPose(p));
  EXPECT_THROW(readLocalizationMap("/does/not/exist.pcd"),std::exception);
}
TEST(MapLocalization, PcdEncodingsAndSameNameReload) {
  namespace fs=std::filesystem;
  auto dir=fs::temp_directory_path()/("dog-localization-test-"+std::to_string(getpid()));
  fs::create_directories(dir);
  struct Cleanup {fs::path path; ~Cleanup(){fs::remove_all(path);}} cleanup{dir};
  auto map=room(); auto path=(dir/"localization.pcd").string();
  for(int mode=0;mode<3;++mode) {
    if(mode==0) pcl::io::savePCDFileASCII(path,*map);
    else if(mode==1) pcl::io::savePCDFileBinary(path,*map);
    else pcl::io::savePCDFileBinaryCompressed(path,*map);
    auto loaded=readLocalizationMap(dir.string());
    ASSERT_EQ(loaded->size(),map->size());
    EXPECT_NEAR(loaded->front().x,map->front().x,1e-5);
  }
  for(auto &p:*map) p.z+=3;
  pcl::io::savePCDFileBinaryCompressed(path,*map);
  EXPECT_NEAR(readLocalizationMap(dir.string())->front().z,3,1e-5);
}

TEST(MapLocalization, InitialPositionAndHeadingAreRefinedBeforePublishing) {
  auto map=room();LocalizationEngine engine(map);
  const auto truth=pose(.2,.1,.65,1.1);
  auto guess=pose(.4,-.1,.72,1.35),odom=pose(8,-4,1.7,-.4);
  engine.initialize(guess,odom);
  EXPECT_FALSE(engine.status().valid);
  for(int i=0;i<3;++i)engine.update(scan(map,truth),odom);
  ASSERT_TRUE(engine.status().valid)<<engine.status().match.reason;
  EXPECT_LT(rotationDistance(engine.status().map_to_odom*odom,truth),.06);
  EXPECT_LT(((engine.status().map_to_odom*odom).translation()-truth.translation()).norm(),.08);
}

TEST(GlobalRelocalization, FindsRemoteMapPoseWithoutPositionOrYawSeed) {
  auto map=room();for(auto &p:*map){p.x+=30;p.y+=10;p.z+=2.4;}
  LocalizationOptions o;o.global.enabled=true;o.global.time_budget=5;
  LocalizationEngine engine(map,o);
  const auto truth=pose(30.2,10.1,3.05,2.2),odom=pose(-8,4,.65,-.4);
  auto cloud=scan(map,truth);
  auto first=engine.update(cloud,odom);
  ASSERT_TRUE(first.match.accepted)<<first.match.reason<<" duration="<<first.match.duration_ms;
  EXPECT_FALSE(first.valid);EXPECT_EQ(first.confirmations,1);
  EXPECT_EQ(first.source,"global_map_search");
  const auto second=engine.update(cloud,odom);auto result=engine.update(cloud,odom);
  ASSERT_TRUE(result.valid)<<result.match.reason<<" confirmations="<<result.confirmations
    <<" second="<<second.match.reason<<" confirmations="<<second.confirmations
    <<" first="<<first.match.map_to_base.translation().transpose()
    <<" second_pose="<<second.match.map_to_base.translation().transpose()
    <<" final="<<result.match.map_to_base.translation().transpose()
    <<" rotation_delta="<<rotationDistance(first.match.map_to_base,second.match.map_to_base)
    <<" correction_delta="<<((first.match.map_to_base*odom.inverse()).translation()-(second.match.map_to_base*odom.inverse()).translation()).norm();
  EXPECT_LT(((result.map_to_odom*odom).translation()-truth.translation()).norm(),.1);
  EXPECT_LT(rotationDistance(result.map_to_odom*odom,truth),.06);
  // An explicit seed takes priority over the optional startup search.
  engine.initialize(truth,odom);EXPECT_EQ(engine.status().source,"initial_pose");
  EXPECT_EQ(engine.status().state,LocalizationState::Initializing);
}

TEST(GlobalRelocalization, RepeatedFloorsAreRejectedAsAmbiguous) {
  auto original=room(),map=std::make_shared<LocalizationCloud>(*original);
  for(auto p:*original){p.z+=6;map->push_back(p);}
  LocalizationOptions o;o.global.enabled=true;o.global.time_budget=5;o.global.refine_candidates=24;
  MapMatcher matcher(map,o);GlobalRelocalizer global(matcher,o);
  auto result=global.search(scan(original,pose(.2,.1,.65,.8)),pose(0,0,.65),matcher);
  EXPECT_FALSE(result.accepted)<<result.reason;
  EXPECT_EQ(result.reason,"ambiguous_global_relocalization");
}

TEST(GlobalRelocalization, DisabledAndIncompleteSearchCannotConfirmLocalization) {
  auto map=room();LocalizationEngine disabled(map);
  EXPECT_FALSE(disabled.globalRelocalize());
  EXPECT_FALSE(disabled.update(scan(map,pose(.2,.1,.65)),pose(0,0,.65)).has_prior);
  LocalizationOptions o;o.global.enabled=true;o.global.max_hypotheses=24;
  LocalizationEngine limited(map,o);
  auto result=limited.update(scan(map,pose(.2,.1,.65)),pose(0,0,.65));
  EXPECT_FALSE(result.valid);EXPECT_FALSE(result.has_prior);
  EXPECT_EQ(result.match.reason,"global_hypothesis_budget_exceeded");
  o.global.max_hypotheses=30000;o.global.time_budget=1e-12;
  LocalizationEngine timed(map,o);
  auto timeout=timed.update(scan(map,pose(.2,.1,.65)),pose(0,0,.65));
  EXPECT_FALSE(timeout.valid);EXPECT_EQ(timeout.match.reason,"global_time_budget_exceeded");
}

TEST(MapLocalization, ConfirmationDoesNotDependOnDistanceToOdomOrigin) {
  auto map=room();LocalizationEngine engine(map);
  auto truth=pose(.2,.1,.65,.8),odom=pose(2000,-1000,.65,-.2);
  engine.initialize(truth,odom);
  for(int i=0;i<3;++i)engine.update(scan(map,truth),odom);
  ASSERT_TRUE(engine.status().valid)<<engine.status().match.reason;
  EXPECT_LT(((engine.status().map_to_odom*odom).translation()-truth.translation()).norm(),.08);
}

TEST(MapLocalization, WrongInitialPoseStaysInvalidAndCanBeReinitialized) {
  auto map=room();LocalizationEngine engine(map);
  const auto truth=pose(.2,.1,.65,1.1),odom=pose(5,2,.65,-.3);
  auto observed=scan(map,truth);
  // Small deterministic measurement perturbations; no real accuracy claim.
  for(size_t i=0;i<observed->size();++i) {
    (*observed)[i].x+=.008*std::sin(double(i));(*observed)[i].z+=.006*std::cos(double(i));
  }
  engine.initialize(pose(25,20,6,-1),odom);
  for(int i=0;i<4;++i)EXPECT_FALSE(engine.update(observed,odom).valid);
  engine.initialize(pose(.3,0,.7,1.25),odom);
  for(int i=0;i<5&&!engine.status().valid;++i)engine.update(observed,odom);
  ASSERT_TRUE(engine.status().valid)<<engine.status().match.reason;
  EXPECT_LT(((engine.status().map_to_odom*odom).translation()-truth.translation()).norm(),.1);
  EXPECT_LT(rotationDistance(engine.status().map_to_odom*odom,truth),.08);
}

TEST(GlobalRelocalization, HistoricalImuSeedsFindRemoteBodyWithUnknownYawAndHeight) {
  auto map=room();for(auto &p:*map){p.x+=30;p.y-=12;p.z+=4;}
  const auto truth=pose(30.2,-11.9,4.65,2.2),odom=pose(-8,4,.65,-.4);
  auto imu=truth;imu.translation()+=truth.linear()*Eigen::Vector3d(.25,.02,.1);
  LocalizationOptions o;o.global.enabled=true;o.global.time_budget=5;
  o.global.body_height=1.8; // Historical metric height must not use nominal height.
  LocalizationEngine engine(map,o,{pose(32,-12,4.8),imu});
  auto cloud=scan(map,truth);auto first=engine.update(cloud,odom);
  ASSERT_TRUE(first.match.accepted)<<first.match.reason;
  EXPECT_EQ(first.source,"historical_keyframes");
  EXPECT_EQ(first.match.scope,"historical_keyframe_search");
  EXPECT_EQ(first.match.hypotheses,2*o.global.yaw_steps);
  EXPECT_FALSE(first.valid);
  for(int i=0;i<2;++i)engine.update(cloud,odom);
  ASSERT_TRUE(engine.status().valid)<<engine.status().match.reason;
  EXPECT_LT(((engine.status().map_to_odom*odom).translation()-truth.translation()).norm(),.1);
  EXPECT_LT(rotationDistance(engine.status().map_to_odom*odom,truth),.06);
}

TEST(GlobalRelocalization, HistoricalRepeatedFloorsRemainAmbiguous) {
  auto original=room(),map=std::make_shared<LocalizationCloud>(*original);
  for(auto p:*original){p.z+=6;map->push_back(p);}
  LocalizationOptions o;o.global.enabled=true;o.global.time_budget=5;o.global.refine_candidates=24;
  MapMatcher matcher(map,o);
  GlobalRelocalizer searcher(matcher,o,{pose(.2,.1,.65),pose(.2,.1,6.65)});
  auto result=searcher.search(scan(original,pose(.2,.1,.65,.8)),pose(0,0,.65),matcher);
  EXPECT_FALSE(result.accepted);EXPECT_EQ(result.reason,"ambiguous_global_relocalization");
}

TEST(MapLocalization, HistoricalArtifactReloadsSameNameAndRejectsMalformedData) {
  namespace fs=std::filesystem;
  auto dir=fs::temp_directory_path()/("dog-keyframes-test-"+std::to_string(getpid()));
  fs::create_directories(dir);
  struct Cleanup {fs::path path;~Cleanup(){fs::remove_all(path);}} cleanup{dir};
  EXPECT_TRUE(readLocalizationKeyframes(dir.string()).empty());
  const auto file=dir/"localization_keyframes.csv";
  for(int height:{2,6}) {
    {std::ofstream out(file);out<<"id,timestamp,px,py,pz,qx,qy,qz,qw\n0,1,30,-12,"<<height<<",0,0,0,1\n";}
    const auto poses=readLocalizationKeyframes(dir.string());
    ASSERT_EQ(poses.size(),1u);EXPECT_DOUBLE_EQ(poses.front().translation().z(),height);
  }
  {std::ofstream out(file);out<<"id,timestamp,px,py,pz,qx,qy,qz,qw\n0,1,0,0,0,0,0,0,0\n";}
  EXPECT_THROW(readLocalizationKeyframes(dir.string()),std::invalid_argument);
}

TEST(MapLocalization, SparseObservedRaysDoNotCountUnobservedMapAsMissingReturns) {
  auto map=room(),sparse=std::make_shared<LocalizationCloud>();
  for(size_t i=0;i<map->size();i+=55)sparse->push_back((*map)[i]);
  LocalizationOptions o;o.min_reverse_overlap=.7;
  MapMatcher matcher(map,o);auto truth=pose(.2,.1,.65,.4);
  auto result=matcher.refine(scan(sparse,truth),truth);
  ASSERT_TRUE(result.accepted)<<result.reason<<" overlap="<<result.overlap<<" reverse="<<result.reverse_overlap;
  EXPECT_GT(result.overlap,.9);
}

TEST(GlobalRelocalization, RetrievalBudgetLimitsScoredCandidatesRatherThanDatabaseSize) {
  auto map=room();LocalizationOptions o;o.global.enabled=true;o.global.time_budget=5;
  o.global.keyframe_candidates=2;o.global.max_hypotheses=2*o.global.yaw_steps;
  const auto truth=pose(.2,.1,.65,.8);
  std::vector<Pose3> history{truth};
  for(int i=0;i<40;++i)history.push_back(pose(100+i,100,.65));
  MapMatcher matcher(map,o);GlobalRelocalizer global(matcher,o,history);
  auto result=global.search(scan(map,truth),pose(0,0,.65),matcher);
  ASSERT_TRUE(result.accepted)<<result.reason;
  EXPECT_EQ(result.hypotheses,2*o.global.yaw_steps);
  EXPECT_LT((result.map_to_base.translation()-truth.translation()).norm(),.1);
}
