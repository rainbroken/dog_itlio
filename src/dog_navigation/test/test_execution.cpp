#include "dog_navigation/path_tracker.hpp"
#include "dog_navigation/timed_trajectory.hpp"
#include "dog_navigation/task_supervisor.hpp"
#include "dog_navigation/safety_arbiter.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include "dog_terrain/terrain_graph.hpp"
#include "dog_terrain/collision_geometry.hpp"
#include "dog_navigation/braking_sweep.hpp"

using namespace dog_navigation;
TEST(Execution, BrakingSweepRetainsCurvedAndMeasuredMomentumGeometry){
  auto straight=brakingSweep({.3,0,0},.15,.5);
  EXPECT_NEAR(straight.back().point.x(),.3*.15+.3*.3/(2*.15),1e-9);
  EXPECT_NEAR(straight.back().point.y(),0,1e-9);
  auto curve=brakingSweep({.6,0,.5},.15,.5);
  EXPECT_NEAR(curve.back().yaw,.5*.15+.5*.5/(2*.5),1e-9);
  ASSERT_GT(curve.back().point.y(),.1);
  Point obstacle=curve.back().point+Point(0,.59,.4);
  EXPECT_FALSE(dog_terrain::sweptCylinderContact(Point::Zero(),Point(1.29,0,0),obstacle,.6,.07,1.));
  bool hit=false;
  for(size_t i=1;i<curve.size();++i){
    EXPECT_EQ(curve[i].point.z(),0);
    hit|=dog_terrain::sweptCylinderContact(curve[i-1].point,curve[i].point,obstacle,.6,.07,1.).has_value();
  }
  EXPECT_TRUE(hit);
  auto reverse=brakingSweep({-.3,0,-.5},.15,.5);
  EXPECT_LT(reverse.back().point.x(),0);EXPECT_LT(reverse.back().yaw,0);
  EXPECT_THROW(brakingSweep({.3,0,0},0,.5),std::invalid_argument);
}
TEST(Execution, RollingRouteReconnectsActualPoseBeforePreservingSharedTarget) {
  ExecutionOptions options;options.startup_linear_speed=.21;options.startup_yaw_rate=.21;
  PathTracker tracker(options);TrackingPose pose;
  pose.ground={3.00,1.39,-.61};pose.yaw=-2.67;
  const Points first{{2.9566,1.3586,-.6012},{2.7554,1.2529,-.5974},{2.5634,1.2612,-.5953}};
  const Points second{{3.0590,1.3530,-.6068},{2.8483,1.2529,-.5992},{2.7554,1.2529,-.5974},{2.5634,1.2612,-.5953}};
  tracker.setRoute(first,{false,false},pose,first.back(),0,false);
  for(int i=0;i<12;++i)ASSERT_FALSE(tracker.update(pose,.05).fault);
  for(int i=0;i<20;++i){
    const bool alternate=i%2;
    tracker.setRoute(alternate?first:second,std::vector<bool>(alternate?2:3,false),pose,first.back(),0,false);
    const auto out=tracker.update(pose,.05);
    EXPECT_FALSE(out.fault)<<out.reason;EXPECT_LT(out.error,.10);
  }
}
TEST(Execution, AlignmentUsesAvailableYawLimitWithoutLongProportionalTail) {
  ExecutionOptions options;options.startup_yaw_rate=.21;
  PathTracker tracker(options);TrackingPose pose;pose.yaw=-.25;
  tracker.setRoute({{0,0,0},{2,0,0}},{false},pose,{2,0,0},0,false);
  TrackResult out;
  for(int i=0;i<40;++i)out=tracker.update(pose,.05);
  EXPECT_EQ(out.phase,"align");EXPECT_NEAR(out.command.yaw,.5,1e-9);
}
TEST(Execution, BrakeCertificateRegulatesTargetBeforeAccelerationAndPreservesDeadZoneFloor) {
  ExecutionOptions options;options.walk_speed=.6;options.startup_linear_speed=.21;options.startup_yaw_rate=.21;
  PathTracker tracker(options);TrackingPose pose;
  tracker.setRoute({{0,0,0},{3,0,0}},{false},pose,{3,0,0},0,false);
  double previous=0;bool moved=false;
  for(int i=0;i<100;++i){
    const auto out=tracker.update(pose,.05,[](const Velocity &v){return v.x<=.32;});
    ASSERT_FALSE(out.fault);EXPECT_LE(out.command.x,.32);
    if(out.command.x>0){EXPECT_GE(out.command.x,.21);moved=true;}
    if(previous>0&&out.command.x>0)EXPECT_LE(std::abs(out.command.x-previous),options.acceleration*.05+1e-9);
    pose.measured=out.command;previous=out.command.x;
  }
  EXPECT_TRUE(moved);
  auto stopped=tracker.update(pose,.05,[](const Velocity &){return false;});
  EXPECT_EQ(speed(stopped.command),0);EXPECT_EQ(stopped.phase,"waiting_safe_velocity");
}
TEST(Execution, NearCornerPlanarRecoveryKeepsCombinedStartupSpeedAndAccelerationBounds) {
  ExecutionOptions options;options.walk_speed=.6;options.startup_linear_speed=.21;options.startup_yaw_rate=.21;
  PathTracker tracker(options);TrackingPose pose;pose.ground={.8,0,0};pose.yaw=std::atan2(.4*.3/std::hypot(1.,.3),.2+.4/std::hypot(1.,.3));
  tracker.setRoute({{0,0,0},{1,0,0},{2,.3,0}},{false,false},pose,{2,.3,0},0,false);
  Velocity previous;bool recovered=false;
  for(int i=0;i<100;++i){
    const auto out=tracker.update(pose,.05,[](const Velocity &v){return std::hypot(v.x,v.y)<1e-9||v.y>=.005;});
    ASSERT_FALSE(out.fault);const double planar=std::hypot(out.command.x,out.command.y);
    if(planar>1e-9){EXPECT_GE(planar,.21-1e-9);EXPECT_LE(planar,.6+1e-9);}
    EXPECT_LE(std::abs(out.command.y),autonomous_lateral_speed+1e-9);
    if(planar>0&&std::hypot(previous.x,previous.y)>0)
      EXPECT_LE(std::hypot(out.command.x-previous.x,out.command.y-previous.y),.5*.05+1e-9);
    recovered|=out.command.y>.01;pose.measured=out.command;previous=out.command;
  }
  EXPECT_TRUE(recovered);
}
TEST(Execution, BrakingCertifiedStairTrackingNeverAddsLateralVelocity) {
  ExecutionOptions options;options.startup_linear_speed=.21;options.startup_yaw_rate=.21;
  PathTracker tracker(options);TrackingPose pose;
  const Points path{{0,0,0},{1,0,.2},{2,.2,.4},{3,.4,.4}};
  tracker.setRoute(path,{true,true,false},pose,path.back(),0,false);
  bool done=false,climbed=false;
  for(int i=0;i<2000&&!done;++i){
    const auto out=tracker.update(pose,.05,[](const Velocity &){return true;});
    ASSERT_FALSE(out.fault)<<out.reason;
    EXPECT_DOUBLE_EQ(out.command.y,0.);
    climbed|=out.phase=="stairs_up";
    pose.ground=out.predicted;pose.yaw=out.predicted_yaw;pose.measured=out.command;done=out.done;
  }
  EXPECT_TRUE(climbed);EXPECT_TRUE(done);
}
TEST(Execution, CertifiedHolonomicCornerDoesNotApplyForwardOnlyCurvatureStop) {
  ExecutionOptions options;options.startup_linear_speed=.21;options.startup_yaw_rate=.21;
  PathTracker tracker(options);TrackingPose pose;pose.ground={.8,0,0};
  pose.yaw=std::atan2(.4*.3/std::hypot(1.,.3),.2+.4/std::hypot(1.,.3));
  tracker.setRoute({{0,0,0},{1,0,0},{2,.3,0}},{false,false},pose,{2,.3,0},0,false);
  TrackResult out;
  for(int i=0;i<30;++i){
    out=tracker.update(pose,.05,[](const Velocity &){return true;});pose.measured=out.command;
  }
  ASSERT_EQ(out.phase,"walk");
  pose.yaw=-1.05;
  out=tracker.update(pose,.05,[](const Velocity &){return true;});
  EXPECT_EQ(out.phase,"walk");EXPECT_GE(std::hypot(out.command.x,out.command.y),.21);
  // The same freedom is needed just after the connected waypoint; treating
  // its outgoing edge as forward-only reintroduces a stop on corner exit.
  pose.ground={1.1,.03,0};pose.measured=out.command;
  out=tracker.update(pose,.05,[](const Velocity &){return true;});
  EXPECT_EQ(out.phase,"walk");EXPECT_GE(std::hypot(out.command.x,out.command.y),.21);
  tracker.setRoute({{1.1,.03,0},{2,.3,0}},{false},pose,{2,.3,0},0,false);
  out=tracker.update(pose,.05,[](const Velocity &){return true;});
  EXPECT_EQ(out.phase,"walk");EXPECT_GE(std::hypot(out.command.x,out.command.y),.21);
}
TEST(Execution, OrdinaryGroundHeadingRemainsResponsiveWhenTranslationIsRegulated) {
  ExecutionOptions options;options.walk_speed=.6;options.startup_linear_speed=.21;options.startup_yaw_rate=.21;
  PathTracker tracker(options);TrackingPose pose;pose.ground={.8,0,0};
  pose.yaw=std::atan2(.4*.3/std::hypot(1.,.3),.2+.4/std::hypot(1.,.3));
  tracker.setRoute({{0,0,0},{1,0,0},{2,.3,0}},{false,false},pose,{2,.3,0},0,false);
  TrackResult out;
  const auto clear=[](const Velocity &v){return std::hypot(v.x,v.y)<=.24;};
  for(int i=0;i<30;++i){out=tracker.update(pose,.05,clear);pose.measured=out.command;}
  ASSERT_EQ(out.phase,"walk");pose.yaw=0;
  for(int i=0;i<30;++i){out=tracker.update(pose,.05,clear);pose.measured=out.command;}
  EXPECT_EQ(out.phase,"walk");EXPECT_LE(std::hypot(out.command.x,out.command.y),.24);
  EXPECT_GT(out.command.yaw,.30);EXPECT_LE(out.command.yaw,options.yaw_rate);
}
TEST(Execution, NonfiniteVelocityFeedbackFaultsBeforeControl) {
  PathTracker tracker;TrackingPose pose;
  pose.measured.yaw=std::numeric_limits<double>::quiet_NaN();
  const auto out=tracker.update(pose,.05);
  EXPECT_TRUE(out.fault);EXPECT_EQ(out.reason,"invalid_control_state_or_period");
  EXPECT_DOUBLE_EQ(speed(out.command),0.);
}
TEST(Execution, VectorAccelerationAllowsSteeringBeforeSpeedRampFinishes) {
  ExecutionOptions options;options.walk_speed=.6;options.startup_linear_speed=.21;
  PathTracker tracker(options);TrackingPose pose;
  tracker.setRoute({{0,0,0},{4,0,0}},{false},pose,{4,0,0},0,false);
  TrackResult out;
  const auto clear=[](const Velocity &){return true;};
  for(int i=0;i<12;++i){out=tracker.update(pose,.05,clear);pose.measured=out.command;}
  ASSERT_EQ(out.phase,"walk");const auto previous=out.command;
  ASSERT_LT(std::hypot(previous.x,previous.y),.5);
  tracker.setRoute({{0,0,0},{2,.4,0},{4,0,0}},{false,false},pose,{4,0,0},0,false);
  out=tracker.update(pose,.05,clear);
  EXPECT_GT(out.command.x,previous.x);EXPECT_GT(out.command.y,1e-4);
  EXPECT_LE(std::hypot(out.command.x-previous.x,out.command.y-previous.y),options.acceleration*.05+1e-9);
}
TEST(Execution, BodyFrameCounterRotationUsesMeasuredYawWithinVelocityBounds) {
  for(double yaw_rate:{-.5,.5}){
    ExecutionOptions options;options.startup_linear_speed=.21;
    PathTracker tracker(options);TrackingPose pose;TrackResult out;
    tracker.setRoute({{0,0,0},{4,0,0}},{false},pose,{4,0,0},0,false);
    const auto clear=[](const Velocity &){return true;};
    for(int i=0;i<30;++i){out=tracker.update(pose,.05,clear);pose.measured=out.command;}
    ASSERT_EQ(out.phase,"walk");ASSERT_DOUBLE_EQ(out.command.y,0.);
    const auto previous=out.command;pose.measured.yaw=yaw_rate;
    out=tracker.update(pose,.05,clear);
    // Holding world-frame travel while the body rotates requires a lateral
    // command opposite to the measured rotation, including during speed ramp.
    EXPECT_LT(out.command.y*yaw_rate,-1e-4);
    EXPECT_LE(std::hypot(out.command.x,out.command.y),options.walk_speed+1e-9);
    EXPECT_GE(std::hypot(out.command.x,out.command.y),.21-1e-9);
    EXPECT_LE(std::hypot(out.command.x-previous.x,out.command.y-previous.y),options.acceleration*.05+1e-9);
  }
}
TEST(Execution, BpxCommandsSkipDeadZoneAndPreserveCornersAndTerrainTransitions){
  ExecutionOptions options;options.startup_linear_speed=.21;options.startup_yaw_rate=.21;
  Points dense{{0,0,0},{.1,0,0},{.2,0,0},{1,0,0},{1,.5,.2},{1,1,.4}};
  PathTracker tracker(options);TrackingPose pose;
  tracker.setRoute(dense,{false,false,false,true,true},pose,dense.back(),0,false);
  ASSERT_EQ(tracker.path().size(),3U);EXPECT_EQ(tracker.path()[1],Point(1,0,0));
  bool done=false,translated=false,rotated=false;
  for(int i=0;i<10000&&!done;++i){
    const auto out=tracker.update(pose,.05);ASSERT_FALSE(out.fault)<<out.reason;
    EXPECT_TRUE(out.command.x==0||out.command.x>=.21);EXPECT_LE(out.command.x,options.walk_speed);
    if(out.command.x==0)EXPECT_TRUE(out.command.yaw==0||std::abs(out.command.yaw)>=.21);
    EXPECT_LE(std::abs(out.command.yaw),options.yaw_rate);
    translated|=out.command.x>0;rotated|=std::abs(out.command.yaw)>0;
    pose.ground=out.predicted;pose.yaw=out.predicted_yaw;pose.measured=out.command;done=out.done;
  }
  EXPECT_TRUE(done);EXPECT_TRUE(translated);EXPECT_TRUE(rotated);
}
TEST(Execution, HeadingNoiseAndRollingReplansKeepForwardMotionContinuous) {
  ExecutionOptions options;options.startup_linear_speed=.21;options.startup_yaw_rate=.21;
  PathTracker tracker(options);TrackingPose pose;pose.yaw=.04;
  const Points path{{0,0,0},{10,0,0}};
  tracker.setRoute(path,{false},pose,path.back(),0,false);
  TrackResult out;
  for(int i=0;i<20;++i)out=tracker.update(pose,.05);
  ASSERT_EQ(out.phase,"walk");ASSERT_GE(out.command.x,.21);
  ASSERT_GT(std::abs(out.command.yaw),0);ASSERT_LT(std::abs(out.command.yaw),.21);
  Velocity previous=out.command;
  for(int i=0;i<100;++i){
    // Cross the former stop/align threshold repeatedly, including replans.
    pose.yaw=(i%2?1.:-1.)*(i%3?.085:.075);
    if(i%4==0)tracker.setRoute(path,{false},pose,path.back(),0,false);
    out=tracker.update(pose,.05);
    ASSERT_FALSE(out.fault);EXPECT_EQ(out.phase,"walk");EXPECT_GE(out.command.x,.21);
    EXPECT_LE(std::abs(out.command.x-previous.x),options.acceleration*.05+1e-9);
    EXPECT_LE(std::abs(out.command.yaw-previous.yaw),options.yaw_acceleration*.05+1e-9);
    pose.measured=out.command;previous=out.command;
  }
  // A large heading error still requires stopping before realignment.
  pose.yaw=1.2;out=tracker.update(pose,.05);EXPECT_EQ(out.phase,"align");
}
TEST(Execution, DenseGentleTerrainEdgesDoNotBecomeStopPoints) {
  ExecutionOptions options;options.startup_linear_speed=.21;options.startup_yaw_rate=.21;
  Points path;
  for(int i=0;i<=30;++i)path.emplace_back(i*.1,.012*std::sin(i*.5),.004*std::sin(i*.4));
  PathTracker tracker(options);TrackingPose pose;pose.ground=path.front();
  tracker.setRoute(path,std::vector<bool>(path.size()-1,false),pose,path.back(),0,false);
  bool done=false;size_t walks=0,stops=0;double max_error=0;Velocity previous;
  for(int i=0;i<2000&&!done;++i){
    auto out=tracker.update(pose,.05);ASSERT_FALSE(out.fault)<<out.reason;
    if(out.phase=="walk"){
      ++walks;EXPECT_GE(out.command.x,.21);
      if(previous.x>0)EXPECT_LE(std::abs(out.command.x-previous.x),options.acceleration*.05+1e-9);
    }
    if(out.phase=="waypoint_stop")++stops;
    max_error=std::max(max_error,out.error);previous=out.command;
    pose.ground=out.predicted;pose.yaw=out.predicted_yaw;pose.measured=out.command;done=out.done;
  }
  EXPECT_TRUE(done);EXPECT_GT(walks,30u);EXPECT_EQ(stops,0u);EXPECT_LT(max_error,.04);
}
TEST(Execution, EstablishedRotationSurvivesGaitVelocityNoise) {
  ExecutionOptions options;options.startup_linear_speed=.21;options.startup_yaw_rate=.21;
  PathTracker tracker(options);TrackingPose pose;pose.yaw=1.;
  tracker.setRoute({{0,0,0},{3,0,0}},{false},pose,{3,0,0},0,false);
  TrackResult out;
  for(int i=0;i<20;++i)out=tracker.update(pose,.05);
  ASSERT_EQ(out.phase,"align");ASSERT_LT(out.command.yaw,-.2);
  for(int i=0;i<60;++i){
    pose.measured={.03,(i%2?.06:-.06),out.command.yaw};
    out=tracker.update(pose,.05);
    EXPECT_EQ(out.phase,"align");EXPECT_EQ(out.command.x,0);EXPECT_LT(out.command.yaw,-.2);
  }
}
TEST(Execution, CustomAutonomousLimitsReachGuardWithoutHiddenClamp) {
  SafetyInput i;i.robot_ready=i.feedback_fresh=i.localization_ready=i.route_ready=i.auto_enabled=true;
  i.command_deadline=1;i.task_id="profile";i.autonomous_limits={.6,.15,.8};i.autonomous={.55,0,.7};
  SafetyArbiter arbiter;auto out=arbiter.update(i,0);
  EXPECT_DOUBLE_EQ(out.command.x,.55);EXPECT_DOUBLE_EQ(out.command.yaw,.7);
  i.autonomous={.9,0,1.};out=arbiter.update(i,0);
  EXPECT_DOUBLE_EQ(out.command.x,.6);EXPECT_DOUBLE_EQ(out.command.yaw,.8);
  i.route_ready=false;EXPECT_EQ(speed(arbiter.update(i,0).command),0);
  i.route_ready=true;i.autonomous_limits.x=0;EXPECT_EQ(speed(arbiter.update(i,0).command),0);
}
TEST(Execution, TimedProfilesPreserveEdgesAndBoundLinearAndAngularDynamics) {
  Points points{{0,0,0},{1,0,0},{1,1,.16},{1,2,.32}};ExecutionOptions options;
  auto samples=parameterizeTerrainPath(points,{false,true,true},options);
  double last=-1;bool rotation=false,stairs=false;
  for(const auto &p:samples) {
    EXPECT_GE(p.time,last);last=p.time;
    EXPECT_LE(p.speed,options.walk_speed+1e-9);EXPECT_GE(p.speed,-1e-9);
    EXPECT_LE(std::abs(p.acceleration),options.acceleration+1e-9);
    EXPECT_LE(std::abs(p.yaw_rate),options.yaw_rate+1e-9);
    EXPECT_LE(std::abs(p.yaw_acceleration),options.yaw_acceleration+1e-9);
    if(p.phase=="align"){rotation=true;EXPECT_EQ(p.speed,0);EXPECT_NEAR(p.point.x(),1,1e-9);EXPECT_NEAR(p.point.y(),0,1e-9);}
    if(p.phase=="stairs_up"){stairs=true;EXPECT_LE(p.speed,options.stair_speed+1e-9);}
  }
  EXPECT_TRUE(rotation);EXPECT_TRUE(stairs);EXPECT_NEAR((samples.back().point-points.back()).norm(),0,1e-9);
}

TEST(Execution, FeedbackClosedLoopStopsTurnsAndTraversesStairsForward) {
  Points path{{0,0,0},{1,0,0},{1,1,.5},{0,1,.5}};std::vector<bool> steps{false,true,false};
  ExecutionOptions options;PathTracker tracker(options);TrackingPose pose;
  tracker.setRoute(path,steps,pose,path.back(),3.141592653589793,true);
  bool done=false,saw_stair=false,saw_align=false;Velocity previous;
  for(int i=0;i<40000&&!done;++i) {
    auto out=tracker.update(pose,.05);ASSERT_FALSE(out.fault)<<out.reason;
    EXPECT_LE(std::abs(out.command.x-previous.x),options.acceleration*.05+1e-9);
    EXPECT_LE(std::abs(out.command.yaw-previous.yaw),options.yaw_acceleration*.05+1e-9);
    previous=out.command;
    if(out.phase=="stairs_up"){saw_stair=true;EXPECT_GE(out.command.x,0);EXPECT_EQ(out.command.y,0);EXPECT_LE(out.command.x,options.stair_speed+1e-9);}
    if(out.phase=="align"||out.phase=="stair_align")saw_align=true;
    pose.ground=out.predicted;pose.yaw=out.predicted_yaw;pose.measured=out.command;done=out.done;
  }
  EXPECT_TRUE(done);EXPECT_TRUE(saw_stair);EXPECT_TRUE(saw_align);
  EXPECT_LE((pose.ground-path.back()).head<2>().norm(),options.goal_xy);
  EXPECT_LE(std::abs(pose.ground.z()-.5),options.goal_z);EXPECT_LT(speed(pose.measured),options.stopped_speed);
}

TEST(Execution, WrongFloorAndTrackingDeviationCannotReportSuccess) {
  PathTracker tracker;TrackingPose pose;pose.ground={0,0,2.4};
  tracker.setRoute({{0,0,0},{1,0,0}},{false},pose,{0,0,0},0,false);
  for(int i=0;i<20;++i) {
    const auto out=tracker.update(pose,.05);EXPECT_FALSE(out.done);
    if(out.fault){EXPECT_EQ(out.reason,"terrain_tracking_error");return;}
  }
  FAIL()<<"Wrong floor was not rejected";
}

TEST(Execution, ArbiterLeasesManualTakeoverAndEstopAreIndependent) {
  SafetyArbiter arbiter;SafetyInput i;i.robot_ready=i.feedback_fresh=i.auto_enabled=i.localization_ready=i.route_ready=true;
  i.command_deadline=1;i.task_id="task-a";i.autonomous.x=.1;
  EXPECT_EQ(arbiter.update(i,0).owner,"autonomous");
  i.localization_ready=false;EXPECT_EQ(speed(arbiter.update(i,.1).command),0);i.localization_ready=true;
  EXPECT_EQ(speed(arbiter.update(i,2).command),0);
  i.command_deadline=3;arbiter.manualTakeover(i.task_id);i.manual_deadline=2;i.manual.x=.05;
  EXPECT_EQ(arbiter.update(i,1).owner,"manual");
  EXPECT_EQ(speed(arbiter.update(i,2.1).command),0);
  i.task_id="task-b";EXPECT_EQ(arbiter.update(i,2.1).owner,"autonomous");
  i.estop=true;EXPECT_EQ(speed(arbiter.update(i,2.2).command),0);
}

TEST(Execution, TaskRecoveryIsBoundedAndResumeRequiresNewPlan) {
  TaskSupervisor task;task.start(0);EXPECT_TRUE(task.active());
  EXPECT_FALSE(task.update(1,false,false,true,false));EXPECT_EQ(task.state(),"relocalizing");
  EXPECT_TRUE(task.retryDue(1));EXPECT_FALSE(task.retryDue(2));
  EXPECT_TRUE(task.update(3,true,true,true,true));task.pause();EXPECT_TRUE(task.paused());
  task.resume(4);EXPECT_EQ(task.state(),"waiting_inputs");
  EXPECT_FALSE(task.update(5,true,false,true,false));
  EXPECT_FALSE(task.update(26,true,false,true,false));EXPECT_EQ(task.state(),"aborted");
}
TEST(Execution, BriefRouteLeaseGapsDoNotResetAnAlreadyRunningPlanner) {
  TaskSupervisor task;task.start(0);
  ASSERT_TRUE(task.update(1,true,true,true,false));
  ASSERT_FALSE(task.update(10,true,false,true,false));
  EXPECT_FALSE(task.retryDue(10.05));
  ASSERT_TRUE(task.update(10.2,true,true,true,false));
  ASSERT_FALSE(task.update(15,true,false,true,false));
  EXPECT_FALSE(task.retryDue(15.05));
  EXPECT_TRUE(task.retryDue(19.01));
}

TEST(Execution, DecisionConfigurationControlsRecoveryAndProgress) {
  DecisionOptions options;
  options.recovery_timeout = 2.; options.progress_timeout = 5.;
  options.retry_interval = 1.; options.max_retries = 1;
  TaskSupervisor task(options); task.start(0);
  EXPECT_TRUE(task.retryDue(0)); EXPECT_FALSE(task.retryDue(2));
  EXPECT_FALSE(task.update(0, false, false, true, false));
  EXPECT_FALSE(task.update(2.1, false, false, true, false));
  EXPECT_EQ(task.reason(), "input_or_recovery_timeout");
  task.start(0);
  EXPECT_FALSE(task.update(5.1, true, true, true, false));
  EXPECT_EQ(task.reason(), "no_measured_progress");
  options.max_retries = 0;
  TaskSupervisor no_retry(options); no_retry.start(0);
  EXPECT_FALSE(no_retry.retryDue(100));
  options.recovery_timeout = 0;
  EXPECT_THROW(TaskSupervisor invalid(options), std::invalid_argument);
  options.recovery_timeout = 2.; options.max_retries = -1;
  EXPECT_THROW(TaskSupervisor invalid(options), std::invalid_argument);
}

// Optional reproducible real-map kinematic audit. Pose feedback follows a simple
// ideal velocity plant on the supplied ground path; this is not dog dynamics.
TEST(Execution, RecordedRouteKinematicAudit) {
  const char *file=std::getenv("DOG_EXECUTION_ROUTE");if(!file)GTEST_SKIP();
  std::ifstream input(file);const auto document=nlohmann::json::parse(input);
  const auto route=document.contains("route")?document["route"]:document;
  Points path;for(const auto &p:route["points"])path.emplace_back(p[0].get<double>(),p[1].get<double>(),p[2].get<double>());
  std::vector<bool> stairs(path.size()-1,false);
  for(const auto &section:route["sections"])if(section["mode"]=="stairs")
    for(size_t i=section["start_index"];i<section["end_index"].get<size_t>();++i)stairs[i]=true;
  PathTracker tracker;TrackingPose pose;pose.ground=path.front();pose.yaw=std::atan2(path[1].y()-path[0].y(),path[1].x()-path[0].x());
  tracker.setRoute(path,stairs,pose,path.back(),0,false);
  bool done=false;double max_error=0,elapsed=0;size_t stair_samples=0;
  nlohmann::json phases=nlohmann::json::object();
  std::shared_ptr<dog_terrain::TerrainGraph> graph;
  if(const char *root=std::getenv("DOG_EXECUTION_MAP_ROOT"))
    graph=dog_terrain::loadProject(root,std::getenv("DOG_EXECUTION_MAP_ID"));
  Point checked=pose.ground;double checked_yaw=pose.yaw;size_t geometry_checks=0,geometry_rejections=0;
  nlohmann::json rejected=nlohmann::json::array();
  nlohmann::json nominal_rejected=nlohmann::json::array();
  if(graph)for(size_t i=1;i<path.size();++i) {
    const auto d=path[i]-path[i-1];const double heading=std::atan2(d.y(),d.x());
    if(!graph->motionClear(path[i-1],path[i],heading,heading))nominal_rejected.push_back(i-1);
  }
  for(int i=0;i<120000&&!done;++i) {
    const auto out=tracker.update(pose,.05);ASSERT_FALSE(out.fault)<<out.reason<<" at "<<pose.ground.transpose();
    max_error=std::max(max_error,out.error);if(out.phase.find("stairs_")==0)++stair_samples;
    phases[out.phase]=phases.value(out.phase,0.)+.05;
    if(graph&&((out.predicted-checked).norm()>.05||std::abs(angleError(out.predicted_yaw,checked_yaw))>.03||out.done)) {
      ++geometry_checks;
      if(!graph->motionClear(checked,out.predicted,checked_yaw,out.predicted_yaw)) {
        ++geometry_rejections;
        if(rejected.size()<30)rejected.push_back({{"position",{checked.x(),checked.y(),checked.z()}},
          {"next",{out.predicted.x(),out.predicted.y(),out.predicted.z()}},
          {"yaw",checked_yaw},{"next_yaw",out.predicted_yaw},{"phase",out.phase}});
      }
      checked=out.predicted;checked_yaw=out.predicted_yaw;
    }
    pose.ground=out.predicted;pose.yaw=out.predicted_yaw;pose.measured=out.command;done=out.done;elapsed+=.05;
  }
  EXPECT_TRUE(done);EXPECT_GT(stair_samples,0u);
  nlohmann::json result={{"kinematic_goal_reached",done},{"navigation_feasible_in_model",done&&graph!=nullptr&&geometry_rejections==0},
      {"model","ideal_velocity_feedback_on_recorded_terrain"},
      {"virtual_seconds",elapsed},{"max_tracking_error",max_error},{"stair_control_samples",stair_samples},
      {"final_position",{pose.ground.x(),pose.ground.y(),pose.ground.z()}},{"physical_robot_verified",false},
      {"phase_seconds",phases},{"geometry_checks",geometry_checks},{"geometry_rejections",geometry_rejections},
      {"geometry_verified",graph!=nullptr&&geometry_rejections==0},{"rejected_sweeps",rejected},
      {"nominal_rejected_edges",nominal_rejected}};
  if(const char *out=std::getenv("DOG_EXECUTION_REPORT")){std::ofstream report(out);report<<result.dump(2)<<'\n';}
}
