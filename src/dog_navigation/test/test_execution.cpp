#include "dog_navigation/path_tracker.hpp"
#include "dog_navigation/timed_trajectory.hpp"
#include "dog_navigation/task_supervisor.hpp"
#include "dog_navigation/safety_arbiter.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include "dog_terrain/terrain_graph.hpp"

using namespace dog_navigation;
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
