#include <gtest/gtest.h>
#include "robot_bridge.hpp"
#include <atomic>
#include <functional>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using Json = nlohmann::json;
using namespace std::chrono_literals;

class RobotBridgeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    // Isolate all command publishers from a robot's ROS graph.
    setenv("ROS_DOMAIN_ID", "197", 1);
    setenv("ROS_LOCALHOST_ONLY", "1", 1);
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    bridge = std::make_shared<RobotBridge>("/tmp/dog-bridge-test", false);
    mock = std::make_shared<rclcpp::Node>("mock_robot");
    status = mock->create_publisher<dog_interfaces::msg::RobotStatus>("/dog/status", 10);
    command = mock->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m) { velocity.store(m->linear.x); ++received; });
    executor.add_node(mock); executor.add_node(bridge);
    spin = std::thread([this] {executor.spin();});
  }
  void TearDown() override { executor.cancel(); spin.join(); bridge.reset(); mock.reset(); }
  bool until(const std::function<bool()> &condition, bool send_status = true) {
    for (int i = 0; i < 200; ++i) {
      if (send_status) {
        dog_interfaces::msg::RobotStatus m;
        m.header.stamp = mock->now(); m.connected = true; m.state_fresh = true;
        m.control_level = m.CONTROL_SAFE_TELEOP; status->publish(m);
      }
      if (condition()) return true;
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<RobotBridge> bridge;
  rclcpp::Node::SharedPtr mock;
  rclcpp::Publisher<dog_interfaces::msg::RobotStatus>::SharedPtr status;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command;
  std::thread spin;
  std::atomic<double> velocity{0};
  std::atomic<int> received{0};
};

TEST_F(RobotBridgeTest, ModelTelemetryUsesFreshNamedJointsAndBodyOrientation) {
  // Model inputs must expire independently of the robot control heartbeat.
  auto joints=mock->create_publisher<sensor_msgs::msg::JointState>("/joint_states",rclcpp::SensorDataQoS());
  auto odom=mock->create_publisher<nav_msgs::msg::Odometry>("/dog/odom",rclcpp::SensorDataQoS());
  auto imu=mock->create_publisher<sensor_msgs::msg::Imu>("/imu/data",rclcpp::SensorDataQoS());
  ASSERT_TRUE(until([&]{
    sensor_msgs::msg::JointState j;j.header.stamp=mock->now();j.name={"fl_knee_joint"};j.position={-.8};joints->publish(j);
    nav_msgs::msg::Odometry o;o.header.stamp=mock->now();o.pose.pose.orientation.w=1;odom->publish(o);
    return !bridge->telemetry()["joints"].is_null();
  }));
  auto state=bridge->telemetry();
  EXPECT_EQ(state["joints"]["names"][0],"fl_knee_joint");
  EXPECT_DOUBLE_EQ(state["joints"]["positions"][0].get<double>(),-.8);
  ASSERT_TRUE(until([&]{return !bridge->telemetry()["body_orientation"].is_null();}));
  EXPECT_EQ(bridge->telemetry()["body_orientation"],Json::array({0.,0.,0.,1.}));
  ASSERT_TRUE(until([&]{
    sensor_msgs::msg::Imu m;m.header.stamp=mock->now();m.orientation.z=.1;m.orientation.w=std::sqrt(.99);imu->publish(m);
    const auto q=bridge->modelTelemetry()["body_orientation"];return !q.is_null()&&q[2]==.1;
  }));
  EXPECT_EQ(bridge->modelTelemetry()["joints"]["positions"][0],-.8);
  std::this_thread::sleep_for(1100ms);
  EXPECT_TRUE(bridge->telemetry()["joints"].is_null());
  EXPECT_TRUE(bridge->telemetry()["body_orientation"].is_null());
  EXPECT_TRUE(bridge->modelTelemetry()["joints"].is_null());
}
TEST_F(RobotBridgeTest, ExecutionPathsRequireMatchingMapStampAndLiveLease) {
  auto state=mock->create_publisher<std_msgs::msg::String>("/navigation_executor/status",rclcpp::QoS(1).transient_local());
  auto planner=mock->create_publisher<std_msgs::msg::String>("/local_navigation/status",rclcpp::QoS(1).transient_local());
  auto global=mock->create_publisher<nav_msgs::msg::Path>("/local_navigation/global_path",1);
  auto local=mock->create_publisher<nav_msgs::msg::Path>("/local_navigation/local_path",1);
  ASSERT_TRUE(until([&]{return state->get_subscription_count()&&planner->get_subscription_count()&&global->get_subscription_count()&&local->get_subscription_count();}));
  auto publish=[&](const std::string &revision,int goal=10){
    nav_msgs::msg::Path path;path.header.frame_id="map";path.header.stamp=mock->now();
    geometry_msgs::msg::PoseStamped p;p.pose.position.z=.12;path.poses={p,p,p};global->publish(path);
    path.poses.resize(2);local->publish(path);
    state->publish(std_msgs::msg::String().set__data(Json({{"task_id","task-a"},{"map_project","/maps/scene"},{"map_revision","r"},
      {"planner_goal_stamp",{{"sec",10},{"nanosec",0}}}}).dump()));
    planner->publish(std_msgs::msg::String().set__data(Json({{"map_project","/maps/scene"},{"map_revision",revision},
      {"valid",true},{"state","route_ready"},{"valid_for_seconds",.5},
      {"goal_stamp",{{"sec",goal},{"nanosec",0}}},
      {"route_stamp",{{"sec",path.header.stamp.sec},{"nanosec",path.header.stamp.nanosec}}}}).dump()));
  };
  // Separate DDS topics may advance between snapshots. Inspect the exact
  // coherent snapshot that satisfied the predicate, not the next refresh.
  Json paths;
  ASSERT_TRUE(until([&]{publish("r");paths=bridge->navigationPaths();return paths["valid"].get<bool>();}));
  EXPECT_EQ(paths["global_path"].size(),3u);EXPECT_EQ(paths["local_path"].size(),2u);
  EXPECT_DOUBLE_EQ(paths["local_path"][0][2].get<double>(),.12);
  ASSERT_TRUE(until([&]{publish("wrong");return bridge->navigationPaths()["reason"]=="map_mismatch";}));
  ASSERT_TRUE(until([&]{publish("r",11);return bridge->navigationPaths()["reason"]=="goal_mismatch";}));
  ASSERT_TRUE(until([&]{publish("r");return bridge->navigationPaths()["valid"].get<bool>();}));
  std::this_thread::sleep_for(550ms);EXPECT_FALSE(bridge->navigationPaths()["valid"].get<bool>());
  EXPECT_TRUE(bridge->navigationPaths()["local_path"].empty());
}

TEST_F(RobotBridgeTest, NavigationRequiresFreshReadinessAndExactMapIdentity) {
  using Action=dog_interfaces::action::NavigateTerrain;
  auto publisher=mock->create_publisher<std_msgs::msg::String>("/navigation_executor/status",rclcpp::QoS(1).transient_local());
  std::atomic<int> accepted{0};
  auto server=rclcpp_action::create_server<Action>(mock,"/navigate_terrain",
    [](const rclcpp_action::GoalUUID &,std::shared_ptr<const Action::Goal>){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
    [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>){return rclcpp_action::CancelResponse::ACCEPT;},
    [&](std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>> h){
      if(h->get_goal()->goal.header.frame_id=="map"&&h->get_goal()->goal.pose.position.z==-2.)++accepted;
      auto result=std::make_shared<Action::Result>();result->success=true;h->succeed(result);
    });
  Json request={{"map_project","/tmp/scene"},{"revision","revision-a"},{"goal",{1.,2.,-2.}}};
  EXPECT_THROW(bridge->command("/api/navigation/start",request),std::invalid_argument);
  ASSERT_TRUE(until([&]{
    std_msgs::msg::String m;m.data=Json({{"enabled",true},{"guard_auto_enabled",true},
      {"localization_ready",true},{"map_project","/tmp/scene"},{"map_revision","revision-a"}}).dump();publisher->publish(m);
    return bridge->telemetry()["navigation_ready"].get<bool>();
  }));
  auto wrong=request;wrong["revision"]="revision-b";
  EXPECT_THROW(bridge->command("/api/navigation/start",wrong),std::invalid_argument);
  wrong=request;wrong["map_project"]="/tmp/other";
  EXPECT_THROW(bridge->command("/api/navigation/start",wrong),std::invalid_argument);
  auto probe=rclcpp_action::create_client<Action>(mock,"/navigate_terrain");
  ASSERT_TRUE(probe->wait_for_action_server(2s));
  auto response=bridge->command("/api/navigation/start",request);
  EXPECT_TRUE(response["success"].get<bool>());
  EXPECT_TRUE(until([&]{return accepted==1;}));
  std::this_thread::sleep_for(900ms);
  EXPECT_FALSE(bridge->telemetry()["navigation_ready"].get<bool>());
}

TEST_F(RobotBridgeTest, NavigationCancelStopsAndClearsLeaseEvenWithoutTaskServices){
  // Cancellation must stop an active Web stream even if ROS task services fail.
  ASSERT_TRUE(until([&]{return bridge->telemetry()["control_ready"].get<bool>();}));
  const auto token=bridge->command("/api/control/acquire",Json::object()).at("token");
  bridge->command("/api/control/velocity",{{"token",token},{"x",.2},{"y",0},{"yaw",0}});
  ASSERT_TRUE(until([&]{return velocity.load()>.1;}));
  const auto result=bridge->command("/api/navigation/cancel",Json::object());
  EXPECT_FALSE(result["success"].get<bool>());
  EXPECT_TRUE(until([&]{return velocity.load()==0.;}));
  EXPECT_FALSE(bridge->telemetry()["control_claimed"].get<bool>());
}

TEST_F(RobotBridgeTest, NavigationCancelRequestsGuardExecutorAndPlannerWithoutMapReadiness){
  std::atomic<int> count{0};
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services;
  for(const auto *path:{"/velocity_guard/cancel","/navigation_executor/cancel","/local_navigation/cancel"})
    services.push_back(mock->create_service<std_srvs::srv::Trigger>(path,[&](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> r){++count;r->success=true;}));
  bool success=false;
  ASSERT_TRUE(until([&]{success=bridge->command("/api/navigation/cancel",Json::object())["success"].get<bool>();return success;}));
  EXPECT_GE(count.load(),3);
  EXPECT_TRUE(bridge->command("/api/navigation/cancel",Json::object())["success"].get<bool>());
}

TEST_F(RobotBridgeTest, RequiresFreshRobotAndExclusiveLeaseAndStopsAfterTimeout) {
  EXPECT_THROW(bridge->command("/api/control/acquire", Json::object()), std::invalid_argument);
  ASSERT_TRUE(until([this] {return bridge->telemetry()["control_ready"] == true;}));
  auto lease = bridge->command("/api/control/acquire", Json::object());
  EXPECT_THROW(bridge->command("/api/control/acquire", Json::object()), std::invalid_argument);
  Json request = {{"token", lease["token"]}, {"x", .2}, {"y", 0}, {"yaw", 0}};
  bridge->command("/api/control/velocity", request);
  ASSERT_TRUE(until([this] {return velocity.load() > .19;}));
  request["x"] = 1;
  for (int i = 0; i < 5; ++i) {
    EXPECT_THROW(bridge->command("/api/control/velocity", request), std::invalid_argument);
    std::this_thread::sleep_for(90ms);
  }
  ASSERT_TRUE(until([this] {return received.load() > 1 && velocity.load() == 0;}));
  EXPECT_FALSE(bridge->telemetry()["control_claimed"].get<bool>());
  request["x"] = .2;
  EXPECT_THROW(bridge->command("/api/control/velocity", request), std::invalid_argument);
  EXPECT_FALSE(bridge->telemetry()["navigation_ready"].get<bool>());
  EXPECT_FALSE(bridge->telemetry()["localization_ready"].get<bool>());
}

TEST_F(RobotBridgeTest, SelectedSpeedLimitsApplyAndSavingRevokesActiveLease) {
  ASSERT_TRUE(until([this]{return bridge->telemetry()["control_ready"]==true;}));
  bridge->saveMotionSettings({{"x",.6},{"y",.3},{"yaw",1.}});
  auto lease=bridge->command("/api/control/acquire",Json::object());
  Json request={{"token",lease["token"]},{"x",.6},{"y",.3},{"yaw",1.}};
  bridge->command("/api/control/velocity",request);
  ASSERT_TRUE(until([this]{return velocity.load()>.59;}));
  bridge->saveMotionSettings({{"x",.2},{"y",.1},{"yaw",.2}});
  ASSERT_TRUE(until([this]{return velocity.load()==0;}));
  EXPECT_FALSE(bridge->telemetry()["control_claimed"].get<bool>());
  request["token"]=bridge->command("/api/control/acquire",Json::object())["token"];
  EXPECT_THROW(bridge->command("/api/control/velocity",request),std::invalid_argument);
}

TEST_F(RobotBridgeTest, ValidInputIsResampledWithoutRenewingItsLease) {
  ASSERT_TRUE(until([this]{return bridge->telemetry()["control_ready"]==true;}));
  auto lease=bridge->command("/api/control/acquire",Json::object());
  const Json request={{"token",lease["token"]},{"x",.2},{"y",0},{"yaw",0}};
  for(int i=0;i<3;++i){bridge->command("/api/control/velocity",request);std::this_thread::sleep_for(200ms);}
  EXPECT_GT(received.load(),20);EXPECT_GT(velocity.load(),.19);
  ASSERT_TRUE(until([this]{return velocity.load()==0;}));
  const auto timing=bridge->telemetry()["control_timing"];
  EXPECT_EQ(timing["received_commands"],3);EXPECT_GT(timing["published_commands"].get<int>(),20);
  EXPECT_EQ(timing["last_stop_reason"],"input_lease_expired");
  EXPECT_FALSE(bridge->telemetry()["control_claimed"].get<bool>());
}

TEST_F(RobotBridgeTest, PropagatesActualServiceRepliesAndRejectsUnsafeMotion) {
  auto save = mock->create_service<std_srvs::srv::Trigger>("/lio/save_pcd",
    [](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr response) {
      response->success = true; response->message = "snapshot saved";
    });
  auto motion = mock->create_service<dog_interfaces::srv::SetMotion>("/dog/set_motion",
    [](const dog_interfaces::srv::SetMotion::Request::SharedPtr request,
       dog_interfaces::srv::SetMotion::Response::SharedPtr response) {
      response->accepted = false; response->message = request->confirm_restricted_motion ? "SDK rejected" : "not confirmed";
    });
  ASSERT_TRUE(until([this] {auto t = bridge->telemetry(); return t["control_ready"] == true && t["mapping"]["save_available"] == true;}));
  EXPECT_EQ(bridge->command("/api/slam/save", Json::object())["message"], "snapshot saved");
  auto lease = bridge->command("/api/control/acquire", Json::object());
  Json req = {{"token", lease["token"]}, {"motion", 6}, {"confirm", true}};
  EXPECT_THROW(bridge->command("/api/control/motion", req), std::invalid_argument);
  req["motion"] = 0; req["confirm"] = false;
  EXPECT_THROW(bridge->command("/api/control/motion", req), std::invalid_argument);
  req["confirm"] = true;
  auto result = bridge->command("/api/control/motion", req);
  EXPECT_FALSE(result["success"].get<bool>()); EXPECT_EQ(result["message"], "SDK rejected");
  EXPECT_THROW(bridge->command("/api/slam/stop", Json::object()), std::invalid_argument);
  bridge->command("/api/control/stop", Json::object());
  EXPECT_FALSE(bridge->telemetry()["control_claimed"].get<bool>());
}

TEST_F(RobotBridgeTest, StaleMessageStampsDoNotBecomeHealthyOnReceipt) {
  dog_interfaces::msg::RobotStatus stale;
  stale.header.stamp = mock->now() - rclcpp::Duration::from_seconds(10);
  stale.connected = true; stale.state_fresh = true;
  ASSERT_TRUE(until([&] {
    status->publish(stale);
    return bridge->telemetry()["streams"].contains("control");
  }, false));
  EXPECT_FALSE(bridge->telemetry()["control_ready"].get<bool>());
  EXPECT_THROW(bridge->command("/api/control/acquire", Json::object()), std::invalid_argument);
}

TEST(CloudEncoding, PreservesIntensityAndRowPaddingAndBoundsPreviewSize) {
  sensor_msgs::msg::PointCloud2 m;m.header.frame_id="odom";m.width=3;m.height=2;m.point_step=16;m.row_step=64;m.data.resize(128);
  for(int i=0;i<4;++i){sensor_msgs::msg::PointField field;field.name=i==3?"intensity":std::string(1,"xyz"[i]);field.offset=i*4;field.datatype=7;field.count=1;m.fields.push_back(field);}
  for(size_t i=0;i<6;++i){float p[4]={float(i),1,2,float(i*10)};std::memcpy(m.data.data()+(i/3)*64+(i%3)*16,p,16);}
  auto full=CloudPreview::encode(m,100);ASSERT_EQ(full.bytes.size(),96);EXPECT_TRUE(full.intensity);EXPECT_EQ(full.frame,"odom");
  float last[4];std::memcpy(last,full.bytes.data()+80,16);EXPECT_EQ(last[0],5);EXPECT_EQ(last[3],50);
  EXPECT_LE(CloudPreview::encode(m,2).bytes.size(),32);
  m.data.resize(80);EXPECT_TRUE(CloudPreview::encode(m,100).bytes.empty());
  m.data.resize(128);m.fields[0].offset=15;EXPECT_TRUE(CloudPreview::encode(m,100).bytes.empty());
}

TEST_F(RobotBridgeTest, LargeCloudEncodingDoesNotHoldControlMutexAndKeepsRawScan) {
  auto global=mock->create_publisher<sensor_msgs::msg::PointCloud2>("/lio/global_map",rclcpp::SensorDataQoS().keep_last(1));
  auto raw=mock->create_publisher<sensor_msgs::msg::PointCloud2>("/interfaces/lidar",rclcpp::SensorDataQoS().keep_last(1));
  sensor_msgs::msg::PointCloud2 m;m.header.frame_id="odom";m.width=300000;m.height=1;m.point_step=16;m.row_step=m.width*16;m.data.resize(m.row_step);
  for(int i=0;i<4;++i){sensor_msgs::msg::PointField f;f.name=i==3?"intensity":std::string(1,"xyz"[i]);f.offset=i*4;f.datatype=7;f.count=1;m.fields.push_back(f);}
  ASSERT_TRUE(until([this]{return bridge->telemetry()["control_ready"]==true;}));
  auto lease=bridge->command("/api/control/acquire",Json::object());double maximum_ms=0;
  for(int i=0;i<20;++i){
    m.header.stamp=mock->now();global->publish(m);
    auto scan=m;scan.header.frame_id="livox_frame";scan.width=12000;scan.row_step=scan.width*16;scan.data.resize(scan.row_step);raw->publish(scan);
    dog_interfaces::msg::RobotStatus state;state.header.stamp=mock->now();state.connected=true;state.state_fresh=true;status->publish(state);
    const auto before=std::chrono::steady_clock::now();
    bridge->command("/api/control/velocity",{{"token",lease["token"]},{"x",.2},{"y",0},{"yaw",0}});
    maximum_ms=std::max(maximum_ms,std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-before).count());
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_LT(maximum_ms,100);EXPECT_GT(received.load(),10);
  ASSERT_TRUE(until([this]{return bool(bridge->cloudPacket(false))&&bool(bridge->cloudPacket(true));}));
  auto map=bridge->cloudPacket(false),scan=bridge->cloudPacket(true);
  EXPECT_EQ(map->bytes.size(),300000*16);EXPECT_EQ(scan->frame,"livox_frame");EXPECT_TRUE(scan->intensity);
  size_t tile_points=0;for(const auto &[id,tile]:map->tiles)tile_points+=tile.indices.size();
  EXPECT_EQ(tile_points,300000U);
  EXPECT_EQ(bridge->cloudMetadata(*map)["frame"],"odom");
}

TEST_F(RobotBridgeTest, InitialPoseUsesSelectedHeadingAndBodyHeightWithoutClaimingSuccess) {
  auto loc=mock->create_publisher<std_msgs::msg::String>("/map_localizer/status",rclcpp::QoS(1).transient_local());
  tf2_ros::TransformBroadcaster tf(mock);
  auto received_pose=std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  auto pose_mutex=std::make_shared<std::mutex>();auto seen=std::make_shared<std::atomic<bool>>(false);
  auto seed=mock->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose",10,
    [received_pose,pose_mutex,seen](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m){
      std::lock_guard<std::mutex> lock(*pose_mutex);*received_pose=*m;seen->store(true);
    });
  Json request={{"map_project","/tmp/loc-project"},{"ground_position",{1.,2.,3.}},{"yaw",1.},{"body_height",.65}};
  ASSERT_TRUE(until([&]{
    std_msgs::msg::String state;state.data=Json({{"map_path","/tmp/loc-project"},{"map_ready",true},{"valid",false}}).dump();loc->publish(state);
    geometry_msgs::msg::TransformStamped t;t.header.frame_id="odom";t.child_frame_id="base_link";t.header.stamp=mock->now();
    tf2::Quaternion q;q.setRPY(.1,.2,-.3);t.transform.rotation.x=q.x();t.transform.rotation.y=q.y();t.transform.rotation.z=q.z();t.transform.rotation.w=q.w();tf.sendTransform(t);
    try{bridge->command("/api/localization/initialize",request);}catch(const std::exception &){return false;}
    return seen->load();
  }));
  std::lock_guard<std::mutex> lock(*pose_mutex);
  tf2::Quaternion desired;desired.setRPY(.1,.2,1.);const auto offset=tf2::quatRotate(desired,tf2::Vector3(0,0,.65));
  EXPECT_NEAR(received_pose->pose.pose.position.x,1+offset.x(),1e-8);
  EXPECT_NEAR(received_pose->pose.pose.position.y,2+offset.y(),1e-8);
  EXPECT_NEAR(received_pose->pose.pose.position.z,3+offset.z(),1e-8);
  EXPECT_NEAR(received_pose->pose.pose.orientation.z,desired.z(),1e-8);
  EXPECT_NEAR(received_pose->pose.pose.orientation.w,desired.w(),1e-8);
  EXPECT_FALSE(bridge->telemetry()["localization_ready"].get<bool>());
  request["map_project"]="/tmp/wrong-project";
  EXPECT_THROW(bridge->command("/api/localization/initialize",request),std::invalid_argument);
  request["map_project"]="/tmp/loc-project";
  EXPECT_THROW(bridge->command("/api/localization/global_relocalize",request),std::invalid_argument);
}
