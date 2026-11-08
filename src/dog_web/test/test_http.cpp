#include <gtest/gtest.h>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <nlohmann/json.hpp>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>
#include <sstream>

namespace asio = boost::asio;
namespace http = boost::beast::http;
namespace fs = std::filesystem;
using Json = nlohmann::json;
using tcp = asio::ip::tcp;

class HttpApi : public ::testing::Test {
protected:
  void SetUp() override {
    root = fs::temp_directory_path() / ("dog-http-test-" + std::to_string(getpid()));
    fs::remove_all(root); fs::create_directory(root);
    fs::create_directories(root / "www/console");
    // Model ament --symlink-install: index files resolve to two source roots.
    fs::create_symlink(fs::path(PREVIEW_WEB_ROOT) / "index.html", root / "www/index.html");
    fs::create_symlink(fs::path(PREVIEW_CONSOLE_ROOT) / "index.html", root / "www/console/index.html");
    int channel[2]; ASSERT_EQ(pipe(channel), 0);
    pid = fork(); ASSERT_GE(pid, 0);
    if (pid == 0) {
      close(channel[0]); dup2(channel[1], STDOUT_FILENO); close(channel[1]);
      execl(PREVIEW_EXECUTABLE, PREVIEW_EXECUTABLE, "--port", "0", "--map-root", root.c_str(),
            "--slam-output",(root/"slam").c_str(),
            "--web-root", (root / "www").c_str(), static_cast<char *>(nullptr));
      _exit(127);
    }
    close(channel[1]);
    std::string line; char ch;
    while (read(channel[0], &ch, 1) == 1 && ch != '\n') line += ch;
    close(channel[0]);
    const std::string prefix = "http://127.0.0.1:";
    auto index = line.find(prefix);
    ASSERT_NE(index, std::string::npos) << line;
    port = std::stoi(line.substr(index + prefix.size()));
  }
  void TearDown() override {
    if (pid > 0) { kill(pid, SIGTERM); waitpid(pid, nullptr, 0); }
    fs::remove_all(root);
  }
  http::response<http::string_body> request(const std::string &path, const std::string &body = "",
      const std::string &origin = "", const std::string &kind = "application/json") {
    asio::io_context context; tcp::socket socket(context);
    socket.connect({asio::ip::make_address("127.0.0.1"), static_cast<unsigned short>(port)});
    http::request<http::string_body> req{body.empty() ? http::verb::get : http::verb::post, path, 11};
    req.set(http::field::host, "127.0.0.1:" + std::to_string(port));
    if (!origin.empty()) req.set(http::field::origin, origin);
    if (!body.empty()) { req.set(http::field::content_type, kind); req.body() = body; req.prepare_payload(); }
    http::write(socket, req);
    boost::beast::flat_buffer buffer;
    http::response_parser<http::string_body> parser; parser.body_limit(16 * 1024 * 1024);
    http::read(socket, buffer, parser); return parser.release();
  }
  Json finishJob(const http::response<http::string_body> &submitted) {
    EXPECT_EQ(submitted.result_int(), 202) << submitted.body();
    auto id = Json::parse(submitted.body()).at("job").get<std::string>();
    for (int i = 0; i < 200; ++i) {
      auto state = Json::parse(request("/api/jobs/" + id).body());
      if (state["state"] != "running") return state;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    throw std::runtime_error("Map job timed out");
  }
  pid_t pid = -1;
  int port = 0;
  fs::path root;
};

TEST_F(HttpApi, MapNamesDeletionAndLoadedMapProtection){
  ASSERT_EQ(finishJob(request("/api/demo",R"({"name":"library-a"})"))["state"],"complete");
  const auto original=Json::parse(request("/api/map").body());
  EXPECT_EQ(request("/api/maps/rename",R"({"id":"library-a","name":"一楼大厅"})").result_int(),200);
  const auto renamed=Json::parse(request("/api/map").body());EXPECT_EQ(renamed["map_id"],original["map_id"]);EXPECT_EQ(renamed["revision"],original["revision"]);EXPECT_EQ(renamed["display_name"],"一楼大厅");
  EXPECT_EQ(request("/api/maps/delete",R"({"id":"library-a","confirm":true})").result_int(),400);
  ASSERT_EQ(request("/api/maps/unload",R"({"id":"library-a"})").result_int(),200);
  EXPECT_EQ(request("/api/maps/delete",R"({"id":"library-a"})").result_int(),400);
  ASSERT_EQ(finishJob(request("/api/load",R"({"name":"library-a"})"))["state"],"complete");
  ASSERT_EQ(request("/api/maps/unload",R"({"id":"library-a"})").result_int(),200);
  EXPECT_EQ(request("/api/maps/delete",R"({"id":"library-a","confirm":true})").result_int(),200);
  EXPECT_TRUE(Json::parse(request("/api/maps").body())["maps"].empty());EXPECT_TRUE(fs::exists(root/".trash"));
  EXPECT_EQ(request("/api/maps/delete",R"({"id":"../outside","confirm":true})").result_int(),400);
}

TEST_F(HttpApi, NavigationMotionProfilePersistsAndRejectsDeadZoneAndPermissionChanges){
  auto response=Json::parse(request("/api/settings/navigation").body());
  auto values=response.at("saved");EXPECT_TRUE(response.at("active").is_null());
  values["walk_speed"]=.45;values["acceleration"]=.10;
  const auto saved=request("/api/settings/navigation",values.dump());ASSERT_EQ(saved.result_int(),200)<<saved.body();
  EXPECT_TRUE(Json::parse(saved.body()).at("restart_required"));
  EXPECT_DOUBLE_EQ(Json::parse(request("/api/settings/navigation").body())["saved"]["walk_speed"],.45);
  auto invalid=values;invalid["stair_speed"]=.08;
  EXPECT_EQ(request("/api/settings/navigation",invalid.dump()).result_int(),400);
  invalid=values;invalid["calibration_confirmed"]=true;
  EXPECT_EQ(request("/api/settings/navigation",invalid.dump()).result_int(),400);
  EXPECT_DOUBLE_EQ(Json::parse(request("/api/settings/navigation").body())["saved"]["walk_speed"],.45);
}

TEST_F(HttpApi, SavedPcdNamesAndDeletionKeepOtherSessions){
  for(const auto *session:{"one","two"}){fs::create_directories(root/"slam"/session/"time");std::ofstream(root/"slam"/session/"time/cloud.pcd")<<session;}
  EXPECT_EQ(request("/api/slam/pcds/rename",R"({"id":"one/time/cloud.pcd","name":"大厅"})").result_int(),200);
  EXPECT_EQ(request("/api/slam/pcds/rename",R"({"id":"two/time/cloud.pcd","name":"大厅"})").result_int(),400);
  EXPECT_EQ(request("/api/slam/pcds/delete",R"({"id":"one/time/cloud.pcd","confirm":true})").result_int(),200);
  const auto files=Json::parse(request("/api/slam/pcds").body())["files"];ASSERT_EQ(files.size(),1U);EXPECT_EQ(files[0]["id"],"two/time/cloud.pcd");
  EXPECT_TRUE(fs::exists(root/"slam/two/time/cloud.pcd"));
}

TEST_F(HttpApi, MapLifecycleUploadPlanningAndVersionChecks) {
  EXPECT_EQ(request("/").result_int(), 200);
  EXPECT_NE(request("/").body().find("机器人工作台"), std::string::npos);
  EXPECT_EQ(request("/console/runtime.js").result_int(), 200);
  EXPECT_EQ(request("/console/style.css").result_int(), 200);
  EXPECT_EQ(request("/index.html").result_int(), 200);
  EXPECT_EQ(request("/vendor/three.module.js").result_int(), 200);
  EXPECT_EQ(request("/vendor/URDFLoader.js").result_int(), 200);
  EXPECT_NE(request("/robot/urdf/bpx.urdf").body().find("fl_knee_joint"), std::string::npos);
  EXPECT_EQ(request("/robot/simply_mesh/torso.obj").result_int(), 200);
  EXPECT_EQ(request("/robot/%2e%2e/package.xml").result_int(), 404);
  EXPECT_EQ(request("/api/map").result_int(), 400);
  auto state = finishJob(request("/api/demo", R"({"name":"building"})"));
  ASSERT_EQ(state["state"], "complete") << state;
  auto map = Json::parse(request("/api/map").body());
  ASSERT_EQ(map["map_id"], "building"); EXPECT_GT(map["valid_surfaces"].get<int>(), 0);
  const auto packed=request("/api/map?encoding=float32");
  ASSERT_EQ(packed.result_int(),200);
  const auto descriptor=Json::parse(packed["X-Map-Metadata"]);
  EXPECT_EQ(descriptor["revision"],map["revision"]);
  EXPECT_EQ(descriptor["cloud_count"],map["cloud"].size());
  EXPECT_EQ(descriptor["surface_count"],map["surfaces"].size());
  EXPECT_EQ(packed.body().size(),(map["cloud"].size()*3+map["surfaces"].size()*4)*4);
  Json body = {{"map_id", "building"}, {"revision", map["revision"]},
               {"start", {-1, 0, 0}}, {"goal", {-1, 0, 2.4}}};
  auto planned = request("/api/plan", body.dump());
  ASSERT_EQ(planned.result_int(), 200) << planned.body();
  EXPECT_NEAR(Json::parse(planned.body())["ascent"].get<double>(), 2.4, 1e-5);
  EXPECT_EQ(Json::parse(planned.body())["collision_check"]["model"],"continuous_vertical_cylinder");
  const Json inspect={{"map_id","building"},{"revision",map["revision"]},{"point",{-1,0,0}}};
  const auto inspected=request("/api/inspect_surface",inspect.dump());
  ASSERT_EQ(inspected.result_int(),200);
  EXPECT_TRUE(Json::parse(inspected.body())["support_valid"].get<bool>());
  EXPECT_TRUE(Json::parse(inspected.body())["body_clear"].get<bool>());
  // Nearest supported nodes must not hide an unsupported actual ground point.
  auto actual=inspect;actual["yaw"]=0.;
  EXPECT_TRUE(Json::parse(request("/api/inspect_surface",actual.dump()).body())["requested_motion_clear"].get<bool>());
  actual["point"]={-1.,0.,-.25};
  EXPECT_FALSE(Json::parse(request("/api/inspect_surface",actual.dump()).body())["requested_motion_clear"].get<bool>());
  body["revision"] = "old";
  EXPECT_EQ(request("/api/plan", body.dump()).result_int(), 409);
  auto loaded = finishJob(request("/api/load", R"({"name":"building"})"));
  EXPECT_EQ(loaded["revision"], map["revision"]);
  std::ifstream cloud(root / "building/cloud.pcd", std::ios::binary);
  std::string bytes(std::istreambuf_iterator<char>(cloud), {});
  auto imported = finishJob(request("/api/import?name=imported", bytes, "", "application/octet-stream"));
  EXPECT_EQ(imported["state"], "complete") << imported;
  EXPECT_EQ(Json::parse(request("/api/map").body())["map_id"], "imported");
  auto duplicate = finishJob(request("/api/import?name=imported", bytes));
  EXPECT_EQ(duplicate["state"], "complete");
  EXPECT_EQ(duplicate["map_id"], "imported-2");
  auto bad = finishJob(request("/api/import?name=broken", "invalid pcd"));
  EXPECT_EQ(bad["state"], "failed");
  EXPECT_EQ(Json::parse(request("/api/map").body())["map_id"], "imported-2");
}

TEST_F(HttpApi, DifferentCloudsWithSameFilenameRemainDistinctAndReloadable) {
  auto floor_pcd = [](double offset) {
    std::ostringstream file;
    file << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
         << "WIDTH 1600\nHEIGHT 1\nPOINTS 1600\nDATA ascii\n";
    for (int x = 0; x < 40; ++x) for (int y = 0; y < 40; ++y)
      file << offset + .015 + x * .04 << ' ' << .015 + y * .04 << " 0\n";
    return file.str();
  };
  const std::string endpoint = "/api/import?name=full_map&filename=full_map.pcd";
  auto first = finishJob(request(endpoint, floor_pcd(0)));
  ASSERT_EQ(first["state"], "complete") << first;
  ASSERT_EQ(first["map_id"], "full_map");
  auto old_map = Json::parse(request("/api/map").body());
  EXPECT_EQ(old_map["source"]["filename"], "full_map.pcd");
  EXPECT_LT(old_map["bounds"][1][0].get<double>(), 2);
  auto second = finishJob(request(endpoint, floor_pcd(10)));
  ASSERT_EQ(second["state"], "complete") << second;
  EXPECT_EQ(second["map_id"], "full_map-2");
  auto new_map = Json::parse(request("/api/map").body());
  EXPECT_GT(new_map["bounds"][0][0].get<double>(), 10);
  EXPECT_NE(old_map["revision"], new_map["revision"]);
  EXPECT_NE(old_map["source"]["upload_sha256"], new_map["source"]["upload_sha256"]);
  EXPECT_NE(old_map["source"]["cloud_sha256"], new_map["source"]["cloud_sha256"]);
  auto third = finishJob(request(endpoint, floor_pcd(20)));
  EXPECT_EQ(third["map_id"], "full_map-3");
  auto reloaded = finishJob(request("/api/load", R"({"name":"full_map"})"));
  EXPECT_EQ(reloaded["revision"], old_map["revision"]);
  EXPECT_EQ(Json::parse(request("/api/map").body())["source"], old_map["source"]);
  EXPECT_LT(Json::parse(request("/api/map").body())["bounds"][1][0].get<double>(), 2);
}

TEST_F(HttpApi, RejectsInvalidEndpointsAndCrossOriginWrites) {
  EXPECT_EQ(Json::parse(request("/api/robot").body())["ros"], false);
  EXPECT_EQ(request("/api/control/stop", "{}").result_int(), 503);
  EXPECT_EQ(request("/api/slam/start", "{}", "https://untrusted.example").result_int(), 403);
  EXPECT_EQ(request("/api/demo", "{}", "https://untrusted.example").result_int(), 403);
  EXPECT_EQ(request("/api/load", R"({"name":"../escape"})").result_int(), 400);
  EXPECT_EQ(request("/api/demo", "{").result_int(), 400);
  EXPECT_EQ(request("/api/demo", "{}", "", "text/plain").result_int(), 400);
  EXPECT_EQ(request("/api/unknown").result_int(), 404);
  EXPECT_EQ(request("/%2e%2e/package.xml").result_int(), 404);
  EXPECT_EQ(request("/console/%2e%2e/AGENTS.md").result_int(), 404);
  EXPECT_TRUE(Json::parse(request("/api/maps").body())["maps"].empty());
}

TEST_F(HttpApi, MotionSettingsArePersistedAndValidated) {
  EXPECT_EQ(request("/api/settings/motion",R"({"x":0.8,"y":0.3,"yaw":0.9})").result_int(),200);
  auto saved=Json::parse(request("/api/settings/motion").body());
  EXPECT_EQ(saved["x"],.8);EXPECT_EQ(saved["maxima"]["y"],.3);
  std::ifstream input(root/".motion.json");
  EXPECT_EQ(Json::parse(input)["x"],.8);
  EXPECT_EQ(request("/api/settings/motion",R"({"x":1.1,"y":0.3,"yaw":0.9})").result_int(),400);
  EXPECT_EQ(request("/api/settings/motion",R"({"x":0.1,"y":0.1,"yaw":0.1})","https://untrusted.example").result_int(),403);
  EXPECT_EQ(Json::parse(request("/api/settings/motion").body())["x"],.8);
}

TEST_F(HttpApi, SavedPcdsAppearWithoutUploadAndSameNamesAreIndependent) {
  auto demo=finishJob(request("/api/demo",R"({"name":"source"})"));
  ASSERT_EQ(demo["state"],"complete");
  const auto saved=root/"slam/session-one/time/cloud.pcd";fs::create_directories(saved.parent_path());
  fs::copy_file(root/"source/cloud.pcd",saved);
  const auto history=saved.parent_path()/"localization_keyframes.csv";
  {std::ofstream out(history);out<<"id,timestamp,px,py,pz,qx,qy,qz,qw\n0,1,0,0,.65,0,0,0,1\n";}
  auto catalog=Json::parse(request("/api/slam/pcds").body());
  ASSERT_EQ(catalog["files"].size(),1);EXPECT_EQ(catalog["files"][0]["id"],"session-one/time/cloud.pcd");
  const Json body={{"id","session-one/time/cloud.pcd"},{"name","saved"}};
  EXPECT_EQ(finishJob(request("/api/import_saved",body.dump()))["map_id"],"saved");
  EXPECT_EQ(finishJob(request("/api/import_saved",body.dump()))["map_id"],"saved-2");
  EXPECT_TRUE(fs::is_regular_file(root/"saved/localization_keyframes.csv"));
  EXPECT_TRUE(fs::is_regular_file(root/"saved-2/localization_keyframes.csv"));
  {std::ofstream out(history);out<<"id,timestamp,px,py,pz,qx,qy,qz,qw\n0,1,4,5,6,0,0,0,1\n";}
  EXPECT_EQ(finishJob(request("/api/import_saved",body.dump()))["map_id"],"saved-3");
  {
    std::ifstream first(root/"saved/map.json"),third(root/"saved-3/map.json");
    EXPECT_NE(Json::parse(first)["files"]["localization_keyframes.csv"],
              Json::parse(third)["files"]["localization_keyframes.csv"]);
  }
  EXPECT_NE(fs::file_size(root/"saved/localization_keyframes.csv"),0u);
  EXPECT_EQ(finishJob(request("/api/load",R"({"name":"saved"})"))["state"],"complete");
  // Legacy raw keyframes are accepted only for the original cloud.
  fs::rename(history,saved.parent_path()/"20260101_raw_keyframes.csv");
  EXPECT_EQ(finishJob(request("/api/import_saved",body.dump()))["map_id"],"saved-4");
  EXPECT_TRUE(fs::is_regular_file(root/"saved-4/localization_keyframes.csv"));
  fs::copy_file(saved,saved.parent_path()/"cloud_optimized.pcd");
  EXPECT_EQ(Json::parse(request("/api/slam/pcds").body())["files"].size(),2u);
  fs::copy_file(saved.parent_path()/"20260101_raw_keyframes.csv",saved.parent_path()/"localization_optimized_keyframes.csv");
  EXPECT_EQ(finishJob(request("/api/import_saved",R"({"id":"session-one/time/cloud_optimized.pcd","name":"optimized"})"))["map_id"],"optimized");
  {
    std::ifstream metadata(root/"optimized/map.json");
    EXPECT_EQ(Json::parse(metadata)["source"]["keyframe_source"],"localization_optimized_keyframes.csv");
  }
  fs::copy_file(saved,saved.parent_path()/"cloud_static.pcd");
  fs::copy_file(saved,saved.parent_path()/"localization.pcd");
  fs::copy_file(saved.parent_path()/"20260101_raw_keyframes.csv",history);
  EXPECT_EQ(Json::parse(request("/api/slam/pcds").body())["files"].size(),3u);
  const std::string static_body=R"({"id":"session-one/time/cloud_static.pcd","name":"filtered"})";
  EXPECT_EQ(finishJob(request("/api/import_saved",static_body))["map_id"],"filtered");
  EXPECT_EQ(finishJob(request("/api/import_saved",static_body))["map_id"],"filtered-2");
  EXPECT_TRUE(fs::is_regular_file(root/"filtered/localization_keyframes.csv"));
  EXPECT_EQ(fs::file_size(root/"filtered/localization.pcd"),fs::file_size(saved));
  EXPECT_EQ(finishJob(request("/api/load",R"({"name":"filtered"})"))["state"],"complete");
  EXPECT_EQ(request("/api/import_saved",R"({"id":"../source/cloud.pcd"})").result_int(),400);
  fs::create_symlink(root/"source/cloud.pcd",saved.parent_path()/"full_map.pcd");
  EXPECT_EQ(request("/api/import_saved",R"({"id":"session-one/time/full_map.pcd"})").result_int(),400);
  EXPECT_EQ(request("/api/cloud/map").result_int(),204);
  EXPECT_EQ(request("/api/cloud/scan").result_int(),204);
}

TEST_F(HttpApi, ShutdownDoesNotWaitForIdleBrowserPreconnection) {
  asio::io_context context;
  tcp::socket idle(context);
  idle.connect({asio::ip::make_address("127.0.0.1"), static_cast<unsigned short>(port)});
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  kill(pid, SIGTERM);
  bool exited = false;
  for (int i = 0; i < 100; ++i) {
    if (waitpid(pid, nullptr, WNOHANG) == pid) {exited = true; pid = -1; break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(exited) << "Idle HTTP read prevented graceful shutdown";
}
