#include "dog_terrain/terrain_graph.hpp"
#include "robot_bridge.hpp"
#include "pcd_catalog.hpp"
#include "control_socket.hpp"
#include "map_library.hpp"
#include "navigation_start.hpp"
#include "resource_monitor.hpp"
#include "dog_navigation/navigation_motion_settings.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <chrono>
#include <atomic>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

namespace fs = std::filesystem;
namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;
using dog_terrain::Json;
using Graph = dog_terrain::TerrainGraph;

namespace {
std::string token() {
  static std::atomic<unsigned long> counter{0};
  return std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "-" + std::to_string(counter++);
}
fs::path expandHome(const std::string &path) {
  if (path.rfind("~/", 0) == 0) {
    const char *home = std::getenv("HOME");
    if (!home) throw std::invalid_argument("HOME is not set; specify an absolute map root");
    return fs::path(home) / path.substr(2);
  }
  return path;
}
std::string decode(const std::string &s) {
  std::string result;
  for (size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '%') {
      if (i + 2 >= s.size() || !std::isxdigit(s[i + 1]) || !std::isxdigit(s[i + 2]))
        throw std::invalid_argument("Invalid URL escape");
      result += static_cast<char>(std::stoi(s.substr(i + 1, 2), nullptr, 16)); i += 2;
    } else result += s[i] == '+' ? ' ' : s[i];
  }
  return result;
}
std::map<std::string, std::string> query(const std::string &target) {
  std::map<std::string, std::string> values;
  auto question = target.find('?');
  if (question == std::string::npos) return values;
  std::istringstream input(target.substr(question + 1));
  std::string item;
  while (std::getline(input, item, '&')) {
    auto eq = item.find('=');
    values[decode(item.substr(0, eq))] = eq == std::string::npos ? "" : decode(item.substr(eq + 1));
  }
  return values;
}
Eigen::Vector3d point(const Json &value) {
  if (!value.is_array() || value.size() != 3) throw std::invalid_argument("Endpoint must be [x,y,z]");
  for (const auto &v : value) if (!v.is_number()) throw std::invalid_argument("Endpoint must be numeric");
  return {value[0].get<double>(), value[1].get<double>(), value[2].get<double>()};
}

class Service {
public:
  Service(fs::path root, fs::path web, std::shared_ptr<RobotBridge> robot = nullptr,
      std::shared_ptr<MotionSettings> settings = std::make_shared<MotionSettings>(),fs::path output={})
      // ament symlink-installs the two index files from distinct source roots.
      // Resolve each index separately before enforcing static-file containment.
      : root_(fs::absolute(root)), web_(fs::weakly_canonical(web / "index.html").parent_path()),
        console_(fs::weakly_canonical(web / "console/index.html").parent_path()), robot_(std::move(robot)), settings_(std::move(settings)),
        catalog_(output.empty()?root_/".slam":std::move(output)),library_(root_,catalog_) {
    fs::create_directories(root_);
    if (!fs::exists(web_ / "vendor/three.module.js") || !fs::exists(web_ / "index.html"))
      throw std::invalid_argument("Web assets missing: run npm ci && npm run prepare-assets in src/dog_web/web before building");
    description_ = fs::weakly_canonical(fs::path(ament_index_cpp::get_package_share_directory("dog_description")) /
        "urdf/bpx.urdf").parent_path().parent_path();
  }
  ~Service() { if (worker_.valid()) worker_.wait(); }
  using Request = http::request<http::string_body>;
  using Response = http::response<http::string_body>;
  void controlStream(tcp::socket socket,Request request){
    if(!robot_)return;
    serveControlSocket(std::move(socket),std::move(request),
      [robot=robot_](const auto &path,const auto &body){return robot->command(path,body);},[]{return rclcpp::ok();});
  }

  Response dispatch(const Request &request) {
    try { return handle(request); }
    catch (const Json::exception &e) { return json(http::status::bad_request, {{"error", e.what()}}); }
    catch (const std::invalid_argument &e) { return json(http::status::bad_request, {{"error", e.what()}}); }
    catch (const fs::filesystem_error &e) { return json(http::status::bad_request, {{"error", e.what()}}); }
    catch (const std::exception &e) {
      std::cerr << "API error: " << e.what() << '\n';
      return json(http::status::internal_server_error, {{"error", "Internal server error; see server log"}});
    }
  }

private:
  Eigen::Vector3d checkedLiveStart(const Graph &graph,const Json &telemetry,const fs::path &project){
    const auto start=localizedPlanningStart(telemetry,project,graph.revision());
    const auto &q=telemetry.at("pose").at("quaternion");
    const Eigen::Quaterniond rotation(q[3].get<double>(),q[0].get<double>(),q[1].get<double>(),q[2].get<double>());
    const auto heading=rotation.normalized()*Eigen::Vector3d::UnitX();
    const double yaw=std::atan2(heading.y(),heading.x());
    if(!graph.motionClear(start,start,yaw,yaw))
      throw std::invalid_argument("机器人实际接地点未通过地图支撑/机身碰撞检查；请核对 base_ground_offset 与定位。规划吸附点不能替代实际起点");
    return start;
  }
  Json navigationSettings(const Json *request=nullptr){
    using Profile=dog_navigation::NavigationMotionSettings;
    std::lock_guard<std::mutex> lock(navigation_settings_mutex_);
    Profile profile(root_/".navigation-motion.json");
    const auto nav=robot_?robot_->telemetry().value("navigation",Json::object()):Json::object();
    const Json active=nav.value("fresh",false)?nav.value("motion_limits",Json(nullptr)):Json(nullptr);
    if(request)profile.save(*request);
    const auto saved=fs::exists(root_/".navigation-motion.json")?Profile::values(profile.load()):
      active.is_object()?active:Profile::values(dog_navigation::ExecutionOptions{});
    return {{"saved",saved},{"active",active},{"restart_required",active.is_null()||active!=saved}};
  }
  Response json(http::status status, const Json &value) {
    return response(status, value.dump(), "application/json; charset=utf-8");
  }
  Response response(http::status status, std::string body, const std::string &content_type) {
    Response result{status, 11};
    result.set(http::field::content_type, content_type); result.set(http::field::cache_control, "no-store");
    result.set("X-Content-Type-Options", "nosniff");
    result.body() = std::move(body); result.keep_alive(false); result.prepare_payload(); return result;
  }
  Json submit(std::function<std::pair<std::string, std::shared_ptr<Graph>>()> operation) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (job_.value("state", "") == "running") throw std::invalid_argument("A map operation is running; wait for it to finish");
    if (worker_.valid()) worker_.get();
    job_id_ = token(); job_ = {{"state", "running"}};
    worker_ = std::async(std::launch::async, [this, operation = std::move(operation)] {
      try {
        auto [name, graph] = operation();
        Json result = graph->summary(); result["state"] = "complete"; result["map_id"] = name;
        std::ifstream metadata_file(dog_terrain::mapPath(root_, name) / "map.json");
        const auto metadata = Json::parse(metadata_file);
        Json source = metadata.value("source", Json::object());
        source["cloud_sha256"] = metadata.at("files").at("cloud.pcd");
        result["source"] = source;
        std::lock_guard<std::mutex> lock(mutex_);
        graph_ = std::move(graph); map_id_ = std::move(name); source_ = std::move(source); job_ = std::move(result);
      } catch (const std::exception &e) {
        std::lock_guard<std::mutex> lock(mutex_); job_ = {{"state", "failed"}, {"error", e.what()}};
      }
    });
    return {{"job", job_id_}};
  }
  Response handle(const Request &request) {
    const std::string target(request.target()), path = target.substr(0, target.find('?'));
    if (request.method() == http::verb::get) {
      if(path=="/api/system/resources")return json(http::status::ok,resources_.snapshot());
      if(path=="/api/settings/navigation")return json(http::status::ok,navigationSettings());
      if(path=="/api/slam/pcds")return json(http::status::ok,library_.savedPcds());
      if(path=="/api/cloud/manifest"||path=="/api/cloud/tile"){
        const auto packet=robot_?robot_->cloudPacket(false):nullptr;
        if(!packet)return response(http::status::no_content,"","application/octet-stream");
        const auto params=query(target);
        auto metadata=robot_->cloudMetadata(*packet);metadata["epoch"]=packet->epoch;
        metadata["revision"]=packet->epoch+"-"+packet->tiles_version;
        if(path=="/api/cloud/manifest"){
          if(params.count("since")&&params.at("since")==metadata.at("revision").get<std::string>()){
            auto result=response(http::status::not_modified,"","application/json");
            result.set("X-Cloud-Metadata",metadata.dump());return result;
          }
          Json tiles=Json::array();
          for(const auto &[id,tile]:packet->tiles)tiles.push_back({{"id",id},{"version",tile.version},{"count",tile.indices.size()},{"center",tile.center}});
          metadata["tiles"]=std::move(tiles);return json(http::status::ok,metadata);
        }
        if(!params.count("id")||!params.count("version")||!params.count("epoch"))throw std::invalid_argument("Missing tile identity");
        const auto found=packet->tiles.find(params.at("id"));
        if(params.at("epoch")!=packet->epoch||found==packet->tiles.end()||found->second.version!=params.at("version"))
          return json(http::status::conflict,{{"error","Map tile changed; refresh manifest"}});
        const auto number=[&](const char *name,size_t fallback,size_t maximum){
          if(!params.count(name))return fallback;
          const auto &v=params.at(name);
          if(v.empty()||v.size()>7||v.find_first_not_of("0123456789")!=std::string::npos)throw std::invalid_argument("Invalid tile slice");
          const size_t n=std::stoul(v);if(n>maximum)throw std::invalid_argument("Invalid tile slice");return n;
        };
        const auto &tile=found->second;const auto offset=number("offset",0,tile.indices.size());
        const auto count=std::min(number("limit",512,512),tile.indices.size()-offset);
        if(!count)throw std::invalid_argument("Empty tile slice");
        auto result=response(http::status::ok,cloudTileBytes(packet->bytes,tile,offset,count),"application/octet-stream");
        result.set("X-Cloud-Tile",Json({{"epoch",packet->epoch},{"id",found->first},{"version",tile.version},{"offset",offset},{"count",count},{"total",tile.indices.size()}}).dump());return result;
      }
      if(path=="/api/cloud/map"||path=="/api/cloud/scan"){
        const auto packet=robot_?robot_->cloudPacket(path=="/api/cloud/scan"):nullptr;
        if(!packet)return response(http::status::no_content,"","application/octet-stream");
        const auto params=query(target);
        const size_t total=packet->bytes.size()/16;size_t limit=std::max(size_t(1),std::min(size_t(60000),total));
        if(params.count("limit")){
          const auto &value=params.at("limit");
          if(value.empty()||value.size()>6||value.find_first_not_of("0123456789")!=std::string::npos)throw std::invalid_argument("Invalid cloud point limit");
          limit=std::stoul(value);if(!limit||limit>60000)throw std::invalid_argument("Cloud point limit must be 1..60000");
        }
        const size_t stride=std::max(size_t(1),(total+limit-1)/limit);
        const auto key=std::to_string(packet->sequence)+"-"+std::to_string(limit);
        const bool unchanged=params.count("since")&&(params.at("since")==key||(!params.count("limit")&&params.at("since")==std::to_string(packet->sequence)));
        std::string bytes;
        if(!unchanged){
          if(stride==1)bytes=packet->bytes;
          else {bytes.reserve(((total+stride-1)/stride)*16);for(size_t i=0;i<total;i+=stride)bytes.append(packet->bytes.data()+i*16,16);}
        }
        auto result=response(unchanged?http::status::not_modified:http::status::ok,std::move(bytes),"application/octet-stream");
        auto metadata=robot_->cloudMetadata(*packet);metadata["full_count"]=total;metadata["count"]=(total+stride-1)/stride;metadata["cache_key"]=key;
        result.set("X-Cloud-Metadata",metadata.dump());return result;
      }
      if(path=="/api/settings/motion")return json(http::status::ok,settings_->snapshot());
      if (path == "/api/robot") return json(http::status::ok, robot_ ? robot_->telemetry() : Json{{"ros", false}});
      if (path == "/api/navigation/paths") return json(http::status::ok,robot_?robot_->navigationPaths():
          Json{{"valid",false},{"reason","ros_unavailable"}});
      if (path == "/api/robot/model") return json(http::status::ok,robot_?robot_->modelTelemetry():
        Json{{"joints",nullptr},{"body_orientation",nullptr},{"pose",nullptr},{"tf_ready",false}});
      if (path == "/api/robot/cloud") return json(http::status::ok,
        robot_ ? robot_->cloud() : Json{{"points", Json::array()}, {"sequence", 0}, {"frame", "map"}});
      if (path == "/api/maps") {
        std::lock_guard<std::mutex> lock(mutex_);return json(http::status::ok,library_.projects(map_id_));
      }
      if (path.rfind("/api/jobs/", 0) == 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (path.substr(10) != job_id_ || job_id_.empty()) throw std::invalid_argument("Unknown job");
        return json(http::status::ok, job_);
      }
      if (path == "/api/map") {
        std::shared_ptr<Graph> graph; std::string name; Json source;
        { std::lock_guard<std::mutex> lock(mutex_); graph = graph_; name = map_id_; source = source_; }
        if (!graph) throw std::invalid_argument("Load a map first");
        auto result = graph->summary(); result["map_id"] = name;
        result["display_name"]=library_.displayName("map",name);
        result["source"] = source;
        const auto params=query(target);
        if(params.count("encoding")&&params.at("encoding")=="float32") {
          std::string bytes;
          const auto &points=graph->cloud();const size_t stride=std::max(size_t(1),(points.size()+149999)/150000);
          const size_t cloud_count=(points.size()+stride-1)/stride;
          size_t surface_count=0;for(const auto &node:graph->nodes())surface_count+=node.valid;
          bytes.reserve((cloud_count*3+surface_count*4)*sizeof(float));
          const auto append=[&](float value){uint32_t bits;std::memcpy(&bits,&value,sizeof(bits));for(int i=0;i<4;++i)bytes.push_back(static_cast<char>((bits>>(8*i))&255));};
          for(size_t i=0;i<points.size();i+=stride){append(points[i].x);append(points[i].y);append(points[i].z);}
          for(const auto &node:graph->nodes())if(node.valid){append(node.point.x());append(node.point.y());append(node.point.z());append(node.component);}
          result["encoding"]="xyz-surface-f32le-v1";result["cloud_count"]=cloud_count;result["surface_count"]=surface_count;
          auto packed=response(http::status::ok,std::move(bytes),"application/octet-stream");
          packed.set("X-Map-Metadata",result.dump(-1,' ',true));return packed;
        }
        Json cloud = Json::array(), surfaces = Json::array();
        const auto &points = graph->cloud();
        size_t stride = std::max(size_t(1), (points.size() + 149999) / 150000);
        for (size_t i = 0; i < points.size(); i += stride) cloud.push_back({points[i].x, points[i].y, points[i].z});
        for (const auto &node : graph->nodes()) if (node.valid)
          surfaces.push_back({node.point.x(), node.point.y(), node.point.z(), node.component});
        result["cloud"] = std::move(cloud); result["surfaces"] = std::move(surfaces);
        return json(http::status::ok, result);
      }
      if (path.rfind("/api/", 0) == 0) return json(http::status::not_found, {{"error", "Unknown endpoint"}});
      const bool console = path.rfind("/console/", 0) == 0 || (path == "/" && fs::exists(console_ / "index.html"));
      const bool model = path.rfind("/robot/", 0) == 0;
      const auto &static_root = model ? description_ : console ? console_ : web_;
      const std::string asset = path == "/" ? "index.html" : decode(path.substr(model ? 7 : console ? 9 : 1));
      fs::path file = fs::weakly_canonical(static_root / asset);
      fs::path relative = file.lexically_relative(static_root);
      if (relative.empty() || *relative.begin() == ".." || !fs::is_regular_file(file))
        return response(http::status::not_found, "Not found", "text/plain");
      const auto ext = file.extension().string();
      const std::string kind = ext == ".html" ? "text/html; charset=utf-8" : ext == ".js" ? "text/javascript; charset=utf-8" :
                               ext == ".css" ? "text/css; charset=utf-8" : "application/octet-stream";
      std::ifstream input(file, std::ios::binary);
      return response(http::status::ok, std::string(std::istreambuf_iterator<char>(input), {}), kind);
    }
    if (request.method() != http::verb::post) return json(http::status::method_not_allowed, {{"error", "Use GET or POST"}});
    const std::string origin(request[http::field::origin]), host(request[http::field::host]);
    if (!origin.empty()) {
      auto scheme = origin.find("://");
      if (scheme == std::string::npos || origin.substr(scheme + 3) != host)
        return json(http::status::forbidden, {{"error", "Cross-origin requests are not allowed"}});
    }
    if (path == "/api/import") {
      const auto params = query(target);
      auto name = params.count("name") ? params.at("name") : "";
      dog_terrain::mapPath(root_, name);
      const std::string filename = params.count("filename") ? params.at("filename") : name + ".pcd";
      if (filename.size() > 255) throw std::invalid_argument("Source filename exceeds 255 bytes");
      auto config = dog_terrain::Config::parse(Json::parse(params.count("config") ? params.at("config") : "{}"));
      auto source = std::shared_ptr<fs::path>(new fs::path(root_ / (".upload-" + token() + ".pcd")),
        [](fs::path *p) { std::error_code ec; fs::remove(*p, ec); delete p; });
      std::ofstream stream(*source, std::ios::binary); stream << request.body(); stream.close();
      if (!stream) throw std::runtime_error("Failed saving upload");
      return json(http::status::accepted, submit([this, source, name, filename, config] {
        // Allocation is serialized by the map worker. saveProject still rejects
        // any external writer's collision, preserving existing projects.
        std::string allocated = name;
        for (unsigned suffix = 2; fs::exists(dog_terrain::mapPath(root_, allocated)); ++suffix) {
          if (suffix > 100000) throw std::invalid_argument("Too many maps with this base ID");
          const std::string ending = "-" + std::to_string(suffix);
          allocated = name.substr(0, 64 - ending.size()) + ending;
        }
        auto graph = std::make_shared<Graph>(dog_terrain::readPcd(*source), config);
        Json provenance = {{"filename", filename}, {"upload_sha256", dog_terrain::fileSha256(*source)}};
        dog_terrain::saveProject(root_, allocated, *graph, provenance); return std::make_pair(allocated, graph);
      }));
    }
    if (request.body().size() > 16384) return json(http::status::payload_too_large, {{"error", "JSON request exceeds 16 KiB"}});
    if (request[http::field::content_type].find("application/json") != 0)
      throw std::invalid_argument("Expected application/json");
    const auto body = Json::parse(request.body());
    if(path=="/api/maps/rename"||path=="/api/maps/delete"||path=="/api/maps/unload"||
       path=="/api/slam/pcds/rename"||path=="/api/slam/pcds/delete"){
      std::lock_guard<std::mutex> lock(mutex_);
      if(job_.value("state","")=="running")throw std::invalid_argument("地图任务进行中，请完成后再管理地图");
      const auto id=body.at("id").get<std::string>();const std::string kind=path.rfind("/api/maps/",0)==0?"map":"pcd";
      if(path=="/api/maps/unload"){
        if(id!=map_id_)throw std::invalid_argument("当前地图已变化，请刷新列表");
        graph_.reset();map_id_.clear();source_=Json::object();return json(http::status::ok,{{"unloaded",true},{"id",id}});
      }
      if(path.find("/rename")!=std::string::npos)return json(http::status::ok,library_.rename(kind,id,body.at("name").get<std::string>()));
      if(!body.value("confirm",false))throw std::invalid_argument("请确认删除所选地图");
      if(kind=="map"&&id==map_id_)throw std::invalid_argument("该地图已加载，请先卸载或切换地图");
      if(robot_){
        const auto state=robot_->telemetry();const auto loc=state.value("localization",Json::object());
        if(kind=="map"&&loc.value("map_ready",false)&&loc.contains("map_path")&&loc["map_path"].is_string()){
          const auto used=fs::weakly_canonical(loc["map_path"].get<std::string>()),target=fs::weakly_canonical(dog_terrain::mapPath(root_,id));
          if(used==target||used.parent_path()==target)throw std::invalid_argument("定位正在使用该地图，请先切换定位地图");
        }
        const auto mapping=state.value("mapping",Json::object());
        if(kind=="pcd"&&mapping.value("owned",false)){
          const auto relative=catalog_.resolve(id).lexically_relative(fs::weakly_canonical(mapping.at("output_dir").get<std::string>()));
          if(!relative.empty()&&*relative.begin()!="..")throw std::invalid_argument("该 PCD 属于正在进行的建图会话，请结束建图后再删除");
        }
      }
      return json(http::status::ok,library_.remove(kind,id));
    }
    if(path=="/api/import_saved"){
      const auto id=body.at("id").get<std::string>();const auto file=catalog_.resolve(id);
      const auto config=dog_terrain::Config::parse(body.value("config",Json::object()));
      const auto name=body.value("name","map-"+file.parent_path().filename().string());dog_terrain::mapPath(root_,name);
      return json(http::status::accepted,submit([this,file,id,name,config]{
        std::string allocated=name;
        for(unsigned suffix=2;fs::exists(dog_terrain::mapPath(root_,allocated));++suffix){
          if(suffix>100000)throw std::invalid_argument("Too many maps with this base ID");
          const auto ending="-"+std::to_string(suffix);allocated=name.substr(0,64-ending.size())+ending;
        }
        auto graph=std::make_shared<Graph>(dog_terrain::readPcd(file),config);
        Json provenance={{"filename",file.filename().string()},{"saved_pcd",id},{"upload_sha256",dog_terrain::fileSha256(file)}};
        fs::path keyframes;
        // Static filtering changes points only; cloud_static retains raw LIO poses.
        const auto companion=file.parent_path()/(file.filename()=="cloud_optimized.pcd" ?
            "localization_optimized_keyframes.csv" : "localization_keyframes.csv");
        if((file.filename()=="cloud.pcd"||file.filename()=="cloud_static.pcd"||file.filename()=="cloud_optimized.pcd")&&fs::exists(companion))keyframes=companion;
        // Legacy pure-LIO runs exported one raw trajectory on shutdown. Its
        // world frame is the same as cloud.pcd; never pair it with an optimized map.
        if(keyframes.empty()&&(file.filename()=="cloud.pcd"||file.filename()=="cloud_static.pcd")) {
          std::vector<fs::path> legacy;
          for(const auto &entry:fs::directory_iterator(file.parent_path())) {
            const auto filename=entry.path().filename().string();
            const std::string suffix="_raw_keyframes.csv";
            if(filename.size()>suffix.size()&&filename.compare(filename.size()-suffix.size(),suffix.size(),suffix)==0)
              legacy.push_back(entry.path());
          }
          if(legacy.size()==1)keyframes=legacy.front();
        }
        if(!keyframes.empty())provenance["keyframe_source"]=keyframes.filename().string();
        fs::path localization;
        if(file.filename()=="cloud_static.pcd"&&fs::exists(file.parent_path()/"localization.pcd")) {
          localization=file.parent_path()/"localization.pcd";
          provenance["localization_source"]="localization.pcd";
        }
        dog_terrain::saveProject(root_,allocated,*graph,provenance,keyframes,localization);return std::make_pair(allocated,graph);
      }));
    }
    if(path=="/api/settings/motion")return json(http::status::ok,robot_?robot_->saveMotionSettings(body):settings_->save(body));
    if(path=="/api/settings/navigation")return json(http::status::ok,navigationSettings(&body));
    if(path.rfind("/api/localization/",0)==0||path=="/api/navigation/start") {
      if(!robot_)return json(http::status::service_unavailable,{{"error","ROS bridge disabled"}});
      Json command=body;
      {std::lock_guard<std::mutex> lock(mutex_);
        if(!graph_||body.at("map_id")!=map_id_||body.at("revision")!=graph_->revision())
          throw std::invalid_argument("地图未加载或版本已变化，请重新加载后再设置位姿");
        command["map_project"]=fs::weakly_canonical(root_/map_id_).string();
        if(path=="/api/navigation/start")checkedLiveStart(*graph_,robot_->telemetry(),root_/map_id_);
      }
      return json(http::status::ok,robot_->command(path,command));
    }
    if (path.rfind("/api/control/", 0) == 0 || path.rfind("/api/slam/", 0) == 0 || path.rfind("/api/navigation/",0)==0) {
      if (!robot_) return json(http::status::service_unavailable, {{"error", "ROS bridge disabled; launch with --ros true"}});
      return json(http::status::ok, robot_->command(path, body));
    }
    if (path == "/api/demo") {
      const std::string name = body.value("name", "demo-" + token());
      dog_terrain::mapPath(root_, name);
      return json(http::status::accepted, submit([this, name] {
        auto graph = std::make_shared<Graph>(dog_terrain::switchbackCloud());
        dog_terrain::saveProject(root_, name, *graph); return std::make_pair(name, graph);
      }));
    }
    if (path == "/api/load") {
      const std::string name = body.at("name").get<std::string>();
      dog_terrain::mapPath(root_, name);
      return json(http::status::accepted, submit([this, name] {
        return std::make_pair(name, dog_terrain::loadProject(root_, name));
      }));
    }
    if (path == "/api/plan" || path == "/api/inspect_surface") {
      std::shared_ptr<Graph> graph; std::string name;
      { std::lock_guard<std::mutex> lock(mutex_); graph = graph_; name = map_id_; }
      if (!graph) throw std::invalid_argument("Load a map first");
      if (body.value("revision", "") != graph->revision() || body.value("map_id", "") != name)
        return json(http::status::conflict, {{"error", "Map changed; reload and select endpoints again"}});
      if(path=="/api/inspect_surface") {
        auto result=graph->inspectSurface(point(body.at("point")));
        if(body.contains("yaw"))result["requested_motion_clear"]=graph->motionClear(point(body.at("point")),point(body.at("point")),body.at("yaw").get<double>(),body.at("yaw").get<double>());
        result["map_id"]=name;return json(http::status::ok,result);
      }
      Json result;
      const bool live=body.value("use_robot_start",false);
      if(live&&!robot_)throw std::invalid_argument("实时定位规划需要启用 ROS 接口");
      const auto start=live?checkedLiveStart(*graph,robot_->telemetry(),root_/name):point(body.at("start"));
      try {result = graph->plan(start, point(body.at("goal")));}
      catch (const std::invalid_argument &error) {
        return json(http::status::bad_request,{{"error",error.what()},
            {"start_surface",graph->inspectSurface(start)},
            {"goal_surface",graph->inspectSurface(point(body.at("goal")))}});
      }
      result["map_id"] = name;result["start_source"]=live?"localization":"manual";
      return json(http::status::ok, result);
    }
    return json(http::status::not_found, {{"error", "Unknown endpoint"}});
  }
  fs::path root_, web_, console_, description_;
  ResourceMonitor resources_;
  std::mutex navigation_settings_mutex_;
  std::mutex mutex_;
  std::future<void> worker_;
  std::shared_ptr<Graph> graph_;
  std::string map_id_, job_id_;
  Json job_ = Json::object();
  Json source_ = Json::object();
  std::shared_ptr<RobotBridge> robot_;
  std::shared_ptr<MotionSettings> settings_;
  PcdCatalog catalog_;
  MapLibrary library_;
};

void session(tcp::socket socket, Service &service,asio::thread_pool &control,asio::thread_pool &responses,asio::thread_pool &streams) {
  try {
    socket.non_blocking(true);
    beast::flat_buffer buffer;
    http::request_parser<http::string_body> parser;
    parser.body_limit(128 * 1024 * 1024); parser.header_limit(32768);
    // Browsers may preconnect without sending a request. Bound idle sockets and
    // allow ROS shutdown to interrupt partial requests instead of hanging join().
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!parser.is_done() && rclcpp::ok()) {
      beast::error_code error;
      const auto bytes = http::read_some(socket, buffer, parser, error);
      if (error && error != asio::error::would_block && error != asio::error::try_again)
        throw boost::system::system_error(error);
      if (bytes) deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
      if (std::chrono::steady_clock::now() > deadline) return;
      if (error) std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!rclcpp::ok()) return;
    auto request=parser.release();
    const std::string target(request.target());
    if(target=="/api/control/stream"&&request[http::field::upgrade]=="websocket"){
      asio::post(streams,[socket=std::move(socket),request=std::move(request),&service]() mutable {service.controlStream(std::move(socket),std::move(request));});return;
    }
    // Tiny status/model snapshots must not queue behind slow point-cloud readers.
    const bool fast=target.rfind("/api/control/",0)==0||target=="/api/navigation/cancel"||target=="/api/navigation/paths"||target=="/api/robot"||target=="/api/robot/model"||target=="/api/settings/motion";
    auto &pool=fast?control:responses;
    asio::post(pool,[socket=std::move(socket),request=std::move(request),&service]() mutable {
    try {
    auto response = service.dispatch(request);
    http::response_serializer<http::string_body> serializer(response);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (!serializer.is_done() && rclcpp::ok()) {
      beast::error_code error;
      const auto bytes = http::write_some(socket, serializer, error);
      if (error && error != asio::error::would_block && error != asio::error::try_again)
        throw boost::system::system_error(error);
      if (bytes) deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
      if (std::chrono::steady_clock::now() > deadline) return;
      if (error) std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    beast::error_code ec; socket.shutdown(tcp::socket::shutdown_send, ec);
    }catch(const std::exception &e){std::cerr<<"HTTP response: "<<e.what()<<'\n';}
    });
  } catch (const std::exception &e) { std::cerr << "HTTP connection: " << e.what() << '\n'; }
}
}  // namespace

int main(int argc, char **argv) {
  try {
    std::string host = "127.0.0.1", root = "~/.local/share/dog/maps", web, output = "~/.local/share/dog/slam", motion_config;
    bool ros = false, sim_time = false, allow_mapping = true, mapping_bridge = true;
    int port = 8080;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "planning_preview [--host 127.0.0.1] [--port 8080] [--map-root PATH] [--web-root PATH] [--ros true] [--slam-output PATH] [--use-sim-time true]\n"; return 0;
      }
      if (i + 1 >= argc) throw std::invalid_argument("Missing argument for " + arg);
      std::string value = argv[++i];
      if (arg == "--host") host = value; else if (arg == "--port") port = std::stoi(value);
      else if (arg == "--map-root") root = value; else if (arg == "--web-root") web = value;
      else if (arg == "--ros") ros = value == "true";
      else if (arg == "--use-sim-time") sim_time = value == "true";
      else if (arg == "--allow-mapping") allow_mapping = value == "true";
      else if (arg == "--mapping-bridge") mapping_bridge = value == "true";
      else if (arg == "--motion-config") motion_config = value;
      else if (arg == "--slam-output") output = value;
      else throw std::invalid_argument("Unknown option: " + arg);
    }
    if (port < 0 || port > 65535) throw std::invalid_argument("Invalid port");
    if (web.empty()) web = (fs::path(ament_index_cpp::get_package_share_directory("dog_web")) / "web").string();
    auto settings=std::make_shared<MotionSettings>(motion_config.empty()?fs::absolute(expandHome(root))/".motion.json":fs::absolute(expandHome(motion_config)));
    std::shared_ptr<RobotBridge> robot;
    rclcpp::init(0, nullptr);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);
    std::thread ros_thread;
    if (ros) {
      robot = std::make_shared<RobotBridge>(fs::absolute(expandHome(output)).string(), sim_time, allow_mapping, mapping_bridge, settings);
      executor.add_node(robot);
      ros_thread = std::thread([&executor] { executor.spin(); });
    }
    // All normal returns and exceptions stop the executor before destroying ROS state.
    struct RosCleanup {
      rclcpp::executors::MultiThreadedExecutor &executor; std::thread &thread;
      ~RosCleanup() { executor.cancel(); if (thread.joinable()) thread.join(); }
    } cleanup{executor, ros_thread};
    Service service(expandHome(root), web, robot, settings,expandHome(output));
    asio::io_context context;
    tcp::acceptor acceptor(context, {asio::ip::make_address(host), static_cast<unsigned short>(port)});
    // Reading idle/preconnected sockets and serving map/save requests must not
    // occupy the threads that deliver the 20 Hz manual-control heartbeat.
    asio::thread_pool responses(4),control(2),streams(4),readers(8);
    std::cout << "Planning preview: http://" << host << ':' << acceptor.local_endpoint().port()
              << (ros ? " (ROS robot console)" : " (offline preview)") << std::endl;
    std::function<void()> accept;
    accept=[&]{acceptor.async_accept([&](boost::system::error_code error,tcp::socket socket){
      if(!error)asio::post(readers,[socket=std::move(socket),&service,&control,&responses,&streams]() mutable {session(std::move(socket),service,control,responses,streams);});
      if(rclcpp::ok()&&acceptor.is_open())accept();
    });};
    accept();while(rclcpp::ok())context.run_for(std::chrono::milliseconds(50));
    acceptor.close();context.stop();
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
