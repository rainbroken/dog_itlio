#include "map_library.hpp"
#include "dog_terrain/terrain_graph.hpp"
#include <chrono>
#include <fstream>
#include <algorithm>
#include <atomic>

namespace fs=std::filesystem;
using Json=nlohmann::json;
namespace {
long long modified(const fs::path &path){
  const auto time=fs::last_write_time(path)-fs::file_time_type::clock::now()+std::chrono::system_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count();
}
std::string nonce(){static std::atomic<unsigned> serial{0};return std::to_string(std::chrono::system_clock::now().time_since_epoch().count())+"-"+std::to_string(serial++);}
void writeJson(const fs::path &path,const Json &data){std::ofstream out(path);out<<data.dump(2);out.close();if(!out)throw std::runtime_error("Cannot write map library metadata");}
std::string validName(std::string name){
  const auto first=name.find_first_not_of(" \t\r\n"),last=name.find_last_not_of(" \t\r\n");
  if(first==std::string::npos)throw std::invalid_argument("名称不能为空");
  name=name.substr(first,last-first+1);size_t count=0;
  for(unsigned char c:name){if(c<32||c==127||c=='/'||c=='\\')throw std::invalid_argument("名称不能包含控制字符或路径分隔符");if((c&0xc0)!=0x80)++count;}
  if(count>80||name.size()>320)throw std::invalid_argument("名称最多 80 个字符");
  return name;
}
}
MapLibrary::MapLibrary(fs::path maps,PcdCatalog &pcds):maps_(fs::weakly_canonical(maps)),pcds_(pcds),names_(Json::object()){
  fs::create_directories(maps_);const auto file=maps_/".library-names.json";
  if(fs::exists(file)){
    if(fs::is_symlink(file)||fs::file_size(file)>4*1024*1024)throw std::invalid_argument("Invalid map library metadata");
    std::ifstream in(file);names_=Json::parse(in);if(!names_.is_object())throw std::invalid_argument("Invalid map library metadata");
  }
}
std::string MapLibrary::label(const std::string &kind,const std::string &id) const{
  const auto key=kind+":"+id;return names_.contains(key)?names_.at(key).get<std::string>():id;
}
fs::path MapLibrary::resolve(const std::string &kind,const std::string &id) const{
  if(kind=="pcd")return pcds_.resolve(id);
  if(kind!="map")throw std::invalid_argument("Unknown map item type");
  const auto path=dog_terrain::mapPath(maps_,id);
  if(!fs::is_directory(path)||!fs::is_regular_file(path/"map.json"))throw std::invalid_argument("地图项目不存在");
  return path;
}
std::string MapLibrary::displayName(const std::string &kind,const std::string &id) const{std::lock_guard<std::mutex> lock(mutex_);return label(kind,id);}
Json MapLibrary::projects(const std::string &loaded) const{
  std::lock_guard<std::mutex> lock(mutex_);Json entries=Json::array();std::vector<std::string> ids;
  for(const auto &entry:fs::directory_iterator(maps_)){
    const auto id=entry.path().filename().string();
    if(id.empty()||id.front()=='.'||entry.is_symlink()||!fs::is_regular_file(entry.path()/"map.json"))continue;
    Json row={{"id",id},{"name",label("map",id)},{"loaded",id==loaded},{"modified_ms",modified(entry.path()/"map.json")},{"bytes",0}};
    uintmax_t bytes=0;for(const auto &file:fs::recursive_directory_iterator(entry.path()))if(!file.is_symlink()&&file.is_regular_file())bytes+=file.file_size();row["bytes"]=bytes;
    try{
      if(fs::is_symlink(entry.path()/"map.json"))throw std::runtime_error("Invalid metadata link");
      std::ifstream in(entry.path()/"map.json");const auto meta=Json::parse(in);
      row["summary"]=meta.value("summary",Json::object());row["source"]=meta.value("source",Json::object());
    }catch(const std::exception &){row["error"]="地图元数据不可读";}
    ids.push_back(id);entries.push_back(std::move(row));
  }
  std::sort(ids.begin(),ids.end());
  std::sort(entries.begin(),entries.end(),[](const auto &a,const auto &b){return a.at("modified_ms")>b.at("modified_ms");});
  return {{"maps",ids},{"projects",entries},{"loaded",loaded}};
}
Json MapLibrary::savedPcds() const{
  std::lock_guard<std::mutex> lock(mutex_);auto catalog=pcds_.list();
  for(auto &file:catalog["files"]){const auto id=file.at("id").get<std::string>();file["name"]=label("pcd",id);file["modified_ms"]=modified(pcds_.resolve(id));}
  return catalog;
}
void MapLibrary::persist(const Json &names) const{
  const auto tmp=maps_/(".names-"+nonce());
  try{writeJson(tmp,names);fs::rename(tmp,maps_/".library-names.json");}catch(...){std::error_code ec;fs::remove(tmp,ec);throw;}
}
Json MapLibrary::rename(const std::string &kind,const std::string &id,const std::string &requested){
  std::lock_guard<std::mutex> lock(mutex_);resolve(kind,id);const auto name=validName(requested),key=kind+":"+id;
  for(const auto &[k,value]:names_.items())if(k!=key&&k.rfind(kind+":",0)==0&&value==name)throw std::invalid_argument("已有同名地图，请换一个名称");
  if(kind=="map"&&name!=id&&fs::exists(maps_/name))throw std::invalid_argument("名称与已有地图编号冲突");
  auto next=names_;next[key]=name;persist(next);names_=std::move(next);return {{"id",id},{"name",name}};
}
Json MapLibrary::remove(const std::string &kind,const std::string &id){
  std::lock_guard<std::mutex> lock(mutex_);const auto source=resolve(kind,id);
  const auto base=kind=="map"?maps_:pcds_.root();
  const auto trash=base/".trash";if(fs::is_symlink(trash))throw std::invalid_argument("Invalid recovery directory");
  fs::create_directories(trash);const auto entry=trash/nonce();fs::create_directory(entry);
  try{
    writeJson(entry/"record.json",{{"kind",kind},{"id",id},{"name",label(kind,id)},{"original_path",source.string()}});
    fs::rename(source,entry/"payload");
    try{auto next=names_;next.erase(kind+":"+id);persist(next);names_=std::move(next);}catch(...){fs::rename(entry/"payload",source);throw;}
  }catch(...){std::error_code ec;if(!fs::exists(entry/"payload"))fs::remove_all(entry,ec);throw;}
  return {{"id",id},{"deleted",true},{"recovery_path",entry.string()}};
}
