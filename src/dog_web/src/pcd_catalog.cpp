#include "pcd_catalog.hpp"
#include <algorithm>
#include <stdexcept>

std::filesystem::path PcdCatalog::resolve(const std::string &id) const {
  namespace fs=std::filesystem;const fs::path relative(id);
  if(relative.empty()||relative.is_absolute()||relative.extension()!=".pcd")throw std::invalid_argument("Invalid saved PCD ID");
  for(const auto &part:relative)if(part==".."||part=="."||part.string().front()=='.')throw std::invalid_argument("Invalid saved PCD ID");
  auto component=root_;for(const auto &part:relative){component/=part;if(fs::is_symlink(component))throw std::invalid_argument("Saved PCD links are not supported");}
  const auto path=fs::weakly_canonical(root_/relative);const auto inside=path.lexically_relative(root_);
  if(inside.empty()||*inside.begin()==".."||!fs::is_regular_file(path))throw std::invalid_argument("Saved PCD is missing or outside SLAM directory");
  return path;
}
nlohmann::json PcdCatalog::list() const {
  namespace fs=std::filesystem;auto files=nlohmann::json::array();
  if(!fs::exists(root_))return {{"files",files},{"root",root_.string()}};
  for(auto it=fs::recursive_directory_iterator(root_,fs::directory_options::skip_permission_denied);it!=fs::recursive_directory_iterator();++it){
    if(it->is_symlink()||it->path().filename().string().front()=='.'||it.depth()>4){if(it->is_directory())it.disable_recursion_pending();continue;}
    if(!it->is_regular_file()||it->path().extension()!=".pcd")continue;
    const auto id=it->path().lexically_relative(root_).generic_string();
    // Match explicit save outputs, excluding temporary scan archives.
    if(it->path().filename()!="cloud.pcd"&&it->path().filename()!="cloud_optimized.pcd"&&
       it->path().filename()!="cloud_static.pcd"&&it->path().filename()!="full_map.pcd")continue;
    files.push_back({{"id",id},{"bytes",it->file_size()},{"modified",it->last_write_time().time_since_epoch().count()}});
  }
  std::sort(files.begin(),files.end(),[](const auto &a,const auto &b){return a.at("modified")>b.at("modified");});
  return {{"files",files},{"root",root_.string()}};
}
