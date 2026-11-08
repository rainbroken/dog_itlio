#include "dog_navigation/localization_gate.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace dog_navigation {
std::filesystem::path canonicalMapPath(std::string path) {
  if(path.rfind("~/",0)==0) {
    const char *home=std::getenv("HOME");if(!home)throw std::invalid_argument("Use an absolute map path");
    path=std::string(home)+path.substr(1);
  }
  return std::filesystem::weakly_canonical(path);
}
LocalizationGate::LocalizationGate(std::filesystem::path project,double timeout)
    :project_(canonicalMapPath(project.string())),timeout_(timeout) {
  if(!std::isfinite(timeout_)||timeout_<=0)throw std::invalid_argument("Invalid localization timeout");
}
std::string LocalizationGate::updateJson(const std::string &payload,Time receipt) {
  valid_=false;receipt_=receipt;
  try {
    const auto j=nlohmann::json::parse(payload);
    const auto path=canonicalMapPath(j.at("map_path").get<std::string>());
    const bool same=path==project_||path==project_/"localization.pcd"||path==project_/"cloud.pcd";
    if(!same)return "map_mismatch";
    valid_=j.at("valid").get<bool>()&&j.at("map_ready").get<bool>();
    return valid_?"":"localization_invalid";
  }catch(const std::exception &){return "invalid_localization_status";}
}
double LocalizationGate::remaining(Time now) const {
  if(!valid_||now<receipt_)return 0;
  return std::max(0.,timeout_-std::chrono::duration<double>(now-receipt_).count());
}
bool LocalizationGate::ready(Time now) const {return remaining(now)>0;}
}  // namespace dog_navigation
