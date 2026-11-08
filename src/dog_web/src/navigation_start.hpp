#pragma once
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <cmath>
#include <stdexcept>

inline Eigen::Vector3d localizedPlanningStart(const nlohmann::json &state,
    const std::filesystem::path &project,const std::string &revision) {
  const auto &loc=state.at("localization"),&nav=state.at("navigation"),&pose=state.at("pose");
  namespace fs=std::filesystem;const auto expected=fs::weakly_canonical(project);
  const auto loaded=fs::weakly_canonical(loc.at("map_path").get<std::string>());
  if(!loc.value("fresh",false)||!loc.value("valid",false)||!loc.value("map_ready",false)||
     (loaded!=expected&&loaded!=expected/"localization.pcd"&&loaded!=expected/"cloud.pcd")||
     !nav.value("fresh",false)||nav.at("map_revision")!=revision||
     fs::weakly_canonical(nav.at("map_project").get<std::string>())!=expected)
    throw std::invalid_argument("地图定位尚未就绪或导航地图版本不同，请重新确认定位");
  if(!state.value("tf_ready",false)||!pose.is_object()||pose.value("frame","")!="map")
    throw std::invalid_argument("缺少地图中的机身位姿");
  const double age=pose.at("age_ms").get<double>(),offset=nav.at("base_ground_offset").get<double>();
  if(!std::isfinite(age)||age< -50||age>300||!std::isfinite(offset)||offset<=0||offset>2)
    throw std::invalid_argument("机身位姿已过期或地面高度配置无效");
  const auto &p=pose.at("position"),&q=pose.at("quaternion");
  if(p.size()!=3||q.size()!=4)throw std::invalid_argument("机身位姿格式无效");
  Eigen::Vector3d position(p.at(0).get<double>(),p.at(1).get<double>(),p.at(2).get<double>());
  Eigen::Quaterniond rotation(q.at(3).get<double>(),q.at(0).get<double>(),q.at(1).get<double>(),q.at(2).get<double>());
  if(!position.allFinite()||!rotation.coeffs().allFinite()||std::abs(rotation.norm()-1)>.001)
    throw std::invalid_argument("机身位姿数据无效");
  return position+rotation.normalized()*Eigen::Vector3d(0,0,-offset);
}
