#pragma once
#include "dog_navigation/execution_types.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <atomic>
#include <unistd.h>

namespace dog_navigation {
// A separate startup profile: never contains hardware permission or calibration.
class NavigationMotionSettings {
public:
  using Json=nlohmann::json;
  inline static constexpr const char *keys[]={"walk_speed","stair_speed","acceleration","yaw_rate","yaw_acceleration","stair_yaw_rate"};
  explicit NavigationMotionSettings(std::filesystem::path path):path_(std::move(path)){}
  static Json values(const ExecutionOptions &o){return {{"walk_speed",o.walk_speed},{"stair_speed",o.stair_speed},
    {"acceleration",o.acceleration},{"yaw_rate",o.yaw_rate},{"yaw_acceleration",o.yaw_acceleration},{"stair_yaw_rate",o.stair_yaw_rate}};}
  static ExecutionOptions apply(const Json &j,ExecutionOptions o={}){
    if(!j.is_object()||j.size()!=6)throw std::invalid_argument("导航设置必须包含全部六项速度与加速度");
    for(const auto *key:keys){
      if(!j.contains(key)||!j.at(key).is_number())throw std::invalid_argument("导航参数必须为数字");
      const double v=j.at(key).get<double>();
      if(!std::isfinite(v)||v<=0)throw std::invalid_argument("导航参数必须为有限正数");
    }
    o.walk_speed=j.at("walk_speed");o.stair_speed=j.at("stair_speed");o.acceleration=j.at("acceleration");
    o.yaw_rate=j.at("yaw_rate");o.yaw_acceleration=j.at("yaw_acceleration");o.stair_yaw_rate=j.at("stair_yaw_rate");
    if(o.stair_speed>o.walk_speed||o.stair_yaw_rate>o.yaw_rate||o.walk_speed>1.)
      throw std::invalid_argument("楼梯速度不能超过普通速度；最大线速度不超过 1.00 m/s");
    if(o.walk_speed<.21||o.stair_speed<.21||o.yaw_rate<.21||o.stair_yaw_rate<.21)
      throw std::invalid_argument("BPX 普通/楼梯速度必须至少为 0.21，避免 0.20 启动死区");
    if(o.acceleration<.01||o.acceleration>1.||o.yaw_acceleration<.01||o.yaw_acceleration>2.||o.yaw_rate>1.)
      throw std::invalid_argument("线加减速度范围 0.01–1.00 m/s²；角加减速度范围 0.01–2.00 rad/s²；角速度不超过 1.00 rad/s");
    return o;
  }
  ExecutionOptions load(ExecutionOptions defaults={})const{
    if(!std::filesystem::exists(path_))return defaults;
    std::ifstream input(path_);const auto j=Json::parse(input);
    if(j.at("version")!=1)throw std::invalid_argument("Unsupported navigation motion settings version");
    return apply(j.at("limits"),defaults);
  }
  void save(const Json &limits)const{
    const auto checked=values(apply(limits));
    static std::atomic<unsigned long> serial{0};
    std::filesystem::create_directories(path_.parent_path());
    auto temporary=path_;temporary+=".tmp-"+std::to_string(getpid())+"-"+std::to_string(serial++);
    try{
      {std::ofstream output(temporary);output<<Json{{"version",1},{"limits",checked}}.dump(2)<<'\n';output.flush();
        if(!output)throw std::runtime_error("Cannot persist navigation motion settings");}
      std::filesystem::rename(temporary,path_);
    }catch(...){std::error_code ignored;std::filesystem::remove(temporary,ignored);throw;}
  }
private:
  std::filesystem::path path_;
};
}
