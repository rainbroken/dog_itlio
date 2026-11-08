#include "motion_settings.hpp"
#include <atomic>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <unistd.h>

namespace {
using Json=nlohmann::json;
constexpr std::array<double,3> maxima{1.00,.30,1.00};
constexpr std::array<const char *,3> keys{"x","y","yaw"};
std::array<double,3> parse(const Json &request) {
  if(!request.is_object()||request.size()!=3)throw std::invalid_argument("速度设置必须包含 x、y、yaw 三项");
  std::array<double,3> values;
  for(size_t i=0;i<keys.size();++i){
    if(!request.contains(keys[i])||!request[keys[i]].is_number())throw std::invalid_argument("速度必须为数字");
    values[i]=request[keys[i]].get<double>();
    if(!std::isfinite(values[i])||values[i]<0||values[i]>maxima[i])
      throw std::invalid_argument("速度超出允许范围：前后 1.00 m/s、平移 0.30 m/s、旋转 1.00 rad/s");
  }
  return values;
}
Json json(const std::array<double,3> &values){return {{"x",values[0]},{"y",values[1]},{"yaw",values[2]}};}
}

MotionSettings::MotionSettings(std::filesystem::path path):path_(std::move(path)) {
  if(path_.empty())return;
  if(std::filesystem::exists(path_)){
    std::ifstream input(path_);auto document=Json::parse(input);
    if(!document.is_object()||document.value("version",0)!=1)throw std::invalid_argument("Unsupported motion settings file: "+path_.string());
    document.erase("version");values_=parse(document);
  }else persist(values_);
}
Json MotionSettings::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);auto result=json(values_);
  result["maxima"]=json(maxima);result["timeout_ms"]=350;return result;
}
std::array<double,3> MotionSettings::limits() const {std::lock_guard<std::mutex> lock(mutex_);return values_;}
Json MotionSettings::save(const Json &request) {
  const auto values=parse(request);
  {std::lock_guard<std::mutex> lock(mutex_);persist(values);values_=values;}
  return snapshot();
}
void MotionSettings::persist(const std::array<double,3> &values) const {
  if(path_.empty())return;
  namespace fs=std::filesystem;static std::atomic<unsigned long> counter{0};
  if(!path_.parent_path().empty())fs::create_directories(path_.parent_path());
  auto temporary=path_;temporary+=".tmp-"+std::to_string(getpid())+"-"+std::to_string(counter++);
  try{
    auto document=json(values);document["version"]=1;
    {std::ofstream output(temporary);output<<document.dump(2)<<'\n';output.flush();
      if(!output)throw std::runtime_error("Cannot save motion settings: "+path_.string());}
    fs::rename(temporary,path_); // Readers see either complete old or complete new JSON.
  }catch(...){std::error_code ignored;fs::remove(temporary,ignored);throw;}
}
