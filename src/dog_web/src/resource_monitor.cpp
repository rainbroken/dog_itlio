#include "resource_monitor.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <stdexcept>

nlohmann::json ResourceMonitor::snapshot(){
  using Json=nlohmann::json;using Clock=std::chrono::steady_clock;
  std::lock_guard<std::mutex> lock(mutex_);const auto now=Clock::now();
  if(!cached_.is_null()&&now-sampled_<std::chrono::milliseconds(250))return cached_;
  std::ifstream stat(proc_/"stat"),memory(proc_/"meminfo");
  if(!stat||!memory)throw std::runtime_error("Cannot read host /proc resource counters");
  Json cpus=Json::array();std::map<std::string,Ticks> current;std::string line;
  while(std::getline(stat,line)){
    std::istringstream row(line);std::string name;row>>name;
    if(name.rfind("cpu",0)!=0)continue;
    std::vector<uint64_t> values;uint64_t value;while(row>>value)values.push_back(value);
    if(values.size()<4)continue;
    Ticks ticks;for(size_t i=0;i<std::min(size_t(8),values.size());++i)ticks.total+=values[i];
    ticks.idle=values[3]+(values.size()>4?values[4]:0);current[name]=ticks;
    Json percent=nullptr;const auto found=previous_.find(name);
    if(found!=previous_.end()&&ticks.total>found->second.total&&ticks.idle>=found->second.idle){
      const auto total=ticks.total-found->second.total,idle=ticks.idle-found->second.idle;
      percent=std::clamp(100.*(double(total)-double(idle))/double(total),0.,100.);
    }
    cpus.push_back({{"id",name},{"percent",percent}});
  }
  if(cpus.empty()||!current.count("cpu"))throw std::runtime_error("Invalid CPU resource counters");
  std::map<std::string,uint64_t> mem;
  while(std::getline(memory,line)){
    std::istringstream row(line);std::string key;uint64_t value;
    if(row>>key>>value){if(!key.empty()&&key.back()==':')key.pop_back();mem[key]=value*1024;}
  }
  if(!mem.count("MemTotal")||!mem.count("MemAvailable")||!mem.count("SwapTotal")||!mem.count("SwapFree"))
    throw std::runtime_error("Invalid memory resource counters");
  const auto usage=[](uint64_t total,uint64_t available){const auto used=total-std::min(total,available);
    return Json{{"total_bytes",total},{"used_bytes",used},{"percent",total?100.*double(used)/double(total):0.}};};
  cached_={{"cpus",cpus},{"memory",usage(mem.at("MemTotal"),mem.at("MemAvailable"))},
    {"swap",usage(mem.at("SwapTotal"),mem.at("SwapFree"))}};
  previous_=std::move(current);sampled_=now;return cached_;
}
