// B005 diagnostic instrumentation, not an estimator change.
#pragma once
#include <chrono>
#include <map>
#include <string>
#include <ostream>
#include <vector>
#include <cstdlib>
#ifndef ITERA_LIO_GATED_DIAGNOSTICS
#define ITERA_LIO_GATED_DIAGNOSTICS 0
#endif
#include "itera_lio/common/geometry.hpp"
namespace b005 {
struct Entry {double ms=0; unsigned long long calls=0,work=0;};
inline thread_local std::map<std::string,Entry> entries;
inline thread_local std::string prefix;
struct PoseTrace {std::string initialization;itera_lio::common::Pose pose;bool converged;int iterations;std::string reason;};
inline thread_local std::vector<PoseTrace> poses;
inline bool capture_poses=false;
inline bool enabled(){
#if ITERA_LIO_GATED_DIAGNOSTICS
 static const bool value=[](){const char* p=std::getenv("ITERA_LIO_INTERNAL_PROFILE");return p&&p[0]=='1'&&p[1]=='\0';}();return value;
#else
 return true;
#endif
}
inline void recordPose(const std::string& name,const itera_lio::common::Pose& pose,bool converged,int iterations,const std::string& reason){if(capture_poses)poses.push_back({name,pose,converged,iterations,reason});}
inline void reset(){entries.clear();prefix.clear();poses.clear();}
inline void count(const std::string& name,unsigned long long n=1){if(enabled())entries[prefix+name].work+=n;}
inline void count(const char* name,unsigned long long n=1){if(enabled())entries[prefix+name].work+=n;}
struct Context {std::string saved;bool active; explicit Context(const char* name):active(enabled()){if(active){saved=prefix;prefix=std::string(name)+".";}} explicit Context(const std::string& name):Context(name.c_str()){} ~Context(){if(active)prefix=saved;}};
struct Timer {
 std::string key; std::chrono::steady_clock::time_point start; bool active=true;
 explicit Timer(const char* name):active(enabled()){if(active){key=prefix+name;start=std::chrono::steady_clock::now();}}
 explicit Timer(const std::string& name):Timer(name.c_str()){}
 void stop(){if(!active)return;auto end=std::chrono::steady_clock::now();auto& e=entries[key];e.ms+=std::chrono::duration<double,std::milli>(end-start).count();++e.calls;active=false;}
 ~Timer(){stop();}
};
inline void dump(std::ostream& out,const std::string& pair,const std::string& engine){for(const auto& [k,e]:entries)out<<pair<<','<<engine<<','<<k<<','<<e.ms<<','<<e.calls<<','<<e.work<<'\n';}
}
