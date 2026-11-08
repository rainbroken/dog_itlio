#pragma once
#include <chrono>
#include <map>
#include <string>
#include <cstdlib>
#ifndef ITERA_LIO_GATED_DIAGNOSTICS
#define ITERA_LIO_GATED_DIAGNOSTICS 0
#endif
namespace b007 {
struct Entry {double ms=0;unsigned long long calls=0,work=0;};
inline thread_local std::map<std::string,Entry> entries;
inline bool enabled(){
#if ITERA_LIO_GATED_DIAGNOSTICS
 static const bool value=[](){const char* p=std::getenv("ITERA_LIO_INTERNAL_PROFILE");return p&&p[0]=='1'&&p[1]=='\0';}();return value;
#else
 return true;
#endif
}
inline void reset(){entries.clear();}
inline void count(const std::string& key,unsigned long long n=1){if(enabled())entries[key].work+=n;}
inline void count(const char* key,unsigned long long n=1){if(enabled())entries[key].work+=n;}
struct Timer {
 std::string key;std::chrono::steady_clock::time_point start;bool active=true;
 explicit Timer(const char* name):active(enabled()){if(active){key=name;start=std::chrono::steady_clock::now();}}
 explicit Timer(const std::string& name):Timer(name.c_str()){}
 void stop(){if(!active)return;auto end=std::chrono::steady_clock::now();auto& e=entries[key];e.ms+=std::chrono::duration<double,std::milli>(end-start).count();++e.calls;active=false;}
 ~Timer(){stop();}
};
}
