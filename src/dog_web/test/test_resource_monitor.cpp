#include "resource_monitor.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <thread>
#include <unistd.h>

TEST(ResourceMonitor, ComputesCpuDeltasAndAvailableMemoryWithoutDoubleCountingGuest){
  const auto root=std::filesystem::temp_directory_path()/("dog-resources-"+std::to_string(getpid()));
  std::filesystem::create_directories(root);
  std::ofstream(root/"stat")<<"cpu  100 0 100 800 0 0 0 0 20 0\ncpu0 100 0 100 800 0 0 0 0 20 0\n";
  std::ofstream(root/"meminfo")<<"MemTotal: 1000 kB\nMemAvailable: 400 kB\nSwapTotal: 200 kB\nSwapFree: 150 kB\n";
  ResourceMonitor monitor(root);auto value=monitor.snapshot();EXPECT_TRUE(value["cpus"][0]["percent"].is_null());
  EXPECT_EQ(value["memory"]["used_bytes"],600*1024);EXPECT_DOUBLE_EQ(value["memory"]["percent"],60.);
  EXPECT_DOUBLE_EQ(value["swap"]["percent"],25.);
  std::ofstream(root/"stat")<<"cpu 150 0 150 900 0 0 0 0 40 0\ncpu0 150 0 150 900 0 0 0 0 40 0\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(260));value=monitor.snapshot();
  EXPECT_DOUBLE_EQ(value["cpus"][0]["percent"],50.);EXPECT_DOUBLE_EQ(value["cpus"][1]["percent"],50.);
  std::ofstream(root/"meminfo")<<"MemTotal: 1000 kB\nMemAvailable: 400 kB\nSwapTotal: 0 kB\nSwapFree: 0 kB\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(260));value=monitor.snapshot();
  EXPECT_DOUBLE_EQ(value["swap"]["percent"],0.);std::filesystem::remove_all(root);
}
TEST(ResourceMonitor, MissingProcDataDoesNotReportFalseZeroUsage){
  ResourceMonitor monitor("/definitely-missing-dog-proc");EXPECT_THROW(monitor.snapshot(),std::runtime_error);
}
