#pragma once
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <string>
#include <cstddef>

namespace itera_lio::common {
// One owner per worker thread/file. Recording happens after the measured task.
inline void recordWorkerTask(const char* kind, std::size_t from, std::size_t to,
  double duration_ms, bool success, bool expired,
  double initial_cost, double final_cost, int iterations, std::size_t nodes) {
  const char* directory = std::getenv("ITERA_LIO_WORKER_METRICS_DIR");
  if (!directory || !*directory) return;
  thread_local std::ofstream stream(std::string(directory) + "/" + kind + "_tasks.csv");
  thread_local bool header = false;
  if (!header) {
    stream << "wall_timestamp,from_id,to_id,duration_ms,success,expired,initial_cost,final_cost,iterations,active_nodes\n" << std::setprecision(17);
    header = true;
  }
  const double timestamp = std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  stream << timestamp << ',' << from << ',' << to << ',' << duration_ms << ','
         << success << ',' << expired << ',' << initial_cost << ',' << final_cost
         << ',' << iterations << ',' << nodes << '\n';
  stream.flush();
}
}
