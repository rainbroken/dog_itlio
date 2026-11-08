#ifndef ITERA_LIO__OUTPUT__IMAGE_RECORDER_HPP_
#define ITERA_LIO__OUTPUT__IMAGE_RECORDER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>

#include "sensor_msgs/msg/image.hpp"

namespace itera_lio::output {

struct ImageRecorderOptions {
  bool enabled{false};
  std::string directory;
  double max_rate_hz{5.0};
  double max_interval_sec{0.5};
};

class ImageRecorder {
public:
  ImageRecorder(const ImageRecorderOptions &options);
  ~ImageRecorder();
  void submit(sensor_msgs::msg::Image::ConstSharedPtr image);

private:
  struct Item { sensor_msgs::msg::Image::ConstSharedPtr image; std::size_t index; };
  void loop();
  ImageRecorderOptions options_;
  std::filesystem::path directory_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<Item> queue_;
  std::thread thread_;
  bool stop_{false};
  std::size_t next_index_{1U};
  double last_timestamp_{-1.0};
  std::size_t dropped_{0U};
};

}  // namespace itera_lio::output

#endif
