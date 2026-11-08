#include "itera_lio/output/image_recorder.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>

namespace itera_lio::output {
namespace {
double timestamp(const sensor_msgs::msg::Image &image) {
  return static_cast<double>(image.header.stamp.sec) +
         static_cast<double>(image.header.stamp.nanosec) * 1e-9;
}
}

ImageRecorder::ImageRecorder(const ImageRecorderOptions &options)
    : options_(options), directory_(options.directory) {
  if (!options_.enabled) return;
  std::filesystem::create_directories(directory_);
  thread_ = std::thread(&ImageRecorder::loop, this);
}

ImageRecorder::~ImageRecorder() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  cv_.notify_one();
  if (thread_.joinable()) thread_.join();
}

void ImageRecorder::submit(sensor_msgs::msg::Image::ConstSharedPtr image) {
  if (!options_.enabled || !image) return;
  const double t = timestamp(*image);
  std::lock_guard<std::mutex> lock(mutex_);
  const double min_period = 1.0 / options_.max_rate_hz;
  if (last_timestamp_ >= 0.0 && std::isfinite(t) &&
      t - last_timestamp_ < min_period &&
      t - last_timestamp_ < options_.max_interval_sec) return;
  last_timestamp_ = t;
  if (queue_.size() >= 32U) { ++dropped_; return; }
  queue_.push_back({std::move(image), next_index_++});
  cv_.notify_one();
}

void ImageRecorder::loop() {
  std::ofstream csv(directory_ / "images.csv");
  csv << "index,timestamp,filename\n";
  for (;;) {
    Item item;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
      if (queue_.empty() && stop_) break;
      item = std::move(queue_.front());
      queue_.pop_front();
    }
    try {
      const auto cv_image = cv_bridge::toCvCopy(item.image, "bgr8");
      std::ostringstream name;
      name << std::setfill('0') << std::setw(5) << item.index << ".png";
      const auto filename = name.str();
      cv::imwrite((directory_ / filename).string(), cv_image->image);
      csv << item.index << ',' << std::setprecision(17) << timestamp(*item.image)
          << ',' << filename << '\n';
      csv.flush();
    } catch (const std::exception &) {
      // A malformed encoding is counted by the missing CSV row.
    }
  }
}
}  // namespace itera_lio::output
