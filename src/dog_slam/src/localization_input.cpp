#include "map_localizer_node.hpp"
#include "ros_pose.hpp"

namespace dog_slam {
void MapLocalizer::enqueue(Command c, const std::string &label) {
    {std::lock_guard<std::mutex> lock(mutex_); ++generation_; public_.valid=false; label_=label;
      if(c.kind==Kind::GlobalRelocalize){global_started_=Steady::now();global_duration_ms_=0;global_timing_state_="running";}
      else if(c.kind==Kind::Load||c.kind==Kind::Reset){global_timing_state_="idle";global_duration_ms_=0;}
      else if(c.kind==Kind::Seed&&global_timing_state_=="running"){
        global_duration_ms_=std::chrono::duration<double,std::milli>(Steady::now()-global_started_).count();global_timing_state_="cancelled";
      }
      commands_.push_back(std::move(c)); if (commands_.size()>16) commands_.pop_front();}
    condition_.notify_one();
  }
void MapLocalizer::receive(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (msg->header.stamp.sec<0 || msg->header.stamp.nanosec>=1000000000u || msg->is_bigendian) return;
    const double stamp=rclcpp::Time(msg->header.stamp).seconds(), age=now().seconds()-stamp;
    if (msg->header.frame_id != cloud_frame_ || stamp <= 0 || age > max_scan_age_ || age < -.1 ||
        static_cast<uint64_t>(msg->width)*msg->height > 500000 || msg->height == 0 ||
        msg->row_step < static_cast<uint64_t>(msg->point_step)*msg->width ||
        msg->data.size() < static_cast<uint64_t>(msg->row_step)*msg->height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Reject cloud: frame, layout or timestamp invalid"); return;
    }
    for (const char *name : {"x","y","z"}) {
      auto it=std::find_if(msg->fields.begin(),msg->fields.end(),[&](const auto &f){return f.name==name;});
      if (it==msg->fields.end() || it->datatype!=sensor_msgs::msg::PointField::FLOAT32 || it->count!=1 ||
          it->offset + sizeof(float)>msg->point_step) return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (stamp <= last_received_stamp_) {
      if (stamp < last_received_stamp_ - .2) {
        ++generation_; public_.valid=false; label_="clock_reset_requires_initial_pose";
        Command c; c.kind=Kind::Reset; commands_.push_back(c); last_received_stamp_=stamp;
      }
      return;
    }
    const auto receipt=Steady::now();
    // Registration deliberately keeps only the newest scan and can spend
    // seconds on global search. Detect gaps in received input, not in the
    // sparse scans chosen by that worker.
    if (last_received_stamp_>0 && (stamp-last_received_stamp_>timeout_ ||
        std::chrono::duration<double>(receipt-input_receipt_).count()>timeout_)) {
      ++generation_; public_.valid=false; label_="lost"; reason_="scan_gap";
      Command c; c.kind=Kind::Invalidate; c.reason="scan_gap"; commands_.push_back(std::move(c));
      if (commands_.size()>16) commands_.pop_front();
    }
    last_received_stamp_=stamp; input_receipt_=receipt; cloud_=std::move(msg); condition_.notify_one();
  }
Pose3 MapLocalizer::lookup(const std::string &parent, const std::string &child, const builtin_interfaces::msg::Time &stamp) {
    if (parent==child) return Pose3::Identity();
    return transform(buffer_->lookupTransform(parent,child,rclcpp::Time(stamp),rclcpp::Duration::from_seconds(.08)).transform);
  }
}  // namespace dog_slam
