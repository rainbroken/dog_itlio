#include "map_localizer_node.hpp"
#include "ros_pose.hpp"
#include <nlohmann/json.hpp>

namespace dog_slam {
using Json=nlohmann::json;
void MapLocalizer::publishTransform() {
    Pose3 map_to_odom;
    {std::lock_guard<std::mutex> lock(mutex_); if (!freshLocked()) return; map_to_odom=public_.map_to_odom;}
    try {
      auto odom=buffer_->lookupTransform(odom_frame_,base_frame_,tf2::TimePointZero);
      const double age=now().seconds()-rclcpp::Time(odom.header.stamp).seconds();
      if (age<-.1 || age>timeout_) return;
      auto body=messagePose(map_to_odom*transform(odom.transform));
      geometry_msgs::msg::TransformStamped tf; tf.header.stamp=odom.header.stamp;
      tf.header.frame_id=map_frame_; tf.child_frame_id=odom_frame_;
      auto p=messagePose(map_to_odom); tf.transform.translation.x=p.position.x; tf.transform.translation.y=p.position.y;
      tf.transform.translation.z=p.position.z; tf.transform.rotation=p.orientation;
      // Recheck after TF lookup so a concurrent reset/load cannot publish old map TF.
      std::lock_guard<std::mutex> lock(mutex_);
      if (!freshLocked() || !public_.map_to_odom.matrix().isApprox(map_to_odom.matrix(),1e-12)) return;
      broadcaster_->sendTransform(tf);
      geometry_msgs::msg::PoseStamped msg; msg.header.stamp=odom.header.stamp; msg.header.frame_id=map_frame_; msg.pose=body;
      pose_pub_->publish(msg);
    } catch (const tf2::TransformException &) {}
  }
void MapLocalizer::publishStatus() {
    Json value;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (public_.valid && !freshLocked()) {
        public_.valid=false; label_="lost"; reason_="stale_data"; ++generation_;
        Command c; c.kind=Kind::Invalidate; commands_.push_back(c); condition_.notify_one();
      }
      value={{"state",label_},{"valid",freshLocked()},{"map_ready",map_ready_},{"map_path",loaded_path_},
        {"input_fresh",last_received_stamp_>0 && std::chrono::duration<double>(Steady::now()-input_receipt_).count()<=timeout_},
        {"input_age_ms",last_received_stamp_>0 ? Json(std::chrono::duration<double,std::milli>(Steady::now()-input_receipt_).count()) : Json(nullptr)},
        {"registration_running",registration_running_},
        {"global_timing_state",global_timing_state_},
        {"global_relocalization_elapsed_ms",global_timing_state_=="running"?
          std::chrono::duration<double,std::milli>(Steady::now()-global_started_).count():global_duration_ms_},
        {"map_frame",map_frame_},{"odom_frame",odom_frame_},{"base_frame",base_frame_},
        {"reason",reason_},{"has_prior",public_.has_prior},{"confirmations",public_.confirmations},
        {"failures",public_.failures},{"rmse",public_.match.rmse},{"overlap",public_.match.overlap},
        {"reverse_overlap",public_.match.reverse_overlap},{"geometry_ratio",public_.match.geometry_ratio},
        {"inliers",public_.match.inliers},{"hypotheses",public_.match.hypotheses},
        {"relocalization_scope",public_.match.scope},{"global_relocalization_enabled",options_.global.enabled},
        {"localization_source",public_.source},
        {"registration_duration_ms",public_.match.duration_ms}};
    }
    std_msgs::msg::String msg; msg.data=value.dump(); status_pub_->publish(msg);
    diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp=now();
    diagnostic_msgs::msg::DiagnosticStatus d; d.name="dog_slam/localization"; d.hardware_id="map_localizer";
    d.level=value["valid"].get<bool>() ? d.OK : value["state"]=="lost" || value["state"]=="map_error" ? d.ERROR : d.WARN;
    d.message=value["state"].get<std::string>()+": "+value["reason"].get<std::string>();
    for (const auto &key : {"valid","map_path","rmse","overlap","geometry_ratio"}) {
      diagnostic_msgs::msg::KeyValue kv; kv.key=key; kv.value=value[key].dump(); d.values.push_back(kv);
    }
    array.status.push_back(d); diagnostic_pub_->publish(array);
  }
}  // namespace dog_slam
