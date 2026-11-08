#include "map_localizer_node.hpp"
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace dog_slam {
void MapLocalizer::run() {
    std::unique_ptr<LocalizationEngine> engine;
    std::optional<Command> seed;
    std::optional<Pose3> previous_odom;
    double previous_stamp=0;
    auto next_frame=Steady::now();
    while (true) {
      std::deque<Command> commands; sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud; uint64_t generation;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_until(lock,next_frame,[&]{return stopping_ || !commands_.empty();});
        if (stopping_) return;
        commands.swap(commands_); generation=generation_;
        if (Steady::now()>=next_frame) {cloud=std::move(cloud_); cloud_.reset();}
      }
      try {
        for (const auto &command : commands) {
          if (command.kind==Kind::Load) {
            engine.reset(); seed.reset(); previous_odom.reset();
            {std::lock_guard<std::mutex> lock(mutex_); map_ready_=false; public_=LocalizationUpdate{}; loaded_path_=command.path;}
            engine=std::make_unique<LocalizationEngine>(readLocalizationMap(command.path),options_,
                                                       readLocalizationKeyframes(command.path));
            sensor_msgs::msg::PointCloud2 map_msg; pcl::toROSMsg(*engine->map(),map_msg);
            map_msg.header.frame_id=map_frame_; map_msg.header.stamp=now(); map_pub_->publish(map_msg);
            {std::lock_guard<std::mutex> lock(mutex_); map_ready_=true;}
          } else if (command.kind==Kind::Seed) {seed=command; previous_odom.reset(); if (engine) engine->reset();}
          else if (command.kind==Kind::Reset) {if (engine) engine->reset(); seed.reset(); previous_odom.reset();}
          else if (command.kind==Kind::Relocalize) {if (engine) engine->relocalize();}
          else if (command.kind==Kind::GlobalRelocalize) {if(engine)engine->globalRelocalize();seed.reset();}
          else if (engine) engine->invalidate(command.reason.empty()?"stale_data":command.reason);
        }
        if (!engine) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (generation==generation_) {public_=LocalizationUpdate{}; label_="no_map";}
          next_frame=Steady::now()+std::chrono::milliseconds(200); continue;
        }
        if (cloud) {
          const double stamp=rclcpp::Time(cloud->header.stamp).seconds();
          if (now().seconds()-stamp > max_scan_age_) {engine->invalidate("stale_scan");}
          else {
            const auto odom_to_base=lookup(odom_frame_,base_frame_,cloud->header.stamp);
            if (previous_odom && (stamp<=previous_stamp ||
                (odom_to_base.translation()-previous_odom->translation()).norm()>odom_jump_+odom_speed_*(stamp-previous_stamp) ||
                rotationDistance(odom_to_base,*previous_odom)>.2+odom_rotation_*(stamp-previous_stamp))) {
              engine->reset(); seed.reset(); previous_odom.reset();
              throw std::invalid_argument("Odometry discontinuity; provide a new initial pose");
            }
            previous_odom=odom_to_base; previous_stamp=stamp;
            if (seed) {
              auto seed_odom=seed->stamp.sec==0 && seed->stamp.nanosec==0 ? odom_to_base : lookup(odom_frame_,base_frame_,seed->stamp);
              engine->initialize(seed->seed * seed_odom.inverse() * odom_to_base,odom_to_base); seed.reset();
            }
            if (engine->status().has_prior||options_.global.enabled) {
              auto scan=std::make_shared<LocalizationCloud>(); pcl::fromROSMsg(*cloud,*scan);
              LocalizationCloud body_scan;
              pcl::transformPointCloud(*scan,body_scan,lookup(base_frame_,cloud_frame_,cloud->header.stamp).matrix().cast<float>());
              {std::lock_guard<std::mutex> lock(mutex_);registration_running_=true;
                const auto &s=engine->status();
                const bool global_search=s.state==LocalizationState::GlobalRelocalizing||
                  (options_.global.enabled&&options_.global.on_lost&&options_.automatic_recovery&&s.state==LocalizationState::Lost&&
                   s.failures>=options_.failures_to_lost+options_.global.failures_before_global);
                if(global_search&&global_timing_state_!="running"){
                  global_started_=Steady::now();global_duration_ms_=0;global_timing_state_="running";
                }
              }
              engine->update(body_scan.makeShared(),odom_to_base);
              {std::lock_guard<std::mutex> lock(mutex_);registration_running_=false;}
              if (now().seconds()-stamp>max_result_age_) engine->invalidate("registration_result_too_old");
            }
          }
        }
        const auto state=engine->status();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (generation==generation_) {
            public_=state; label_=localizationStateName(state.state); reason_=state.match.reason;
            if(state.valid&&global_timing_state_=="running"){
              global_duration_ms_=std::chrono::duration<double,std::milli>(Steady::now()-global_started_).count();global_timing_state_="succeeded";
            }
            if (cloud && state.valid) valid_receipt_=Steady::now();
          }
        }
        const auto state_name=engine->status().state;
        const double interval=state_name==LocalizationState::GlobalRelocalizing&&cloud ? global_retry_interval_ :
          state_name==LocalizationState::Lost ? recovery_interval_ : 1.0/rate_;
        next_frame=Steady::now()+std::chrono::duration_cast<Steady::duration>(std::chrono::duration<double>(interval));
      } catch (const std::exception &e) {
        if (engine) engine->invalidate(e.what());
        std::lock_guard<std::mutex> lock(mutex_);
        registration_running_=false;
        if (generation==generation_) {public_.valid=false; if (engine) public_=engine->status();
          label_=engine ? localizationStateName(public_.state) : "map_error"; reason_=e.what();}
        next_frame=Steady::now()+std::chrono::milliseconds(200);
      }
    }
  }
bool MapLocalizer::freshLocked() const {
    return public_.valid && std::chrono::duration<double>(Steady::now()-valid_receipt_).count()<=timeout_ &&
        std::chrono::duration<double>(Steady::now()-input_receipt_).count()<=timeout_;
  }
}  // namespace dog_slam
