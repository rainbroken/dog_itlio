#include "local_navigation_node.hpp"
#include "../common/navigation_ros.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

namespace dog_navigation {
Pose LocalNavigation::lookup(const std::string &parent,const std::string &child,const builtin_interfaces::msg::Time &stamp) {
    if(parent==child)return Pose::Identity();
    return transform(buffer_->lookupTransform(parent,child,rclcpp::Time(stamp),rclcpp::Duration::from_seconds(.03)).transform);
  }
void LocalNavigation::tick() {
    const auto begin=Clock::now();
    try {
      if(!gate_->ready(Clock::now())){invalidate("localization_unavailable");return;}
      if(pending_) {
        auto m=std::move(pending_);pending_.reset();
        const double stamp=rclcpp::Time(m->header.stamp).seconds(),age=now().seconds()-stamp;
        if(m->header.frame_id!=sensor_frame_||stamp<=0||age<-.1||age>timeout_||m->is_bigendian||
            m->height==0||uint64_t(m->width)*m->height>500000||
            m->row_step<uint64_t(m->width)*m->point_step||m->data.size()<uint64_t(m->row_step)*m->height)
          throw std::invalid_argument("Invalid scan frame/layout/freshness");
        for(const char *name:{"x","y","z"}) {
          auto field=std::find_if(m->fields.begin(),m->fields.end(),[&](const auto &f){return f.name==name;});
          if(field==m->fields.end()||field->datatype!=7||field->count!=1||field->offset+sizeof(float)>m->point_step)
            throw std::invalid_argument("Scan requires FLOAT32 XYZ");
        }
        const auto body=lookup(odom_frame_,base_frame_,m->header.stamp);
        if(last_body_&&((body.translation()-last_body_->translation()).norm()>jump_+speed_*(stamp-last_scan_stamp_)||
            Eigen::AngleAxisd(last_body_->linear().transpose()*body.linear()).angle()>.2+4*(stamp-last_scan_stamp_))) {
          memory_->clear();last_body_.reset();goal_.reset();throw std::invalid_argument("Odometry reset; provide new goal");
        }
        const auto sensor=lookup(odom_frame_,sensor_frame_,m->header.stamp);
        pcl::PointCloud<pcl::PointXYZ> cloud;pcl::fromROSMsg(*m,cloud);Points points;points.reserve(cloud.size());
        for(const auto &p:cloud)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))
          points.push_back(sensor*Point(p.x,p.y,p.z));
        memory_->update(sensor.translation(),points,stamp,body.translation());scan_valid_=true;
        last_body_=body;last_scan_stamp_=stamp;scan_header_=m->header;
      }
      const double age=now().seconds()-memory_->stamp();
      if(!scan_valid_||memory_->stamp()<=0||age<-.1||age>timeout_) {invalidate("scan_unavailable");return;}
      const auto odom=buffer_->lookupTransform(odom_frame_,base_frame_,tf2::TimePointZero);
      const double tf_age=now().seconds()-rclcpp::Time(odom.header.stamp).seconds();
      if(tf_age<-.1||tf_age>tf_timeout_)throw std::invalid_argument("Odometry TF stale");
      const auto map_tf=buffer_->lookupTransform(map_frame_,odom_frame_,tf2::TimePointZero);
      const double map_age=now().seconds()-rclcpp::Time(map_tf.header.stamp).seconds();
      if(map_age<-.1||map_age>tf_timeout_)throw std::invalid_argument("Map TF stale");
      tf_expiry_=std::min(rclcpp::Time(odom.header.stamp).seconds(),rclcpp::Time(map_tf.header.stamp).seconds())+tf_timeout_;
      if(!goal_){invalidate("waiting_goal");return;}
      const Pose map_from_odom=transform(map_tf.transform);
      const Pose map_from_body=map_from_odom*transform(odom.transform);
      // Offset is a body-fixed vector, retaining full roll/pitch on slopes.
      const Point ground=map_from_body*Point(0,0,-ground_offset_);
      auto route=planner_->plan(ground,*goal_,memory_->points(),map_from_odom);
      duration_ms_=elapsed(begin)*1000;
      if(elapsed(begin)>budget_||now().seconds()-memory_->stamp()>timeout_||
          !gate_->ready(Clock::now())||now().seconds()>tf_expiry_){invalidate("processing_or_data_expired");return;}
      ++generation_;
      if(!route.valid){invalidate(route.reason,route);return;}
      // The executor confirms metric XYZ/yaw and measured stop, not graph-node identity.
      // Paths contain terrain contact positions, not body poses or velocity commands.
      route_stamp_=now();
      path_pub_->publish(path(route.points,route_stamp_));local_pub_->publish(path(route.local,route_stamp_));
      pcl::PointCloud<pcl::PointXYZ> obstacles;for(const auto &p:route.obstacles_map)obstacles.push_back(pcl::PointXYZ(p.x(),p.y(),p.z()));
      sensor_msgs::msg::PointCloud2 obs;pcl::toROSMsg(obstacles,obs);obs.header=scan_header_;obs.header.frame_id=map_frame_;
      obstacle_pub_->publish(obs);status(route.ids.size()==1?"goal_reached":"route_ready",true,route);
    }catch(const std::exception &e){scan_valid_=false;invalidate(e.what());}
  }
}  // namespace dog_navigation
