#include "local_navigation_node.hpp"
#include "dog_terrain/terrain_graph.hpp"

namespace dog_navigation {
void LocalNavigation::receiveGoal(geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        const auto &p=m->pose.position;const Point goal(p.x,p.y,p.z);
        try {
          if(m->header.frame_id!=map_frame_||!goal.allFinite())throw std::invalid_argument("Goal requires finite map ground XYZ");
          if(m->header.stamp.sec<0||m->header.stamp.nanosec>=1000000000u)throw std::invalid_argument("Invalid goal stamp");
          const double stamp=rclcpp::Time(m->header.stamp).seconds();
          if(stamp>0&&std::abs(now().seconds()-stamp)>timeout_)throw std::invalid_argument("Stale goal");
          graph_->snap(goal);goal_=goal;goal_stamp_=m->header.stamp;invalidate("new_goal_pending");
        }catch(const std::exception &e){goal_.reset();invalidate(e.what());}
}
void LocalNavigation::receiveLocalization(std_msgs::msg::String::ConstSharedPtr m) {
  const auto reason=gate_->updateJson(m->data,Clock::now());
  if(!reason.empty())invalidate(reason);
}
void LocalNavigation::receiveCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
        // Keep latest only; parsing, ray integration and planning run at a bounded rate.
        if(m->header.stamp.sec<0||m->header.stamp.nanosec>=1000000000u){scan_valid_=false;invalidate("invalid_scan_stamp");return;}
        const double stamp=rclcpp::Time(m->header.stamp).seconds();
        if(stamp<last_received_-.2){memory_->clear();goal_.reset();last_received_=0;scan_valid_=false;invalidate("clock_reset_requires_goal");}
        if(stamp<=last_received_)return;
        last_received_=stamp;pending_=std::move(m);
}
}  // namespace dog_navigation
