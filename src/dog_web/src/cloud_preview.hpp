#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <nlohmann/json.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>
#include "cloud_tiles.hpp"

struct CloudPacket {
  std::string bytes, frame;
  double stamp=0, received=0;
  uint64_t sequence=0;
  bool intensity=false;
  float low=0,high=1;
  std::string epoch,tiles_version;
  CloudTiles tiles;
};
// ROS callbacks only replace shared pointers. Sampling and encoding never hold
// the robot control lock, and storage is bounded independently of map size.
class CloudPreview {
public:
  CloudPreview(rclcpp::Node &node,tf2_ros::Buffer &tf);
  ~CloudPreview();
  std::shared_ptr<const CloudPacket> packet(bool scan) const;
  nlohmann::json metadata(const CloudPacket &packet) const;
  static CloudPacket encode(const sensor_msgs::msg::PointCloud2 &,size_t limit);
  void resetMap();
private:
  void run();
  rclcpp::Node &node_; tf2_ros::Buffer &tf_;
  mutable std::mutex mutex_;std::condition_variable condition_;bool stop_=false;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_[2];
  std::shared_ptr<const CloudPacket> packets_[2];
  uint64_t sequence_[2]={0,0};
  std::string pending_sources_[2],source_,epoch_;
  std::string blocked_source_;
  uint64_t reset_generation_=0;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriptions_[2];
  std::thread worker_;
};
