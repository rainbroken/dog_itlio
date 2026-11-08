#include "cloud_preview.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}}
CloudPreview::CloudPreview(rclcpp::Node &node,tf2_ros::Buffer &tf):node_(node),tf_(tf) {
  const char *topics[]={"/lio/global_map","/interfaces/lidar"};
  for(int i=0;i<2;++i)subscriptions_[i]=node.create_subscription<sensor_msgs::msg::PointCloud2>(topics[i],
    rclcpp::SensorDataQoS().keep_last(1),[this,i](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,const rclcpp::MessageInfo &info){
      const auto &gid=info.get_rmw_message_info().publisher_gid;
      const std::string source(reinterpret_cast<const char *>(gid.data),sizeof(gid.data));
      {std::lock_guard<std::mutex> lock(mutex_);
        if(!i&&!blocked_source_.empty()&&source==blocked_source_)return;
        pending_[i]=std::move(msg);pending_sources_[i]=source;}condition_.notify_one();
    });
  worker_=std::thread([this]{run();});
}
CloudPreview::~CloudPreview(){
  {std::lock_guard<std::mutex> lock(mutex_);stop_=true;}condition_.notify_one();worker_.join();
}
void CloudPreview::resetMap(){
  std::lock_guard<std::mutex> lock(mutex_);blocked_source_=pending_sources_[0];
  pending_[0].reset();packets_[0].reset();++reset_generation_;
}
CloudPacket CloudPreview::encode(const sensor_msgs::msg::PointCloud2 &m,size_t limit){
  CloudPacket result;result.frame=m.header.frame_id;result.stamp=rclcpp::Time(m.header.stamp).seconds();
  result.received=steady();
  int offsets[4]={-1,-1,-1,-1};uint8_t intensity_type=0;
  for(const auto &f:m.fields)for(int j=0;j<4;++j){
    const auto name=j==3?"intensity":std::string(1,"xyz"[j]);
    if(f.name==name&&f.count==1&&(f.datatype==7||(j==3&&f.datatype==2))){offsets[j]=f.offset;if(j==3)intensity_type=f.datatype;}
  }
  if(m.is_bigendian||!m.point_step||!m.width||!m.height||!limit)return result;
  for(int j=0;j<3;++j)if(offsets[j]<0||uint64_t(offsets[j])+4>m.point_step)return result;
  if(uint64_t(m.row_step)<uint64_t(m.width)*m.point_step||uint64_t(m.row_step)*m.height>m.data.size())return result;
  result.intensity=offsets[3]>=0&&uint64_t(offsets[3])+(intensity_type==2?1:4)<=m.point_step;
  const size_t count=size_t(m.width)*m.height,stride=std::max(size_t(1),(count+limit-1)/limit);
  result.bytes.reserve(std::min(count,limit)*16);std::vector<float> scalars;
  for(size_t i=0;i<count;i+=stride){
    const size_t base=(i/m.width)*m.row_step+(i%m.width)*m.point_step;float p[4]={0,0,0,0};
    for(int j=0;j<3;++j)std::memcpy(&p[j],m.data.data()+base+offsets[j],4);
    if(!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2]))continue;
    if(result.intensity){if(intensity_type==2)p[3]=m.data[base+offsets[3]];else std::memcpy(&p[3],m.data.data()+base+offsets[3],4);}
    else p[3]=p[2];
    if(!std::isfinite(p[3]))p[3]=0;
    scalars.push_back(p[3]);result.bytes.append(reinterpret_cast<const char *>(p),16);
  }
  if(!scalars.empty()){auto range=std::minmax_element(scalars.begin(),scalars.end());result.low=*range.first;result.high=*range.second;}
  return result;
}
void CloudPreview::run(){
  while(true){
    sensor_msgs::msg::PointCloud2::ConstSharedPtr messages[2];std::string sources[2];uint64_t reset_generation;
    {std::unique_lock<std::mutex> lock(mutex_);condition_.wait(lock,[this]{return stop_||pending_[0]||pending_[1];});
      if(stop_)return;
      reset_generation=reset_generation_;
      for(int i=0;i<2;++i){messages[i]=std::move(pending_[i]);sources[i]=pending_sources_[i];}}
    for(int i=0;i<2;++i)if(messages[i]){
      auto packet=std::make_shared<CloudPacket>(encode(*messages[i],i?12000:1500000));packet->sequence=++sequence_[i];
      if(!i){
        const auto previous=this->packet(false);
        if(source_!=sources[i]||epoch_.empty()||(previous&&(packet->frame!=previous->frame||packet->stamp<previous->stamp))){
          source_=sources[i];epoch_=std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
        }
        packet->epoch=epoch_;packet->tiles=buildCloudTiles(packet->bytes);packet->tiles_version=cloudTilesVersion(packet->tiles);
      }
      std::lock_guard<std::mutex> lock(mutex_);if(i||reset_generation==reset_generation_)packets_[i]=std::move(packet);
    }
  }
}
std::shared_ptr<const CloudPacket> CloudPreview::packet(bool scan) const {std::lock_guard<std::mutex> lock(mutex_);return packets_[scan?1:0];}
nlohmann::json CloudPreview::metadata(const CloudPacket &p) const {
  const auto age=steady()-p.received;
  nlohmann::json result={{"sequence",p.sequence},{"frame",p.frame},{"stamp",p.stamp},{"age_ms",age*1000},
    {"fresh",age<1.5&&std::abs(node_.now().seconds()-p.stamp)<1.5},{"count",p.bytes.size()/16},
    {"field",p.intensity?"intensity":"z"},{"range",{p.low,p.high}},{"transform",nullptr}};
  try{
    if(p.frame=="map")result["transform"]={{"position",{0,0,0}},{"quaternion",{0,0,0,1}}};
    else {
      const auto t=tf_.lookupTransform("map",p.frame,rclcpp::Time(int64_t(p.stamp*1e9)));
      const auto &v=t.transform.translation;const auto &q=t.transform.rotation;
      result["transform"]={{"position",{v.x,v.y,v.z}},{"quaternion",{q.x,q.y,q.z,q.w}}};
    }
  }catch(const tf2::TransformException &){}
  return result;
}
