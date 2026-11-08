#include "dog_navigation/obstacle_memory.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include "voxel_key_hash.hpp"
#include <stdexcept>

namespace dog_navigation {
bool validPose(const Pose &p) {
  return p.matrix().allFinite() && p.translation().cwiseAbs().maxCoeff()<100000 &&
    (p.linear().transpose()*p.linear()).isApprox(Eigen::Matrix3d::Identity(),1e-6) &&
    std::abs(p.linear().determinant()-1)<1e-6 &&
    p.matrix().row(3).isApprox(Eigen::RowVector4d(0,0,0,1));
}
void Options::validate() const {
  for(double v:{voxel,range,vertical_range,min_range,padding,obstacle_bottom,static_tolerance,
                avoidance_distance,avoidance_weight,local_length})
    if(!std::isfinite(v)||v<=0) throw std::invalid_argument("Planner limits must be finite and positive");
  if(voxel<.05||voxel>.3||range>20||vertical_range>10||min_range>=range||padding>1||
      static_tolerance>voxel*2||obstacle_bottom>.3||max_rays<100||max_rays>50000||
      max_voxels<max_rays||max_voxels>500000||local_length>range)
    throw std::invalid_argument("Inconsistent planner limits");
}
ObstacleMemory::ObstacleMemory(Options o):options_(o) {o.validate();}
ObstacleMemory::Key ObstacleMemory::key(const Point &p) const {
  return {int(std::floor(p.x()/options_.voxel)),int(std::floor(p.y()/options_.voxel)),int(std::floor(p.z()/options_.voxel))};
}
void ObstacleMemory::clear() {cells_.clear();stamp_=0;}
Points ObstacleMemory::points() const {Points out;out.reserve(cells_.size());for(const auto &c:cells_)out.push_back(c.second);return out;}
void ObstacleMemory::update(const Point &sensor,const Points &returns,double stamp,const Point &body) {
  if(!sensor.allFinite()||!body.allFinite()||body.cwiseAbs().maxCoeff()>100000||
      sensor.cwiseAbs().maxCoeff()>100000||!std::isfinite(stamp)||stamp<=stamp_||stamp<=0)
    throw std::invalid_argument("Invalid or nonmonotonic scan/odometry");
  // Deduplicate rays, with occupied endpoints taking precedence over free ones.
  struct Ray {Point end;bool hit;};
  std::map<Key,Ray> rays;
  for(const auto &p:returns) {
    if(!p.allFinite()||p.cwiseAbs().maxCoeff()>100000) continue;
    const Point delta=p-sensor;const double distance=delta.norm();
    if(distance<options_.min_range) continue;
    const bool hit=distance<=options_.range;
    Point end=hit?p:Point(sensor+delta*(options_.range/distance));
    auto k=key(end);auto found=rays.find(k);
    if(found==rays.end()||hit) rays.insert_or_assign(k,Ray{end,hit});
    if(rays.size()>size_t(options_.max_rays)) throw std::invalid_argument("Scan exceeds ray budget");
  }
  if(rays.size()<3) throw std::invalid_argument("Insufficient finite scan returns");
  // Erasure order is irrelevant: occupied endpoints are still applied last,
  // and the committed cells remain ordered for reproducible output.
  std::unordered_set<Key,VoxelKeyHash> free;
  free.reserve(std::min<size_t>(rays.size()*16,1u<<20));
  std::map<Key,Point> hits;
  for(const auto &[k,ray]:rays) {
    const Point delta=ray.end-sensor;
    // Traverse every voxel intersected by the free part of the ray. Fixed
    // half-voxel samples can skip short intersections near a voxel corner,
    // leaving a removed obstacle permanently occupied across repeated scans.
    Key cell=key(sensor);
    Point crossing=Point::Constant(std::numeric_limits<double>::infinity());
    Point increment=crossing;Eigen::Vector3i direction=Eigen::Vector3i::Zero();
    for(int axis=0;axis<3;++axis)if(std::abs(delta[axis])>1e-12){
      direction[axis]=delta[axis]>0?1:-1;
      const double boundary=(cell[axis]+(direction[axis]>0?1:0))*options_.voxel;
      crossing[axis]=(boundary-sensor[axis])/delta[axis];
      increment[axis]=options_.voxel/std::abs(delta[axis]);
    }
    while(cell!=k){
      free.insert(cell);
      const double next=crossing.minCoeff();
      if(!std::isfinite(next)||next>1.+1e-12)break;
      // Simultaneous crossings touch the intervening side cells only at an
      // edge/corner; they do not constitute observed free volume.
      for(int axis=0;axis<3;++axis)if(crossing[axis]<=next+1e-12){
        cell[axis]+=direction[axis];crossing[axis]+=increment[axis];
      }
    }
    if(ray.hit) hits.insert_or_assign(k,ray.end);
  }
  auto next=cells_;
  for(const auto &k:free) next.erase(k);
  for(const auto &hit:hits) next.insert_or_assign(hit.first,hit.second);
  for(auto it=next.begin();it!=next.end();) {
    const Point delta=it->second-body;
    if(delta.head<2>().norm()>options_.range||std::abs(delta.z())>options_.vertical_range)it=next.erase(it);
    else ++it;
  }
  if(next.size()>size_t(options_.max_voxels)) throw std::invalid_argument("Local occupancy budget exceeded");
  cells_.swap(next);stamp_=stamp;
}

}  // namespace dog_navigation
