#include "dog_terrain/terrain_graph.hpp"
#include "dog_terrain/collision_geometry.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_set>

namespace dog_terrain {
namespace {
constexpr double pi = 3.14159265358979323846;
Eigen::Vector3d xyz(const pcl::PointXYZ &p) { return {p.x, p.y, p.z}; }
}
bool TerrainGraph::supported(const Eigen::Vector3d &p) const {
  if (config_.body_length>0) {
    SupportContext context;
    for(int heading=0;heading<16;++heading)
      if(supportedAtYaw(p,heading*pi/16,&context))return true;
    return false;
  }
  return supportedAtYaw(p,0);
}

bool TerrainGraph::supportedAtYaw(const Eigen::Vector3d &p, double yaw,SupportContext *context) const {
  // Use observed points assigned to surface planes. A cell centroid can be
  // shifted away from a tread edge even when that edge has measured support.
  SupportContext local;auto &support=context?*context:local;
  for (int direction = 0; direction < (config_.body_length>0 ? 9 : 8); ++direction) {
    double angle = direction * pi / 4;
    Eigen::Vector3d offset(std::cos(angle)*config_.body_radius,std::sin(angle)*config_.body_radius,0);
    if(config_.body_length>0) {
      // Corners and edge midpoints of the actual footprint, rather than a circle.
      const double x=(direction==2 || direction==6) ? 0 : (direction<2 || direction>6 ? 1 : -1);
      const double y=(direction==0 || direction==4) ? 0 : (direction<4 ? 1 : -1);
      const double lx=x*config_.body_length*.5,ly=y*config_.body_width*.5;
      offset={std::cos(yaw)*lx-std::sin(yaw)*ly,std::sin(yaw)*lx+std::cos(yaw)*ly,0};
      if(direction==8)offset.setZero();
    }
    Eigen::Vector3d sample = p + offset;
    Cell key = cell(sample);
    bool found = false;
    std::vector<int> nearby;
    const double tolerance=config_.support_distance>0 ? config_.support_distance : config_.resolution*.5;
    const int reach=std::max(1,int(std::ceil(tolerance/config_.resolution)));
    for (int dx = -reach; dx <= reach && !found; ++dx) for (int dy = -reach; dy <= reach && !found; ++dy) {
      auto it = support_cells_.find({key.first + dx, key.second + dy});
      if (it == support_cells_.end()) continue;
      for (int id : it->second) {
        const Eigen::Vector3d point = xyz((*cloud_)[id]);
        if((point-sample).head<2>().norm()<=tolerance)nearby.push_back(id);
        if ((point - sample).head<2>().norm() <= tolerance &&
            std::abs(point.z() - p.z()) <= config_.max_step + .015) { found = true; break; }
      }
    }
    if(!found&&config_.terrain_tolerance>0) {
      if(!support.built) {
        const double radius=std::hypot(config_.body_length,config_.body_width)*.5+tolerance;
        support.terrain=localTerrain(p,p,{radius,radius,0});support.built=true;
      }
      for(int id:nearby) {
        const auto point=xyz((*cloud_)[id]);
        for(int surface:support.terrain) {
          const auto q=nodes_[surface].point;
          if((point-q).head<2>().norm()<=config_.resolution*.75&&
              std::abs(point.z()-q.z())<=config_.height_cluster){found=true;break;}
        }
        if(found)break;
      }
    }
    if (!found) return false;
  }
  return true;
}

std::vector<int> TerrainGraph::localTerrain(const Eigen::Vector3d &a,const Eigen::Vector3d &b,
                                           const Eigen::Vector3d &pad) const {
  const auto &c=config_;
  std::vector<int> terrain;
  if(c.terrain_tolerance==0)return terrain;
  const Cell low=cell(a.cwiseMin(b)-pad),high=cell(a.cwiseMax(b)+pad);
    // A tread underneath a long footprint is terrain, not a body obstacle just
    // because it is two risers above the footprint centre. Only locally
    // step-connected observed surfaces qualify; do not erase arbitrary planes.
    int seed=-1;double nearest=std::numeric_limits<double>::infinity();
    const Cell anchor=cell(a);
    for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy) {
      auto it=cells_.find({anchor.first+dx,anchor.second+dy});if(it==cells_.end())continue;
      for(int id:it->second) {
        const double distance=(nodes_[id].point-a).squaredNorm();
        if(distance<nearest){nearest=distance;seed=id;}
      }
    }
    const double link=c.connection_distance>0 ? c.connection_distance : c.resolution*1.5;
    const int reach=int(std::ceil(link/c.resolution));
    std::unordered_set<int> seen;
    if(seed>=0&&nearest<=c.resolution*c.resolution){terrain.push_back(seed);seen.insert(seed);}
    for(size_t i=0;i<terrain.size();++i) {
      const auto p=nodes_[terrain[i]].point;const Cell key=cell(p);
      for(int dx=-reach;dx<=reach;++dx)for(int dy=-reach;dy<=reach;++dy) {
        const Cell next(key.first+dx,key.second+dy);
        if(next.first<low.first-1||next.first>high.first+1||next.second<low.second-1||next.second>high.second+1)continue;
        auto it=cells_.find(next);if(it==cells_.end())continue;
        for(int id:it->second) {
          if(seen.count(id))continue;
          const auto q=nodes_[id].point;
          if(std::abs(q.z()-p.z())>c.max_step || (q-p).head<2>().norm()>link)continue;
          // Verify centre-line observations independently of footprint validity.
          bool observed=true;
          const int count=std::max(2,int(std::ceil((q-p).head<2>().norm()/.04)));
          const double tolerance=c.support_distance>0 ? c.support_distance : c.resolution*.5;
          const int bins=int(std::ceil(tolerance/c.resolution));
          for(int k=1;k<count&&observed;++k) {
            const Eigen::Vector3d sample=p+(q-p)*(double(k)/count);const Cell sk=cell(sample);
            bool found=false;
            for(int sx=-bins;sx<=bins&&!found;++sx)for(int sy=-bins;sy<=bins&&!found;++sy) {
              auto raw=support_cells_.find({sk.first+sx,sk.second+sy});if(raw==support_cells_.end())continue;
              for(int point_id:raw->second) {
                const auto point=xyz((*cloud_)[point_id]);
                if((point-sample).head<2>().norm()<=tolerance&&std::abs(point.z()-sample.z())<=c.max_step+.015){found=true;break;}
              }
            }
            observed=found;
          }
          if(observed){seen.insert(id);terrain.push_back(id);}
        }
      }
    }
  return terrain;
}

bool TerrainGraph::bodyClear(const Eigen::Vector3d &p) const {
  if(config_.body_length>0) {
    for(int heading=0;heading<16;++heading)
      if(rectangleClear(p,p,heading*pi/16))return true;
    return false;
  }
  return bodySweepClear(p,p);
}

bool TerrainGraph::rectangleClear(const Eigen::Vector3d &a,const Eigen::Vector3d &b,
                                  double yaw,double extra,Eigen::Vector3d *obstacle,RotationContext *context) const {
  const auto &c=config_;
  const double hx=c.body_length*.5+c.footprint_padding+extra;
  const double hy=c.body_width*.5+c.footprint_padding+extra;
  const double co=std::abs(std::cos(yaw)),si=std::abs(std::sin(yaw));
  const Eigen::Vector3d pad(co*hx+si*hy,si*hx+co*hy,0);
  const Cell low=cell(a.cwiseMin(b)-pad),high=cell(a.cwiseMax(b)+pad);
  std::vector<int> terrain;
  const std::vector<int> *local=&terrain;
  bool terrain_built=false;
  auto build_terrain=[&]() {
    terrain_built=true;
    if(context) {
      const std::array<int,4> key{low.first,low.second,high.first,high.second};
      auto it=context->terrain.find(key);
      if(it==context->terrain.end())it=context->terrain.emplace(key,localTerrain(a,b,pad)).first;
      local=&it->second;
    } else terrain=localTerrain(a,b,pad);
  };
  for(int x=low.first;x<=high.first;++x)for(int y=low.second;y<=high.second;++y) {
    auto it=raw_cells_.find({x,y});if(it==raw_cells_.end())continue;
    for(int id:it->second) {
      const auto point=xyz((*cloud_)[id]);
      if(sweptBoxContact(a,b,point,yaw,hx,hy,c.body_bottom+1e-5,c.body_top)) {
        if(!terrain_built)build_terrain();
        bool ground=false;
        for(int surface:*local) {
          const auto &node=nodes_[surface];
          // Apply the obstacle-height filter relative to the local tread,
          // rather than the centre's (possibly several steps lower) height.
          // Only sub-step protrusions are exempted; taller obstacles remain.
          if(point.z()<=node.point.z()+std::min(c.max_step,c.body_bottom)+c.terrain_tolerance &&
              (point-node.point).head<2>().norm()<=c.resolution*.75) {
            ground=true;break;
          }
        }
        if(ground)continue;
        if(obstacle)*obstacle=point;
        return false;
      }
    }
  }
  return true;
}

bool TerrainGraph::bodySweepClear(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                                 Eigen::Vector3d *obstacle, double *fraction) const {
  const auto &c = config_;
  if(c.body_length>0) {
    const auto delta=b-a;
    return rectangleClear(a,b,std::atan2(delta.y(),delta.x()),0,obstacle);
  }
  const double radius = c.body_radius+c.footprint_padding;
  const Eigen::Vector3d padding(radius,radius,0);
  Cell low = cell(a.cwiseMin(b)-padding), high = cell(a.cwiseMax(b)+padding);
  for (int x = low.first; x <= high.first; ++x) for (int y = low.second; y <= high.second; ++y) {
    auto it = raw_cells_.find({x, y});
    if (it == raw_cells_.end()) continue;
    for (int id : it->second) {
      const Eigen::Vector3d point = xyz((*cloud_)[id]);
      const auto contact = sweptCylinderContact(a,b,point,radius,c.body_bottom+1e-5,c.body_top);
      if (contact) {
        if (obstacle) *obstacle = point;
        if (fraction) *fraction = *contact;
        return false;
      }
    }
  }
  return true;
}

double TerrainGraph::obstacleClearance(const Eigen::Vector3d &p) const {
  // Circumscribed circle only supplies a conservative soft clearance cost.
  const double radius = (config_.body_length>0 ? std::hypot(config_.body_length,config_.body_width)*.5 :
      config_.body_radius)+config_.footprint_padding;
  const double reach = radius+config_.clearance_distance;
  const Eigen::Vector3d padding(reach,reach,0);
  const Cell low = cell(p-padding), high = cell(p+padding);
  double closest = reach;
  for (int x=low.first;x<=high.first;++x) for (int y=low.second;y<=high.second;++y) {
    const auto it = raw_cells_.find({x,y});
    if (it == raw_cells_.end()) continue;
    for (int id : it->second) {
      const Eigen::Vector3d delta = xyz((*cloud_)[id])-p;
      if (delta.z() > config_.body_bottom+1e-5 && delta.z() < config_.body_top)
        closest = std::min(closest,delta.head<2>().norm());
    }
  }
  return std::max(0.0,closest-radius);
}

bool TerrainGraph::cornerSupported(const Cell &key, const Eigen::Vector3d &a, const Eigen::Vector3d &b) const {
  auto it = cells_.find(key);
  if (it == cells_.end()) return false;
  for (int i : it->second)
    if (nodes_[i].valid && std::abs(nodes_[i].point.z() - a.z()) <= config_.max_step &&
        std::abs(nodes_[i].point.z() - b.z()) <= config_.max_step) return true;
  return false;
}

bool TerrainGraph::edgeClear(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const {
  // An analytic swept volume catches thin obstacles between sample poses.
  // Support still requires observed terrain samples, independently of body clearance.
  if (!bodySweepClear(a,b)) return false;
  const double yaw=std::atan2(b.y()-a.y(),b.x()-a.x());
  if(config_.body_length>0 && (!supportedAtYaw(a,yaw)||!supportedAtYaw(b,yaw)))return false;
  const int horizontal_samples=std::max(2,static_cast<int>(std::ceil(
      (b-a).head<2>().norm()/(config_.resolution/3))));
  const int spatial_samples=std::max(2,static_cast<int>(std::ceil(
      (b-a).norm()/(config_.resolution/3))));
  auto supported_samples=[&](int count) {
    for(int i=1;i<count;++i)
      if(!supportedAtYaw(a+(b-a)*(static_cast<double>(i)/count),yaw))return false;
    return true;
  };
  // Preserve the previous sample set as well. Simply changing the division
  // count could otherwise skip an old unsupported position on a different grid.
  if(!supported_samples(horizontal_samples))return false;
  if(spatial_samples!=horizontal_samples&&!supported_samples(spatial_samples))return false;
  return true;
}

Json TerrainGraph::inspectConnection(const Eigen::Vector3d &a,const Eigen::Vector3d &b) const {
  if(!a.allFinite()||!b.allFinite()||a.cwiseAbs().maxCoeff()>100000||b.cwiseAbs().maxCoeff()>100000)
    throw std::invalid_argument("Connection requires finite local coordinates");
  const double yaw=std::atan2(b.y()-a.y(),b.x()-a.x());
  Eigen::Vector3d obstacle;
  const bool clear=bodySweepClear(a,b,&obstacle);
  Json result={{"body_clear",clear},{"yaw",yaw},{"support_failures",Json::array()}};
  if(!clear)result["collision_point"]={obstacle.x(),obstacle.y(),obstacle.z()};
  const int count=std::max(2,int(std::ceil((b-a).norm()/(config_.resolution/3))));
  for(int i=0;i<=count;++i) {
    const Eigen::Vector3d p=a+(b-a)*(double(i)/count);
    if(!supportedAtYaw(p,yaw))result["support_failures"].push_back({p.x(),p.y(),p.z()});
  }
  result["edge_clear"]=edgeClear(a,b);
  return result;
}

bool TerrainGraph::turnClear(int previous,int current,int next) const {
  if(config_.body_length==0 || previous<0)return true;
  const auto &p=nodes_.at(current).point;
  const auto a=p-nodes_.at(previous).point,b=nodes_.at(next).point-p;
  const double first=std::atan2(a.y(),a.x()),last=std::atan2(b.y(),b.x());
  // A symmetric rectangle can drive in reverse: yaw is equivalent modulo pi.
  const double delta=std::remainder(last-first,pi);
  const int count=std::max(1,int(std::ceil(std::abs(delta)/(.025))));
  const double corner=std::hypot(config_.body_length*.5+config_.footprint_padding,
                                 config_.body_width*.5+config_.footprint_padding);
  // Inflate each sample by the maximum motion to its nearest sampled yaw.
  // This bounds the entire rotation's collision sweep, not just sampled poses.
  const double pad=2*corner*std::sin(std::abs(delta)/count/4);
  SupportContext support;
  RotationContext rotation;
  for(int i=0;i<=count;++i) {
    const double yaw=first+delta*double(i)/count;
    if(!rectangleClear(p,p,yaw,pad,nullptr,&rotation)||!supportedAtYaw(p,yaw,&support))return false;
  }
  return true;
}

bool TerrainGraph::motionClear(const Eigen::Vector3d &a,const Eigen::Vector3d &b,
                               double first,double last) const {
  if(!a.allFinite()||!b.allFinite()||!std::isfinite(first)||!std::isfinite(last))return false;
  const double angle=std::remainder(last-first,2*pi);
  const int count=std::max({1,int(std::ceil((b-a).norm()/.025)),int(std::ceil(std::abs(angle)/.025))});
  const double corner=config_.body_length>0?std::hypot(config_.body_length,config_.body_width)*.5+config_.footprint_padding:config_.body_radius;
  // Translation is swept analytically; only the rotation about the midpoint
  // heading needs inflation. This also covers vertical motion between samples.
  const double extra=2*corner*std::sin(std::abs(angle)/count/4);
  if(config_.body_length==0&& !bodySweepClear(a,b))return false;
  const bool stationary=(a.array()==b.array()).all();
  SupportContext support;RotationContext rotation;
  for(int i=0;i<=count;++i) {
    const auto p=a+(b-a)*(double(i)/count);const double yaw=first+angle*double(i)/count;
    if(!supportedAtYaw(p,yaw,stationary?&support:nullptr))return false;
    if(config_.body_length>0&&i<count) {
      const Eigen::Vector3d next=a+(b-a)*(double(i+1)/count);
      if(!rectangleClear(p,next,yaw+angle/count*.5,extra,nullptr,stationary?&rotation:nullptr))return false;
    }
  }
  return true;
}

Json TerrainGraph::inspectSurface(const Eigen::Vector3d &p) const {
  if (!p.allFinite() || p.cwiseAbs().maxCoeff()>100000)
    throw std::invalid_argument("Endpoint must be finite local XYZ");
  int nearest = -1;
  double distance = std::numeric_limits<double>::infinity();
  for (size_t i=0;i<nodes_.size();++i) {
    const auto delta = nodes_[i].point-p;
    if (delta.head<2>().norm()<=config_.snap_xy && std::abs(delta.z())<=config_.snap_z &&
        delta.squaredNorm()<distance) {nearest = static_cast<int>(i);distance=delta.squaredNorm();}
  }
  Json result={{"requested",{p.x(),p.y(),p.z()}},{"frame_id","map"},{"revision",revision_},
               {"collision_model",config_.body_length>0 ? "oriented_rectangle_height_filtered" : "continuous_vertical_cylinder"},
               {"body_length",config_.body_length},{"body_width",config_.body_width},
               {"collision_radius",(config_.body_length>0 ? std::hypot(config_.body_length,config_.body_width)*.5 :
                                         config_.body_radius)+config_.footprint_padding},
               {"body_bottom",config_.body_bottom},{"body_top",config_.body_top}};
  result["selection"]="nearest_observed_surface";
  result["planning_snap"]=nullptr;
  try {
    const int id=snap(p);const auto &snapped=nodes_[id];
    result["planning_snap"]={{"node_id",id},{"surface",{snapped.point.x(),snapped.point.y(),snapped.point.z()}},
                             {"component",snapped.component},{"distance",(snapped.point-p).norm()}};
  } catch (const std::invalid_argument &) {}
  if (nearest<0) {result["reason"]="no_observed_surface_at_height";return result;}
  const auto &node=nodes_[nearest];
  result["node_id"]=nearest;result["surface"]={node.point.x(),node.point.y(),node.point.z()};
  result["support_valid"]=node.support_valid;result["body_clear"]=node.body_clear;
  result["valid"]=node.valid;result["component"]=node.component;
  result["body_clearance"]=obstacleClearance(node.point);
  result["reason"]=node.valid ? "traversable_surface" :
      !node.support_valid ? "insufficient_observed_support" : !node.body_clear ? "body_collision" : "no_common_supported_clear_heading";
  Eigen::Vector3d witness;
  if (!node.body_clear && !bodySweepClear(node.point,node.point,&witness))
    result["collision_point"]={witness.x(),witness.y(),witness.z()};
  return result;
}

}  // namespace dog_terrain
