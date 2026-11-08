// Copyright 2026 rainbroken
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>
#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#define ITERA_MANUAL_X86 1
#else
#define ITERA_MANUAL_X86 0
#endif
#ifndef ITERA_MANUAL_AVX_LEAVES
#define ITERA_MANUAL_AVX_LEAVES 0
#endif
#ifndef ITERA_MANUAL_SIMD_KNN
#define ITERA_MANUAL_SIMD_KNN 0
#endif
#ifndef ITERA_MANUAL_SIMD_LEAVES
#define ITERA_MANUAL_SIMD_LEAVES 0
#endif
#ifndef ITERA_MANUAL_CACHED_BOUNDS
#define ITERA_MANUAL_CACHED_BOUNDS 0
#endif
#ifndef ITERA_MANUAL_FLOAT_BOUNDS
#define ITERA_MANUAL_FLOAT_BOUNDS 0
#endif
#ifndef ITERA_MANUAL_LEAF_SIZE
#define ITERA_MANUAL_LEAF_SIZE 16
#endif
namespace itera_lio::loop::detail {
// Immutable, balanced 3D index. Query scratch lives on the call stack.
// Squared distances use the same float arithmetic as PCL's L2_Simple metric.
class ExactKdTree {
public:
  using Point=std::array<float,3>;
  void build(std::vector<Point> points) {
    for(const auto& p:points)for(float v:p)if(!std::isfinite(v))throw std::invalid_argument("nonfinite KD-tree point");
    points_=std::move(points);order_.resize(points_.size());std::iota(order_.begin(),order_.end(),0);
    nodes_.clear();nodes_.reserve(points_.size()/8+1);if(!points_.empty())buildNode(0,points_.size());
#if ITERA_MANUAL_CACHED_BOUNDS
    leaf_points_.resize(order_.size());for(std::size_t k=0;k<order_.size();++k)leaf_points_[k]=points_[order_[k]];
#if ITERA_MANUAL_SIMD_LEAVES
    for(int a=0;a<3;++a){coordinates_[a].resize(order_.size());for(std::size_t k=0;k<order_.size();++k)coordinates_[a][k]=leaf_points_[k][a];}
#endif
#endif
  }
  int nearest(const Point& query,double maximum_squared,float* squared=nullptr,int hint=-1) const {
    if(!(maximum_squared>=0))return -1;
    for(float v:query)if(!std::isfinite(v))return -1;
    int best=-1;double bound=maximum_squared;
    if(hint>=0 && static_cast<std::size_t>(hint)<points_.size()){
      const auto& p=points_[hint];const float dx=query[0]-p[0],dy=query[1]-p[1],dz=query[2]-p[2];
      const float d=dx*dx+dy*dy+dz*dz;if(d<=bound){best=hint;bound=d;}
    }
    if(!nodes_.empty()){
#if ITERA_MANUAL_CACHED_BOUNDS
      search(0,query,bound,best,lowerBound(nodes_[0],query));
#else
      search(0,query,bound,best);
#endif
    }
    if(squared)*squared=best<0?std::numeric_limits<float>::infinity():static_cast<float>(bound);
    return best;
  }
  int nearestK(const Point& q,int k,std::vector<int>& ids,std::vector<float>& distances)const {
    ids.clear();distances.clear();if(k<1)return 0;
    for(float v:q)if(!std::isfinite(v))return 0;
    ids.reserve(k);distances.reserve(k);
    if(!nodes_.empty())searchK(0,q,k,ids,distances,lowerBound(nodes_[0],q));
    return static_cast<int>(ids.size());
  }

private:
  struct Node {Point lower,upper;std::size_t begin,end;int left=-1,right=-1;};
  std::array<std::vector<float>,3> coordinates_;
  std::vector<Point> leaf_points_;
  std::vector<Point> points_;std::vector<int> order_;std::vector<Node> nodes_;
  int buildNode(std::size_t begin,std::size_t end) {
    Node node;node.begin=begin;node.end=end;node.lower=points_[order_[begin]];node.upper=node.lower;
    for(auto i=begin+1;i<end;++i)for(int a=0;a<3;++a){node.lower[a]=std::min(node.lower[a],points_[order_[i]][a]);node.upper[a]=std::max(node.upper[a],points_[order_[i]][a]);}
    const int id=static_cast<int>(nodes_.size());nodes_.push_back(node);
    if(end-begin<=ITERA_MANUAL_LEAF_SIZE)return id;
    int axis=0;for(int a=1;a<3;++a)if(node.upper[a]-node.lower[a]>node.upper[axis]-node.lower[axis])axis=a;
    const auto middle=begin+(end-begin)/2;
    std::nth_element(order_.begin()+begin,order_.begin()+middle,order_.begin()+end,[&](int a,int b){return points_[a][axis]<points_[b][axis] || (points_[a][axis]==points_[b][axis]&&a<b);});
    const int left=buildNode(begin,middle),right=buildNode(middle,end);nodes_[id].left=left;nodes_[id].right=right;return id;
  }
  static double lowerBound(const Node& n,const Point& q) {
#if ITERA_MANUAL_FLOAT_BOUNDS
    const float x=std::max(0.f,std::max(n.lower[0]-q[0],q[0]-n.upper[0]));
    const float y=std::max(0.f,std::max(n.lower[1]-q[1],q[1]-n.upper[1]));
    const float z=std::max(0.f,std::max(n.lower[2]-q[2],q[2]-n.upper[2]));
    return x*x+y*y+z*z;
#else
    double d=0;for(int a=0;a<3;++a){const double x=q[a]<n.lower[a]?double(n.lower[a])-q[a]:(q[a]>n.upper[a]?double(q[a])-n.upper[a]:0.);d+=x*x;}return d;
#endif
  }
  static bool prunable(double lower,double bound) {
    // Conservative allowance for float distance rounding. Equal-distance leaves
    // stay searchable so the lowest original point index breaks ties.
    return lower>bound+8.*std::numeric_limits<float>::epsilon()*std::max(1.,bound);
  }
  void searchK(int id,const Point& q,int k,std::vector<int>& ids,std::vector<float>& ds,double lower)const {
    if(static_cast<int>(ids.size())==k && prunable(lower,ds.back()))return;
    const auto& node=nodes_[id];
    if(node.left<0){
      const auto insert=[&](float d,int i){
        int n=static_cast<int>(ids.size());
        if(n==k && (d>ds.back() || (d==ds.back()&&i>=ids.back())))return;
        int at=n;while(at>0 && (d<ds[at-1] || (d==ds[at-1]&&i<ids[at-1])))--at;
        if(n<k){ids.push_back(i);ds.push_back(d);++n;}
        for(int z=n-1;z>at;--z){ids[z]=ids[z-1];ds[z]=ds[z-1];}ids[at]=i;ds[at]=d;
      };
      auto j=node.begin;
#if ITERA_MANUAL_SIMD_KNN && ITERA_MANUAL_SIMD_LEAVES && ITERA_MANUAL_X86 && ITERA_MANUAL_CACHED_BOUNDS
      const __m128 x=_mm_set1_ps(q[0]),y=_mm_set1_ps(q[1]),z=_mm_set1_ps(q[2]);
      for(;j+4<=node.end;j+=4){
        const __m128 dx=_mm_sub_ps(x,_mm_loadu_ps(coordinates_[0].data()+j));
        const __m128 dy=_mm_sub_ps(y,_mm_loadu_ps(coordinates_[1].data()+j));
        const __m128 dz=_mm_sub_ps(z,_mm_loadu_ps(coordinates_[2].data()+j));
        const __m128 d=_mm_add_ps(_mm_add_ps(_mm_mul_ps(dx,dx),_mm_mul_ps(dy,dy)),_mm_mul_ps(dz,dz));
        int mask=static_cast<int>(ids.size())<k?15:_mm_movemask_ps(_mm_cmple_ps(d,_mm_set1_ps(ds.back())));
        if(!mask)continue;
        alignas(16)float values[4];_mm_store_ps(values,d);
        while(mask){const int a=__builtin_ctz(static_cast<unsigned>(mask));mask&=mask-1;insert(values[a],order_[j+a]);}
      }
#endif
      for(;j<node.end;++j){const int i=order_[j];
#if ITERA_MANUAL_CACHED_BOUNDS
        const auto& p=leaf_points_[j];
#else
        const auto& p=points_[i];
#endif
        const float dx=q[0]-p[0],dy=q[1]-p[1],dz=q[2]-p[2];insert(dx*dx+dy*dy+dz*dz,i);
      }return;
    }
    const double dl=lowerBound(nodes_[node.left],q),dr=lowerBound(nodes_[node.right],q);
    if(dl<=dr){searchK(node.left,q,k,ids,ds,dl);searchK(node.right,q,k,ids,ds,dr);}
    else{searchK(node.right,q,k,ids,ds,dr);searchK(node.left,q,k,ids,ds,dl);}
  }

#if ITERA_MANUAL_AVX_LEAVES && ITERA_MANUAL_X86
  __attribute__((target("avx2")))
  void searchLeafAvx(const Node& node,const Point& q,double& bound,int& best)const {
    auto k=node.begin;
    const __m256 x=_mm256_set1_ps(q[0]),y=_mm256_set1_ps(q[1]),z=_mm256_set1_ps(q[2]);
    for(;k+8<=node.end;k+=8){
      const __m256 dx=_mm256_sub_ps(x,_mm256_loadu_ps(coordinates_[0].data()+k));
      const __m256 dy=_mm256_sub_ps(y,_mm256_loadu_ps(coordinates_[1].data()+k));
      const __m256 dz=_mm256_sub_ps(z,_mm256_loadu_ps(coordinates_[2].data()+k));
      const __m256 d=_mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(dx,dx),_mm256_mul_ps(dy,dy)),_mm256_mul_ps(dz,dz));
      int mask=_mm256_movemask_ps(_mm256_cmp_ps(d,_mm256_set1_ps(static_cast<float>(bound)),_CMP_LE_OQ));
      if(!mask)continue;
      alignas(32)float ds[8];_mm256_store_ps(ds,d);
      while(mask){const int a=__builtin_ctz(static_cast<unsigned>(mask));mask&=mask-1;const int i=order_[k+a];
        if(ds[a]<bound || (ds[a]==bound&&(best<0||i<best))){bound=ds[a];best=i;}}
    }
    for(;k<node.end;++k){const auto& p=leaf_points_[k];const int i=order_[k];
      const float dx=q[0]-p[0],dy=q[1]-p[1],dz=q[2]-p[2];const float d=dx*dx+dy*dy+dz*dz;
      if(d<bound||(d==bound&&(best<0||i<best))){bound=d;best=i;}}
  }
#endif

#if ITERA_MANUAL_CACHED_BOUNDS
  void search(int id,const Point& q,double& bound,int& best,double lower)const {
    const auto& node=nodes_[id];if(prunable(lower,bound))return;
    if(node.left<0){
#if ITERA_MANUAL_AVX_LEAVES && ITERA_MANUAL_X86 && ITERA_MANUAL_SIMD_LEAVES
      if(__builtin_cpu_supports("avx2")){searchLeafAvx(node,q,bound,best);return;}
#endif
      auto k=node.begin;
#if ITERA_MANUAL_SIMD_LEAVES && ITERA_MANUAL_X86
      const __m128 x=_mm_set1_ps(q[0]),y=_mm_set1_ps(q[1]),z=_mm_set1_ps(q[2]);
      for(;k+4<=node.end;k+=4){
        const __m128 dx=_mm_sub_ps(x,_mm_loadu_ps(coordinates_[0].data()+k));
        const __m128 dy=_mm_sub_ps(y,_mm_loadu_ps(coordinates_[1].data()+k));
        const __m128 dz=_mm_sub_ps(z,_mm_loadu_ps(coordinates_[2].data()+k));
        const __m128 d=_mm_add_ps(_mm_add_ps(_mm_mul_ps(dx,dx),_mm_mul_ps(dy,dy)),_mm_mul_ps(dz,dz));
        int mask=_mm_movemask_ps(_mm_cmple_ps(d,_mm_set1_ps(static_cast<float>(bound))));
        if(!mask)continue;
        alignas(16)float ds[4];_mm_store_ps(ds,d);
        while(mask){const int a=__builtin_ctz(static_cast<unsigned>(mask));mask&=mask-1;const int i=order_[k+a];
          if(ds[a]<bound || (ds[a]==bound&&(best<0||i<best))){bound=ds[a];best=i;}}
      }
#endif
      for(;k<node.end;++k){const int i=order_[k];const auto& p=leaf_points_[k];
        const float dx=q[0]-p[0],dy=q[1]-p[1],dz=q[2]-p[2];const float distance=dx*dx+dy*dy+dz*dz;
        if(distance<bound || (distance==bound&&(best<0||i<best))){bound=distance;best=i;}}
      return;
    }
    const double dl=lowerBound(nodes_[node.left],q),dr=lowerBound(nodes_[node.right],q);
    const int first=dl<=dr?node.left:node.right,second=dl<=dr?node.right:node.left;
    search(first,q,bound,best,dl<=dr?dl:dr);search(second,q,bound,best,dl<=dr?dr:dl);
  }
#else
  void search(int id,const Point& q,double& bound,int& best)const {
    const auto& node=nodes_[id];if(prunable(lowerBound(node,q),bound))return;
    if(node.left<0){
      for(auto k=node.begin;k<node.end;++k){const int i=order_[k];const auto& p=points_[i];
        const float dx=q[0]-p[0],dy=q[1]-p[1],dz=q[2]-p[2];const float distance=dx*dx+dy*dy+dz*dz;
        if(distance<bound || (distance==bound&&(best<0||i<best))){bound=distance;best=i;}}
      return;
    }
    const double dl=lowerBound(nodes_[node.left],q),dr=lowerBound(nodes_[node.right],q);
    const int first=dl<=dr?node.left:node.right,second=dl<=dr?node.right:node.left;
    search(first,q,bound,best);search(second,q,bound,best);
  }
#endif
};
}
