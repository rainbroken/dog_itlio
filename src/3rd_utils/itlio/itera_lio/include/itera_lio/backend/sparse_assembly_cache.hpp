// Copyright 2026 rainbroken
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/SparseCore>
#include <algorithm>
#include <vector>
#include <stdexcept>
namespace itera_lio::backend::detail {
// Per-solve cache: always validate the complete ordered triplet structure.
// Changes in topology/order rebuild safely. Duplicate sums retain input order.
class SparseAssemblyCache {
  using Matrix=Eigen::SparseMatrix<double>;
  using Index=Matrix::StorageIndex;
  struct Slot {Index row,column,offset;};
  std::vector<Slot> slots_;bool initialized_=false,offsets_ready_=false;
public:
  bool assemble(Matrix& matrix,const std::vector<Eigen::Triplet<double>>& triplets){
    bool same=initialized_&&slots_.size()==triplets.size();
    if(same)for(std::size_t k=0;k<triplets.size();++k)if(slots_[k].row!=triplets[k].row()||slots_[k].column!=triplets[k].col()){same=false;break;}
    if(same&&!offsets_ready_){
      for(auto& slot:slots_){
        const Index begin=matrix.outerIndexPtr()[slot.column],end=matrix.outerIndexPtr()[slot.column+1];
        const auto* found=std::lower_bound(matrix.innerIndexPtr()+begin,matrix.innerIndexPtr()+end,slot.row);
        if(found==matrix.innerIndexPtr()+end||*found!=slot.row){same=false;break;}
        slot.offset=static_cast<Index>(found-matrix.innerIndexPtr());
      }
      offsets_ready_=same;
    }
    if(same){
      std::fill(matrix.valuePtr(),matrix.valuePtr()+matrix.nonZeros(),0.);
      for(std::size_t k=0;k<triplets.size();++k)matrix.valuePtr()[slots_[k].offset]+=triplets[k].value();
      return true;
    }
    matrix.setFromTriplets(triplets.begin(),triplets.end());matrix.makeCompressed();
    slots_.clear();slots_.reserve(triplets.size());
    for(const auto& t:triplets)slots_.push_back({t.row(),t.col(),-1});
    initialized_=true;offsets_ready_=false;return false;
  }
};
// Preserve every ordered triplet for a safe rebuild on any topology/order
// change. Stable iterations validate and accumulate as entries are generated,
// avoiding two later full passes over the temporary triplet vector.
class StreamingSparseAssembly {
  using Matrix=Eigen::SparseMatrix<double>;
  using Index=Matrix::StorageIndex;
  struct Slot {Index row,column,offset;};
  Matrix* matrix_{nullptr};
  std::vector<Eigen::Triplet<double>> entries_;
  std::vector<Slot> slots_;
  SparseAssemblyCache reference_;
  std::size_t cursor_{0};
  Eigen::Index rows_{-1}, columns_{-1};
  bool ready_{false},fast_{false},enabled_{true},fixed_{false},lower_only_{false};
public:
  // fixed_pattern is valid only while the owner holds immutable topology.
  // In that mode, reject any structural change instead of silently reusing it.
  void begin(Matrix& matrix,bool enabled=true,bool fixed_pattern=false,bool lower_only=false){
    const bool same_owner=matrix_==&matrix;
    const bool same_mode=enabled_==enabled&&lower_only_==lower_only;
    matrix_=&matrix;cursor_=0;enabled_=enabled;fixed_=fixed_pattern;lower_only_=lower_only;
    fast_=enabled&&same_owner&&same_mode&&ready_&&matrix.rows()==rows_&&matrix.cols()==columns_;
    if(fast_)std::fill(matrix.valuePtr(),matrix.valuePtr()+matrix.nonZeros(),0.);
  }
  void reserve(std::size_t n){entries_.reserve(n);}
  void emplace_back(Index row,Index column,double value){
    if(lower_only_&&row<column)return;
    if(fast_&&fixed_){
      if(cursor_>=slots_.size()||slots_[cursor_].row!=row||slots_[cursor_].column!=column)
        throw std::logic_error("immutable sparse assembly pattern changed");
      matrix_->valuePtr()[slots_[cursor_++].offset]+=value;
      return;
    }
    if(cursor_<entries_.size())entries_[cursor_]=Eigen::Triplet<double>(row,column,value);
    else entries_.emplace_back(row,column,value);
    if(fast_){
      if(cursor_<slots_.size()&&slots_[cursor_].row==row&&slots_[cursor_].column==column)
        matrix_->valuePtr()[slots_[cursor_].offset]+=value;
      else fast_=false;
    }
    ++cursor_;
  }
  bool finish(){
    if(fast_&&fixed_){
      if(cursor_!=slots_.size())throw std::logic_error("immutable sparse assembly count changed");
      return true;
    }
    entries_.resize(cursor_);
    if(!enabled_)return reference_.assemble(*matrix_,entries_);
    if(fast_&&cursor_==slots_.size())return true;
    matrix_->setFromTriplets(entries_.begin(),entries_.end());matrix_->makeCompressed();
    slots_.clear();slots_.reserve(entries_.size());
    for(const auto& t:entries_){
      const auto begin=matrix_->outerIndexPtr()[t.col()],end=matrix_->outerIndexPtr()[t.col()+1];
      const auto* found=std::lower_bound(matrix_->innerIndexPtr()+begin,matrix_->innerIndexPtr()+end,t.row());
      slots_.push_back({t.row(),t.col(),static_cast<Index>(found-matrix_->innerIndexPtr())});
    }
    rows_=matrix_->rows();columns_=matrix_->cols();ready_=true;
    return false;
  }
};
}
