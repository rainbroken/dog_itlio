// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ITERA_LIO__BACKEND__POSE_GRAPH_HPP_
#define ITERA_LIO__BACKEND__POSE_GRAPH_HPP_

#include <Eigen/Core>

#include <cstddef>
#include <vector>

#include "itera_lio/common/geometry.hpp"
#include "itera_lio/common/ids.hpp"

namespace itera_lio::backend
{

using PoseInformationMatrix = Eigen::Matrix<double, 6, 6>;
using KeyframeId = common::KeyframeId;

enum class ConstraintType
{
  kOdometry,
  kLoop
};

struct Keyframe
{
  KeyframeId id{0U};
  double timestamp{0.0};
  common::Pose pose_WI;

  bool isValid() const;
};

// T_Ia_Ib maps coordinates from keyframe b's IMU frame into keyframe a's IMU frame.
struct PoseGraphConstraint
{
  KeyframeId from_id{0U};
  KeyframeId to_id{0U};
  common::Pose T_Ia_Ib;
  PoseInformationMatrix information{PoseInformationMatrix::Identity()};
  ConstraintType type{ConstraintType::kOdometry};

  bool isValid() const;
};

common::Pose composePoses(const common::Pose & T_A_B, const common::Pose & T_B_C);
common::Pose inversePose(const common::Pose & T_A_B);
common::Pose relativePose(const common::Pose & T_W_A, const common::Pose & T_W_B);

// Forward measurement information uses [left rotation, additive translation].
// Transport it to the graph residual of the inverse measurement; inversion
// couples rotation and translation and cannot keep the same matrix unchanged.
PoseInformationMatrix inverseMeasurementInformation(
  const common::Pose & measurement, const PoseInformationMatrix & information);

class PoseGraph
{
public:
  bool addNode(const Keyframe & keyframe);
  bool addConstraint(const PoseGraphConstraint & constraint);

  const std::vector<Keyframe> & nodes() const {return nodes_;}
  const std::vector<PoseGraphConstraint> & constraints() const {return constraints_;}
  bool hasLoopConstraint() const {return loop_constraint_count_ > 0U;}
  std::size_t loopConstraintCount() const {return loop_constraint_count_;}

private:
  std::vector<Keyframe> nodes_;
  std::vector<PoseGraphConstraint> constraints_;
  std::size_t loop_constraint_count_{0U};
};

}  // namespace itera_lio::backend

#endif  // ITERA_LIO__BACKEND__POSE_GRAPH_HPP_
