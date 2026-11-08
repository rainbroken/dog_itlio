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

#ifndef ITERA_LIO__BACKEND__KEYFRAME_MANAGER_HPP_
#define ITERA_LIO__BACKEND__KEYFRAME_MANAGER_HPP_

#include <optional>

#include "itera_lio/backend/pose_graph.hpp"

namespace itera_lio::backend
{

struct KeyframeManagerOptions
{
  double min_translation{1.0};
  double min_rotation_rad{0.17453292519943295};
  double max_interval_sec{10.0};
};

class KeyframeManager
{
public:
  explicit KeyframeManager(const KeyframeManagerOptions & options = {});

  std::optional<Keyframe> addIfNeeded(double timestamp, const common::Pose & pose_WI);
  const std::optional<Keyframe> & lastKeyframe() const {return last_keyframe_;}

private:
  KeyframeManagerOptions options_;
  std::optional<Keyframe> last_keyframe_;
  KeyframeId next_id_{0U};
};

}  // namespace itera_lio::backend

#endif  // ITERA_LIO__BACKEND__KEYFRAME_MANAGER_HPP_
