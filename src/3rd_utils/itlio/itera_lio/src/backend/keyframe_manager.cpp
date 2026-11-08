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

#include "itera_lio/backend/keyframe_manager.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "itera_lio/common/so3.hpp"

namespace itera_lio::backend
{

KeyframeManager::KeyframeManager(const KeyframeManagerOptions & options)
: options_(options)
{
  if (!std::isfinite(options_.min_translation) || options_.min_translation < 0.0 ||
    !std::isfinite(options_.min_rotation_rad) || options_.min_rotation_rad < 0.0 ||
    !std::isfinite(options_.max_interval_sec) || options_.max_interval_sec <= 0.0)
  {
    throw std::invalid_argument("invalid keyframe manager options");
  }
}

std::optional<Keyframe> KeyframeManager::addIfNeeded(
  const double timestamp, const common::Pose & pose_WI)
{
  if (!std::isfinite(timestamp) || !pose_WI.isValid()) {
    return std::nullopt;
  }
  if (last_keyframe_) {
    const common::Pose relative = relativePose(last_keyframe_->pose_WI, pose_WI);
    const double translation = relative.p_WI.norm();
    const double rotation = common::LogSO3(relative.R_WI).norm();
    const double interval = timestamp - last_keyframe_->timestamp;
    if (interval < 0.0 ||
      (translation < options_.min_translation && rotation < options_.min_rotation_rad &&
      interval < options_.max_interval_sec))
    {
      return std::nullopt;
    }
  }
  Keyframe keyframe{next_id_++, timestamp, pose_WI};
  last_keyframe_ = keyframe;
  return keyframe;
}

}  // namespace itera_lio::backend
