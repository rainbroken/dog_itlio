#pragma once
#include "dog_navigation/execution_types.hpp"

namespace dog_navigation {
// Stop-to-stop trapezoidal profiles on certified terrain edges. No shortcut,
// smoothing or invented ground. These are nominal profiles, retimed by
// feedback.
std::vector<TimedSample>
parameterizeTerrainPath(const Points &path, const std::vector<bool> &stairs,
                        const ExecutionOptions &options,
                        double sample_period = .1);
} // namespace dog_navigation
