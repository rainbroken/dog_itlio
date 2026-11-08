// Copyright 2026 rainbroken
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <filesystem>
#include "itera_lio/loop/loop_verifier.hpp"
namespace itera_lio::loop {
struct LoopPairInput {
  LoopVerifierOptions options;
  LoopCandidate candidate;
  KeyframeCloud current;
  KeyframeCloud matched;
  std::vector<common::MapPoint> submap;
};
// A new directory is required. metadata.txt is committed last as a complete marker.
void saveLoopPair(const std::filesystem::path & directory,const LoopPairInput & input);
LoopPairInput loadLoopPair(const std::filesystem::path & directory);
}
