// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef ITERA_LIO__LOOP__KEYFRAME_CLOUD_STORE_HPP_
#define ITERA_LIO__LOOP__KEYFRAME_CLOUD_STORE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

#include "itera_lio/loop/loop_types.hpp"

namespace itera_lio::loop
{

struct KeyframeCloudStoreStats
{
  std::size_t resident_clouds{0U};
  std::size_t archived_clouds{0U};
  std::uint64_t archive_writes{0U};
  std::uint64_t archive_reads{0U};
  std::uint64_t archive_errors{0U};
};

class KeyframeCloudStore
{
public:
  explicit KeyframeCloudStore(
    std::size_t max_resident_clouds = 0U,
    const std::string & archive_directory = {});
  ~KeyframeCloudStore();

  KeyframeCloudStore(const KeyframeCloudStore &) = delete;
  KeyframeCloudStore & operator=(const KeyframeCloudStore &) = delete;

  bool add(KeyframeCloud cloud);
  const KeyframeCloud * find(common::KeyframeId id) const;
  bool get(common::KeyframeId id, KeyframeCloud & cloud) const;
  const std::vector<KeyframeCloud> & clouds() const {return clouds_;}
  std::size_t size() const {return clouds_.size();}
  KeyframeCloudStoreStats stats() const;
  void clear();

private:
  // Serialize map-output reads with insertion and resident-cloud eviction.
  mutable std::mutex mutex_;
  bool archive(common::KeyframeId id);
  bool load(common::KeyframeId id, KeyframeCloud & cloud) const;
  std::filesystem::path archivePath(common::KeyframeId id) const;

  std::vector<KeyframeCloud> clouds_;
  std::size_t max_resident_clouds_{0U};
  std::filesystem::path archive_directory_;
  std::deque<common::KeyframeId> resident_order_;
  mutable KeyframeCloudStoreStats stats_;
};

}  // namespace itera_lio::loop

#endif  // ITERA_LIO__LOOP__KEYFRAME_CLOUD_STORE_HPP_
