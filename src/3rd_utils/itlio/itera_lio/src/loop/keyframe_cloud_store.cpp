// Copyright 2026 rainbroken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "itera_lio/loop/keyframe_cloud_store.hpp"

#include <chrono>
#include <array>
#include <fstream>
#include <limits>
#include <system_error>

namespace itera_lio::loop
{
namespace
{
constexpr std::uint32_t kArchiveMagic = 0x4B46434CU;
// Preserve the existing x,y,z,intensity float stream, independent of Eigen's
// alignment/padding. One stream operation per payload replaces four per point.
using PackedPoint = std::array<float, 4>;
static_assert(sizeof(PackedPoint) == 4U * sizeof(float));

template<typename T>
bool writeValue(std::ostream & stream, const T & value)
{
  stream.write(reinterpret_cast<const char *>(&value), sizeof(T));
  return static_cast<bool>(stream);
}

template<typename T>
bool readValue(std::istream & stream, T & value)
{
  stream.read(reinterpret_cast<char *>(&value), sizeof(T));
  return static_cast<bool>(stream);
}
}  // namespace

KeyframeCloudStore::KeyframeCloudStore(
  const std::size_t max_resident_clouds, const std::string & archive_directory)
: max_resident_clouds_(max_resident_clouds)
{
  if (max_resident_clouds_ == 0U) {return;}
  const auto root = archive_directory.empty() ?
    std::filesystem::temp_directory_path() :
    std::filesystem::absolute(std::filesystem::path(archive_directory));
  const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
  archive_directory_ = root / (".itera_lio_keyframes_" + std::to_string(suffix));
  std::filesystem::create_directories(archive_directory_);
}

KeyframeCloudStore::~KeyframeCloudStore()
{
  if (!archive_directory_.empty()) {
    std::error_code error;
    std::filesystem::remove_all(archive_directory_, error);
  }
}

std::filesystem::path KeyframeCloudStore::archivePath(const common::KeyframeId id) const
{
  return archive_directory_ / (std::to_string(id) + ".bin");
}

bool KeyframeCloudStore::archive(const common::KeyframeId id)
{
  if (id >= clouds_.size() || clouds_[id].points_I.empty() || archive_directory_.empty()) {
    return false;
  }
  const auto path = archivePath(id);
  const auto temporary = path.string() + ".tmp";
  std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
  const std::uint64_t count = static_cast<std::uint64_t>(clouds_[id].points_I.size());
  if (!stream || !writeValue(stream, kArchiveMagic) || !writeValue(stream, id) ||
    !writeValue(stream, count))
  {
    ++stats_.archive_errors;
    return false;
  }
  std::vector<PackedPoint> payload;
  payload.reserve(clouds_[id].points_I.size());
  for (const auto & point : clouds_[id].points_I) {
    payload.push_back({point.position.x(), point.position.y(), point.position.z(), point.intensity});
  }
  stream.write(reinterpret_cast<const char *>(payload.data()),
    static_cast<std::streamsize>(payload.size() * sizeof(PackedPoint)));
  if (!stream) {
    ++stats_.archive_errors;
    return false;
  }
  stream.flush();
  stream.close();
  if (!stream) {
    ++stats_.archive_errors;
    return false;
  }
  std::error_code error;
  std::filesystem::rename(temporary, path, error);
  if (error) {
    std::filesystem::remove(temporary, error);
    ++stats_.archive_errors;
    return false;
  }
  clouds_[id].points_I.clear();
  clouds_[id].points_I.shrink_to_fit();
  ++stats_.archive_writes;
  return true;
}

bool KeyframeCloudStore::load(const common::KeyframeId id, KeyframeCloud & cloud) const
{
  std::ifstream stream(archivePath(id), std::ios::binary);
  std::uint32_t magic = 0U;
  common::KeyframeId stored_id = 0U;
  std::uint64_t count = 0U;
  if (!stream || !readValue(stream, magic) || magic != kArchiveMagic ||
    !readValue(stream, stored_id) || stored_id != id || !readValue(stream, count))
  {
    ++stats_.archive_errors;
    return false;
  }
  // Reject truncated/overflowed counts before allocating; these are temporary
  // local files, but an interrupted write must not trigger an unbounded resize.
  const auto payload_start = stream.tellg();
  stream.seekg(0, std::ios::end);
  const auto payload_end = stream.tellg();
  if (payload_start < 0 || payload_end < payload_start ||
    count > static_cast<std::uint64_t>(std::numeric_limits<std::streamsize>::max()) / sizeof(PackedPoint) ||
    count > static_cast<std::uint64_t>(payload_end - payload_start) / sizeof(PackedPoint))
  {
    ++stats_.archive_errors;
    return false;
  }
  stream.seekg(payload_start);
  std::vector<PackedPoint> payload(static_cast<std::size_t>(count));
  stream.read(reinterpret_cast<char *>(payload.data()),
    static_cast<std::streamsize>(payload.size() * sizeof(PackedPoint)));
  if (!stream) {
    ++stats_.archive_errors;
    return false;
  }
  cloud = clouds_[id];
  cloud.points_I.resize(payload.size());
  for (std::size_t i = 0; i < payload.size(); ++i) {
    cloud.points_I[i] = {
      Eigen::Vector3f(payload[i][0], payload[i][1], payload[i][2]), payload[i][3]};
  }
  ++stats_.archive_reads;
  return cloud.isValid();
}

bool KeyframeCloudStore::add(KeyframeCloud cloud)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!cloud.isValid() || cloud.keyframe.id != clouds_.size()) {return false;}
  if (max_resident_clouds_ > 0U && resident_order_.size() >= max_resident_clouds_) {
    if (!archive(resident_order_.front())) {return false;}
    resident_order_.pop_front();
  }
  resident_order_.push_back(cloud.keyframe.id);
  clouds_.push_back(std::move(cloud));
  return true;
}

const KeyframeCloud * KeyframeCloudStore::find(const common::KeyframeId id) const
{
  if (id >= clouds_.size() || clouds_[id].keyframe.id != id) {return nullptr;}
  return &clouds_[id];
}

bool KeyframeCloudStore::get(const common::KeyframeId id, KeyframeCloud & cloud) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (id >= clouds_.size() || clouds_[id].keyframe.id != id) {return false;}
  if (!clouds_[id].points_I.empty()) {
    cloud = clouds_[id];
    return true;
  }
  return load(id, cloud);
}

KeyframeCloudStoreStats KeyframeCloudStore::stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = stats_;
  result.resident_clouds = resident_order_.size();
  result.archived_clouds = clouds_.size() - resident_order_.size();
  return result;
}

void KeyframeCloudStore::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  clouds_.clear();
  resident_order_.clear();
  stats_ = {};
  if (!archive_directory_.empty()) {
    std::error_code error;
    std::filesystem::remove_all(archive_directory_, error);
    std::filesystem::create_directories(archive_directory_, error);
  }
}

}  // namespace itera_lio::loop
