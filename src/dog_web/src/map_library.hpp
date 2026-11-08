#pragma once
#include "pcd_catalog.hpp"
#include <mutex>

// Display names never change artifact IDs, hashes, or localization paths.
// Removal atomically moves one project/file to a hidden recovery directory.
class MapLibrary {
public:
  MapLibrary(std::filesystem::path maps,PcdCatalog &pcds);
  nlohmann::json projects(const std::string &loaded) const;
  nlohmann::json savedPcds() const;
  std::string displayName(const std::string &kind,const std::string &id) const;
  nlohmann::json rename(const std::string &kind,const std::string &id,const std::string &name);
  nlohmann::json remove(const std::string &kind,const std::string &id);
private:
  std::filesystem::path resolve(const std::string &kind,const std::string &id) const;
  std::string label(const std::string &kind,const std::string &id) const;
  void persist(const nlohmann::json &names) const;
  std::filesystem::path maps_;
  PcdCatalog &pcds_;
  nlohmann::json names_;
  mutable std::mutex mutex_;
};
