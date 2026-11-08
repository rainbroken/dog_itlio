#pragma once
#include <filesystem>
#include <nlohmann/json.hpp>

// Only saved files beneath the configured SLAM root can be imported by ID.
class PcdCatalog {
public:
  explicit PcdCatalog(std::filesystem::path root):root_(std::filesystem::weakly_canonical(std::move(root))){}
  std::filesystem::path resolve(const std::string &id) const;
  nlohmann::json list() const;
  const std::filesystem::path &root() const {return root_;}
private:
  std::filesystem::path root_;
};
