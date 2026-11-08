#include "dog_terrain/terrain_graph.hpp"
#include <fstream>
#include <iostream>

int main(int argc, char **argv) {
  namespace fs = std::filesystem;
  using namespace dog_terrain;
  const bool has_config = argc >= 3 && std::string(argv[argc - 2]) == "--config";
  const int arguments = argc - (has_config ? 2 : 0);
  if (arguments != 4 && arguments != 10) {
    std::cerr << "Usage: rebuild_map MAP_ROOT SOURCE_ID OUTPUT_ID [sx sy sz gx gy gz] [--config JSON_FILE]\n";
    return 1;
  }
  try {
    const fs::path root = fs::absolute(argv[1]);
    const auto source_path = mapPath(root, argv[2]);
    const auto output_path = mapPath(root, argv[3]);
    if (fs::exists(output_path)) throw std::invalid_argument("Output map already exists; choose a new ID");
    auto original = loadProject(root, argv[2]);
    auto config = original->config().json();
    Json overrides = Json::object();
    if (has_config) {
      std::ifstream settings(argv[argc - 1]);
      if (!settings) throw std::invalid_argument("Cannot open terrain config");
      overrides = Json::parse(settings);
      if (!overrides.is_object()) throw std::invalid_argument("Terrain config must be an object");
      Config::parse(overrides);
      config.update(overrides);
    }
    Eigen::Vector3d start = Eigen::Vector3d::Zero(), goal = Eigen::Vector3d::Zero();
    if (arguments == 10) {
      start = {std::stod(argv[4]), std::stod(argv[5]), std::stod(argv[6])};
      goal = {std::stod(argv[7]), std::stod(argv[8]), std::stod(argv[9])};
      if (!start.allFinite() || !goal.allFinite()) throw std::invalid_argument("Endpoints must be finite");
    }
    TerrainGraph graph(original->cloud().makeShared(), Config::parse(config));
    std::ifstream file(source_path / "map.json");
    const auto metadata = Json::parse(file);
    auto source = metadata.value("source", Json::object());
    source["rebuild"] = {{"map_id", argv[2]}, {"revision", original->revision()},
                         {"cloud_sha256", metadata.at("files").at("cloud.pcd")},
                         {"builder", "observed_surface_support_v5"},
                         {"config_overrides", overrides}};
    const auto keyframes = metadata.at("files").contains("localization_keyframes.csv") ?
        source_path / "localization_keyframes.csv" : fs::path{};
    saveProject(root, argv[3], graph, source, keyframes, source_path/"localization.pcd");
    auto result = graph.summary();
    result["map_id"] = argv[3]; result["path"] = output_path.string();
    if (arguments == 10) {
      result["start_surface"]=graph.inspectSurface(start);
      result["goal_surface"]=graph.inspectSurface(goal);
      try { result["route"] = graph.plan(start, goal); }
      catch (const std::invalid_argument &error) {
        result["route_error"] = error.what();
        std::cout << result.dump(2) << '\n'; return 2;
      }
    }
    std::cout << result.dump(2) << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
