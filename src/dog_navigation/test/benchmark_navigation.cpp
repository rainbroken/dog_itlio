#include "benchmark_support.hpp"
#include "dog_navigation/local_planner.hpp"
#include "dog_terrain/terrain_graph.hpp"
#include <fstream>
#include <iostream>
#if defined(__linux__)
#include <sys/resource.h>
#endif

using namespace dog_navigation;
using namespace dog_navigation::benchmark;
int main(int argc, char **argv) {
  try {
    if (argc != 5)
      throw std::invalid_argument(
          "benchmark_navigation MAP_ROOT MAP_ID ROUTE_JSON OUTPUT_JSON");
    std::ifstream input(argv[3]);
    const Json document = Json::parse(input);
    const auto &route = document.at("route");
    Points points;
    for (const auto &p : route.at("points"))
      points.emplace_back(p[0], p[1], p[2]);
    std::vector<bool> stairs(points.size() - 1, false);
    for (const auto &section : route.at("sections"))
      if (section.at("mode") == "stairs")
        for (size_t i = section.at("start_index");
             i < section.at("end_index").get<size_t>(); ++i)
          stairs[i] = true;
    Json report = {
        {"map_id", argv[2]},
        {"model", "offline kernels; no hardware commands"},
        {"percentile_note", "Observed quantiles of small samples or batch "
                            "means; no WCET or optimality guarantee"}};
    std::shared_ptr<dog_terrain::TerrainGraph> graph;
    std::vector<double> loads, setup, cold, warm;
    Json outcomes = Json::array();
    auto stages = [](const PlanningPerformance &p) {
      return Json{{"total_ms", p.total_ms},
                  {"classification_ms", p.classification_ms},
                  {"search_ms", p.search_ms},
                  {"certification_ms", p.certification_ms},
                  {"turn_evaluations", p.turn_evaluations},
                  {"turn_cache_hits", p.turn_cache_hits},
                  {"turn_cache_entries", p.turn_cache_entries},
                  {"lower_bound_builds", p.lower_bound_builds}};
    };
    for (int i = 0; i < 3; ++i) {
      auto start = Clock::now();
      graph = dog_terrain::loadProject(argv[1], argv[2]);
      loads.push_back(milliseconds(start));
      start=Clock::now();LocalPlanner planner(graph);setup.push_back(milliseconds(start));
      start = Clock::now();
      const auto first =
          planner.plan(points.front(), points.back(), {}, Pose::Identity());
      cold.push_back(milliseconds(start));
      auto first_stages = stages(planner.performance());
      start = Clock::now();
      const auto second =
          planner.plan(points.front(), points.back(), {}, Pose::Identity());
      warm.push_back(milliseconds(start));
      outcomes.push_back({{"cold_valid", first.valid},
                          {"cold_reason", first.reason},
                          {"warm_valid", second.valid},
                          {"warm_reason", second.reason},
                          {"cold_stages", first_stages},
                          {"warm_stages", stages(planner.performance())}});
      std::cerr << "planning run " << i + 1 << ": " << cold.back() << " / "
                << warm.back() << " ms\n";
    }
    report["revision"] = graph->revision();
    report["map_load"] = distribution(loads, "milliseconds");
    report["planner_setup"] = distribution(setup, "milliseconds");
    report["cold_planning"] = distribution(cold, "milliseconds");
    report["warm_planning"] = distribution(warm, "milliseconds");
    report["planning_outcomes"] = outcomes;
    report["map_points"] = graph->cloud().size();
    report["map_nodes"] = graph->nodes().size();
    report["motion_certificate"] = kernel(
        [&] {
          int valid = 0;
          for (size_t i = 1; i < points.size(); ++i) {
            const auto d = points[i] - points[i - 1];
            const double yaw = std::atan2(d.y(), d.x());
            valid += graph->motionClear(points[i - 1], points[i], yaw, yaw);
          }
          return valid;
        },
        15, 1);
    report["controls"] = controlKernels(points, stairs);
    report["perception"] = perceptionKernels();
    report["successful_planning"] = successfulPlanningKernels();
#if defined(__linux__)
    rusage usage{};if(getrusage(RUSAGE_SELF,&usage)==0)report["peak_process_rss_kib"]=usage.ru_maxrss;
#endif
    std::ofstream output(argv[4]);
    output << report.dump(2) << '\n';
    if (!output)
      throw std::runtime_error("Cannot save benchmark report");
    std::cout << "Saved " << argv[4] << '\n';
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
