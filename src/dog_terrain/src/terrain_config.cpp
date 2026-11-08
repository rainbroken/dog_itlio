#include "dog_terrain/terrain_graph.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace dog_terrain {
Json Config::json() const {
  return {{"resolution", resolution}, {"height_cluster", height_cluster}, {"min_points", min_points},
          {"max_roughness", max_roughness}, {"max_slope_deg", max_slope_deg}, {"max_step", max_step},
          {"min_step", min_step}, {"body_radius", body_radius}, {"body_bottom", body_bottom},
          {"body_length", body_length}, {"body_width", body_width},
          {"normal_radius", normal_radius}, {"normal_thickness", normal_thickness},
          {"normal_fallback_radius", normal_fallback_radius},
          {"plane_distance", plane_distance}, {"support_distance", support_distance},
          {"connection_distance", connection_distance},
          {"terrain_tolerance", terrain_tolerance},
          {"body_top", body_top}, {"footprint_padding", footprint_padding}, {"snap_xy", snap_xy}, {"snap_z", snap_z},
          {"step_penalty", step_penalty}, {"max_tread_gap", max_tread_gap}, {"max_nodes", max_nodes},
          {"clearance_weight", clearance_weight}, {"clearance_distance", clearance_distance}};
}

Config Config::parse(const Json &value) {
  if (!value.is_object()) throw std::invalid_argument("Terrain config must be an object");
  Config c;
  auto number = [&](const char *name, auto &target) {
    if (value.contains(name)) {
      if (!value.at(name).is_number()) throw std::invalid_argument(std::string(name) + " must be numeric");
      target = value.at(name).get<std::decay_t<decltype(target)>>();
    }
  };
  for (auto it = value.begin(); it != value.end(); ++it)
    if (!c.json().contains(it.key())) throw std::invalid_argument("Unknown terrain parameter: " + it.key());
  number("resolution", c.resolution); number("height_cluster", c.height_cluster);
  number("max_roughness", c.max_roughness); number("max_slope_deg", c.max_slope_deg);
  number("max_step", c.max_step); number("min_step", c.min_step);
  number("body_radius", c.body_radius); number("body_bottom", c.body_bottom); number("body_top", c.body_top);
  number("body_length", c.body_length); number("body_width", c.body_width);
  number("normal_radius", c.normal_radius); number("normal_thickness", c.normal_thickness);
  number("normal_fallback_radius", c.normal_fallback_radius);
  number("plane_distance", c.plane_distance); number("support_distance", c.support_distance);
  number("connection_distance", c.connection_distance);
  number("terrain_tolerance", c.terrain_tolerance);
  number("footprint_padding", c.footprint_padding);
  number("snap_xy", c.snap_xy); number("snap_z", c.snap_z);
  number("step_penalty", c.step_penalty); number("max_tread_gap", c.max_tread_gap);
  number("clearance_weight", c.clearance_weight); number("clearance_distance", c.clearance_distance);
  for (const char *key : {"min_points", "max_nodes"})
    if (value.contains(key) && !value.at(key).is_number_integer())
      throw std::invalid_argument(std::string(key) + " must be an integer");
  number("min_points", c.min_points); number("max_nodes", c.max_nodes);
  c.validate();
  return c;
}

void Config::validate() const {
  const auto values = json();
  for (auto it = values.begin(); it != values.end(); ++it) {
    double v = it.value().get<double>();
    const bool may_be_zero = it.key() == "clearance_weight" || it.key() == "footprint_padding" ||
        it.key() == "body_length" || it.key() == "body_width" || it.key() == "support_distance" ||
        it.key() == "connection_distance" || it.key() == "terrain_tolerance" || it.key()=="normal_fallback_radius";
    if (!std::isfinite(v) || (may_be_zero ? v < 0 : v <= 0))
      throw std::invalid_argument(it.key() + " must be finite and positive (weights/padding may be zero)");
  }
  if (resolution < .03 || resolution > 1 || body_radius > 2 || max_nodes > 500000 || min_points < 3 ||
      (body_length == 0) != (body_width == 0) || body_length > 4 || body_width > 4 ||
      normal_radius > .3 || normal_thickness > .05 || plane_distance > .05 || support_distance > .15 ||
      normal_fallback_radius > .3 || (normal_fallback_radius>0&&normal_fallback_radius<=normal_radius) ||
      connection_distance > .5 || (connection_distance>0 && body_length==0) ||
      terrain_tolerance > .05 || (terrain_tolerance>0 && body_length==0) ||
      footprint_padding > 1 || height_cluster >= min_step || min_step >= max_step || body_bottom >= body_top || max_slope_deg >= 80)
    throw std::invalid_argument("Invalid resolution, body geometry, point/node limits or height thresholds");
}

}  // namespace dog_terrain
