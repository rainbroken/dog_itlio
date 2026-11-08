#include "dog_terrain/terrain_graph.hpp"

namespace dog_terrain {
Cloud::Ptr switchbackCloud() {
  auto cloud = std::make_shared<Cloud>();
  auto plane = [&](double x0, double x1, double y0, double y1, double z) {
    for (double x = x0 + .012; x < x1; x += .035)
      for (double y = y0 + .012; y < y1; y += .035)
        cloud->push_back(pcl::PointXYZ(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
  };
  plane(-2, 0, -.6, 2.1, 0); plane(-2, 0, -.6, 2.1, 2.4);
  for (int step = 0; step < 8; ++step) {
    plane(step * .3, (step + 1) * .3, -.6, .6, (step + 1) * .15);
    plane(step * .3, (step + 1) * .3, .9, 2.1, 2.4 - step * .15);
  }
  plane(2.4, 3.4, -.6, 2.1, 1.2);
  return cloud;
}
}  // namespace dog_terrain
