#include "dog_navigation/navigation_motion_settings.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace dog_navigation;
TEST(NavigationMotionSettings, PersistsValidatedProfileWithoutPermissionFlags){
  const auto root=std::filesystem::temp_directory_path()/("navigation-motion-"+std::to_string(getpid()));
  NavigationMotionSettings settings(root/"profile.json");
  auto values=NavigationMotionSettings::values(ExecutionOptions{});
  values["walk_speed"]=.4;values["acceleration"]=.1;
  settings.save(values);EXPECT_DOUBLE_EQ(settings.load().walk_speed,.4);EXPECT_DOUBLE_EQ(settings.load().acceleration,.1);
  auto bad=values;bad["enabled"]=true;EXPECT_THROW(settings.save(bad),std::invalid_argument);
  bad=values;bad["stair_speed"]=.5;EXPECT_THROW(settings.save(bad),std::invalid_argument);
  bad=values;bad["stair_speed"]=.08;EXPECT_THROW(settings.save(bad),std::invalid_argument);
  bad=values;bad["yaw_rate"]=std::numeric_limits<double>::infinity();EXPECT_THROW(settings.save(bad),std::invalid_argument);
  EXPECT_DOUBLE_EQ(settings.load().walk_speed,.4);std::filesystem::remove_all(root);
}
