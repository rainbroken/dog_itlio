#include "motion_settings.hpp"
#include <gtest/gtest.h>
#include <unistd.h>

TEST(MotionSettings, PersistsAcrossStartupAndRejectsInvalidValues) {
  auto root=std::filesystem::temp_directory_path()/ ("dog-motion-"+std::to_string(getpid()));
  std::filesystem::remove_all(root);
  auto path=root/"motion.json";
  MotionSettings settings(path);
  EXPECT_DOUBLE_EQ(settings.limits()[0],.3);
  settings.save({{"x",1.},{"y",.3},{"yaw",1.}});
  MotionSettings restored(path);
  EXPECT_EQ(restored.limits(),(std::array<double,3>{1.,.3,1.}));
  for(auto invalid : {nlohmann::json{{"x",1.01},{"y",.3},{"yaw",1.}},
      nlohmann::json{{"x",.5},{"y",-.1},{"yaw",1.}},
      nlohmann::json{{"x",.5},{"y",.31},{"yaw",1.}},
      nlohmann::json{{"x",.5},{"y",.3},{"yaw",1.01}},
      nlohmann::json{{"x","1"},{"y",.3},{"yaw",1.}}})
    EXPECT_THROW(settings.save(invalid),std::invalid_argument);
  EXPECT_EQ(MotionSettings(path).limits(),restored.limits());
  settings.save({{"x",0},{"y",0},{"yaw",0}});
  EXPECT_EQ(MotionSettings(path).limits(),(std::array<double,3>{0,0,0}));
  std::filesystem::remove(path);std::filesystem::create_directory(path);
  EXPECT_THROW(settings.save({{"x",.5},{"y",.1},{"yaw",.5}}),std::filesystem::filesystem_error);
  EXPECT_EQ(settings.limits(),(std::array<double,3>{0,0,0}));
  std::filesystem::remove_all(root);
}
