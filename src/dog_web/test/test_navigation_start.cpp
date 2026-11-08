#include "navigation_start.hpp"
#include <gtest/gtest.h>
using Json=nlohmann::json;
namespace {
Json state(){return {{"tf_ready",true},{"pose",{{"frame","map"},{"age_ms",20},{"position",{1.,2.,1.}},{"quaternion",{0.,0.,0.,1.}}}},
 {"localization",{{"fresh",true},{"valid",true},{"map_ready",true},{"map_path","/maps/a/localization.pcd"}}},
 {"navigation",{{"fresh",true},{"map_project","/maps/a"},{"map_revision","r"},{"base_ground_offset",.65}}}};}
}
TEST(NavigationStart, UsesConfiguredBodyOffsetAndOrientation){
 auto s=state();auto p=localizedPlanningStart(s,"/maps/a","r");EXPECT_NEAR(p.z(),.35,1e-9);
 const double angle=.2;s["pose"]["quaternion"]={std::sin(angle/2),0.,0.,std::cos(angle/2)};
 p=localizedPlanningStart(s,"/maps/a","r");EXPECT_NEAR(p.y(),2+.65*std::sin(angle),1e-9);EXPECT_NEAR(p.z(),1-.65*std::cos(angle),1e-9);
}
TEST(NavigationStart, RejectsStalePoseWrongProjectAndUnknownCalibration){
 auto s=state();s["pose"]["age_ms"]=301;EXPECT_THROW(localizedPlanningStart(s,"/maps/a","r"),std::invalid_argument);
 s=state();s["localization"]["valid"]=false;EXPECT_THROW(localizedPlanningStart(s,"/maps/a","r"),std::invalid_argument);
 s=state();s["localization"]["map_path"]="/other/a";EXPECT_THROW(localizedPlanningStart(s,"/maps/a","r"),std::invalid_argument);
 s=state();EXPECT_THROW(localizedPlanningStart(s,"/maps/a","old"),std::invalid_argument);
 s["navigation"]["base_ground_offset"]=-1;EXPECT_THROW(localizedPlanningStart(s,"/maps/a","r"),std::invalid_argument);
}
