#include <gtest/gtest.h>
#include "itera_lio/loop/exact_kdtree.hpp"
#include <random>
#include <thread>
using Tree=itera_lio::loop::detail::ExactKdTree;
namespace {
std::pair<int,float> brute(const std::vector<Tree::Point>& points,const Tree::Point& q,double radius2){
 int index=-1;double best=radius2;
 for(std::size_t i=0;i<points.size();++i){float d=0;for(int a=0;a<3;++a){const float v=q[a]-points[i][a];d+=v*v;}
  if(d<best || (d==best&&index<0)){best=d;index=static_cast<int>(i);}}
 return {index,static_cast<float>(best)};
}
}
TEST(ExactKdTree, MatchesExhaustiveSearchForBoundedAndUnboundedQueries){
 std::mt19937 random(20261024);std::uniform_real_distribution<float> distribution(-100,100);
 std::vector<Tree::Point> points;for(int i=0;i<6000;++i)points.push_back({distribution(random),distribution(random),distribution(random)});
 Tree tree;tree.build(points);
 for(int i=0;i<2000;++i){Tree::Point q{distribution(random),distribution(random),distribution(random)};const double bound=i%3==0?2.25:(i%3==1?400.:1.e8);
  const auto expected=brute(points,q,bound);float d;const int index=tree.nearest(q,bound,&d);ASSERT_EQ(index,expected.first)<<i;
  if(index>=0)EXPECT_FLOAT_EQ(d,expected.second);}
}
TEST(ExactKdTree, HandlesBoundaryTiesDuplicatesAndEmptyIndex){
 Tree tree;EXPECT_EQ(tree.nearest({0,0,0},1),-1);
 std::vector<Tree::Point> points{{1,0,0},{-1,0,0},{1,0,0}};
 for(int i=0;i<100;++i)points.push_back({100.f+i,0,0});tree.build(points);
 EXPECT_EQ(tree.nearest({0,0,0},1.),0);EXPECT_EQ(tree.nearest({0,0,0},std::nextafter(1.,0.)),-1);
 EXPECT_EQ(tree.nearest({1,0,0},0.),0);tree.build({});EXPECT_EQ(tree.nearest({0,0,0},1),-1);
}
TEST(ExactKdTree, ArbitraryHintsPreserveExactDistancesIndicesAndBoundaryTies){
 std::mt19937 random(1515);std::uniform_real_distribution<float> d(-10,10);
 std::vector<Tree::Point> points{{1,0,0},{-1,0,0},{1,0,0}};
 for(int i=0;i<4096;++i)points.push_back({d(random),d(random),d(random)});
 Tree tree;tree.build(points);
 for(int i=0;i<1000;++i){
  Tree::Point q{d(random),d(random),d(random)};const double bound=i%2?4.:1000.;
  float expected_distance;int expected=tree.nearest(q,bound,&expected_distance);
  for(int hint:{-1,0,expected,static_cast<int>(points.size()/2),static_cast<int>(points.size()+5)}){
   float actual_distance;EXPECT_EQ(tree.nearest(q,bound,&actual_distance,hint),expected);
   EXPECT_EQ(actual_distance,expected_distance);
  }
 }
 Tree ties;ties.build({{1,0,0},{-1,0,0},{1,0,0}});
 for(int hint:{0,1,2,4000}){
  EXPECT_EQ(ties.nearest({0,0,0},1.,nullptr,hint),0);
  EXPECT_EQ(ties.nearest({0,0,0},std::nextafter(1.,0.),nullptr,hint),-1);
 }
}
TEST(ExactKdTree, ThinPlanesNegativeCoordinatesAndReadOnlyConcurrency){
 std::vector<Tree::Point> points;for(int i=-40;i<=40;++i)for(int j=-40;j<=40;++j)points.push_back({i*.05f,j*.05f,1.e-6f*(i%3)});
 Tree tree;tree.build(points);std::vector<Tree::Point> queries;for(int i=-100;i<=100;++i)queries.push_back({i*.0137f,-i*.0071f,.01f});
 std::vector<int> serial;for(const auto& q:queries)serial.push_back(tree.nearest(q,2.25));
 std::array<std::vector<int>,4> parallel;std::array<std::thread,4> workers;
 for(int t=0;t<4;++t)workers[t]=std::thread([&,t]{for(const auto& q:queries)parallel[t].push_back(tree.nearest(q,2.25));});
 for(auto& worker:workers)worker.join();for(const auto& result:parallel)EXPECT_EQ(serial,result);
 for(std::size_t i=0;i<queries.size();++i)EXPECT_EQ(serial[i],brute(points,queries[i],2.25).first);
}
TEST(ExactKdTree, RejectsNonfiniteInputAndInvalidQueries){
 Tree tree;EXPECT_THROW(tree.build({{0,0,std::numeric_limits<float>::quiet_NaN()}}),std::invalid_argument);
 tree.build({{0,0,0}});EXPECT_EQ(tree.nearest({0,0,0},-1),-1);EXPECT_EQ(tree.nearest({INFINITY,0,0},1),-1);
}
