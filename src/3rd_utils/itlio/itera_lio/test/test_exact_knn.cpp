#include <gtest/gtest.h>
#include <random>
#include "itera_lio/loop/exact_kdtree.hpp"
using Tree=itera_lio::loop::detail::ExactKdTree;
TEST(ExactRefinement, KNNAndHintMatchBruteForceIncludingTiesAndCutoffs){
 std::mt19937 rng(20261024);std::uniform_real_distribution<float>d(-20,20);std::vector<Tree::Point>p;
 for(int i=0;i<3000;++i)p.push_back({d(rng),d(rng),d(rng)});
 for(int i=0;i<80;++i)p.push_back(p[i]);Tree t;t.build(p);
 std::vector<int>ids;std::vector<float>ds;
 for(int j=0;j<700;++j){Tree::Point q=j<100?p[j]:Tree::Point{d(rng),d(rng),d(rng)};
  std::vector<std::pair<float,int>> brute;
  for(int i=0;i<(int)p.size();++i){float x=q[0]-p[i][0],y=q[1]-p[i][1],z=q[2]-p[i][2];brute.emplace_back(x*x+y*y+z*z,i);}std::sort(brute.begin(),brute.end());
  for(int k:{1,5,10,25,4000}){ASSERT_EQ(t.nearestK(q,k,ids,ds),std::min(k,(int)p.size()));
   for(int a=0;a<(int)ids.size();++a){ASSERT_EQ(ids[a],brute[a].second);ASSERT_EQ(ds[a],brute[a].first);}}
  for(int hint:{-2,-1,0,j,brute[0].second,5000})for(double bound:{0.,1.,20.,10000.}){
   const int expected=brute[0].first<=bound?brute[0].second:-1;EXPECT_EQ(t.nearest(q,bound,nullptr,hint),expected);}
 }
}
TEST(ExactRefinement, KNNEmptyAndNonfinite){Tree t;std::vector<int>i;std::vector<float>d;EXPECT_EQ(t.nearestK({0,0,0},10,i,d),0);t.build({{0,0,0}});EXPECT_EQ(t.nearestK({NAN,0,0},10,i,d),0);EXPECT_EQ(t.nearestK({0,0,0},0,i,d),0);}
