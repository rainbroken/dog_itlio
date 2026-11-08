#include "stream_frequency.hpp"
#include <gtest/gtest.h>
TEST(StreamFrequency, BurstCallbacksCannotInflateTenAndTwoHundredHertzSources){
  for(const auto interval:{100000000LL,5000000LL}){
    StreamFrequency rate;
    // All calls are immediate, as when DDS drains a queue after a busy frame.
    for(int64_t i=0;i<501;++i)rate.observe(1000000000LL+i*interval);
    ASSERT_TRUE(rate.available());EXPECT_NEAR(rate.hz(),1e9/interval,1e-9);
  }
}
TEST(StreamFrequency, DuplicatesClockResetsAndGapsDoNotReuseOldFrequency){
  StreamFrequency rate;
  for(int64_t i=0;i<21;++i){rate.observe(1000000000LL+i*100000000LL);rate.observe(1000000000LL+i*100000000LL);}
  ASSERT_TRUE(rate.available());EXPECT_DOUBLE_EQ(rate.hz(),10.);
  rate.observe(500000000LL);EXPECT_FALSE(rate.available());
  rate.observe(10000000000LL);EXPECT_FALSE(rate.available());
  rate.observe(0);EXPECT_FALSE(rate.available());EXPECT_DOUBLE_EQ(rate.hz(),0.);
}
