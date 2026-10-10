// Copyright 2026 Zach. MIT License.

#include <gtest/gtest.h>

#include <cmath>

#include "vision_bot_plugins/avoidance_cost.hpp"

using vision_bot_plugins::avoidanceInfluenceRadius;
using vision_bot_plugins::computeAvoidanceCost;
using vision_bot_plugins::isPoseFresh;

TEST(AvoidanceCost, LethalAtCenter)
{
  EXPECT_EQ(computeAvoidanceCost(0.0, 0.4, 3.0), 254);
}

TEST(AvoidanceCost, LethalAtRadius)
{
  EXPECT_EQ(computeAvoidanceCost(0.4, 0.4, 3.0), 254);
}

TEST(AvoidanceCost, DecaysBeyondRadius)
{
  // One decay length past the radius: floor(252 * exp(-1)) = 92.
  EXPECT_EQ(computeAvoidanceCost(0.4 + 1.0 / 3.0, 0.4, 3.0), 92);
}

TEST(AvoidanceCost, ZeroFarAway)
{
  EXPECT_EQ(computeAvoidanceCost(10.0, 0.4, 3.0), 0);
}

TEST(AvoidanceCost, MonotonicNonIncreasing)
{
  unsigned char previous = computeAvoidanceCost(0.0, 0.4, 3.0);
  for (int step = 1; step <= 100; ++step) {
    const unsigned char cost = computeAvoidanceCost(step * 0.05, 0.4, 3.0);
    EXPECT_LE(cost, previous) << "cost rose at distance " << step * 0.05;
    previous = cost;
  }
}

TEST(AvoidanceCost, NeverInscribedOutsideRadius)
{
  // 253 means "certain collision for this robot"; only the other robot's
  // own disk may reach it (as 254).
  EXPECT_LE(computeAvoidanceCost(0.4001, 0.4, 3.0), 252);
}

TEST(AvoidanceCost, ZeroAtInfluenceRadius)
{
  const double r = avoidanceInfluenceRadius(0.4, 3.0);
  EXPECT_NEAR(r, 0.4 + std::log(253.0) / 3.0, 1e-12);
  EXPECT_EQ(computeAvoidanceCost(r, 0.4, 3.0), 0);
  EXPECT_GT(computeAvoidanceCost(r - 0.05, 0.4, 3.0), 0);
}

TEST(AvoidanceCost, StalePoseIgnored)
{
  EXPECT_FALSE(isPoseFresh(0.0, 5.0, 2.0));
  EXPECT_TRUE(isPoseFresh(4.5, 5.0, 2.0));
}
