/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#include <gtest/gtest.h>
#include <cmath>
#include "amr_common/math.h"

constexpr double kPi = M_PI;

TEST(math, Rad2Deg) {
  using reflector_localizer::math::Rad2Deg;
  EXPECT_DOUBLE_EQ(0.0, Rad2Deg(0.0));
  EXPECT_DOUBLE_EQ(90.0, Rad2Deg(kPi / 2.0));
  EXPECT_DOUBLE_EQ(180.0, Rad2Deg(kPi));
  EXPECT_DOUBLE_EQ(360.0, Rad2Deg(kPi * 2.0));
  EXPECT_DOUBLE_EQ(-90.0, Rad2Deg(-kPi / 2.0));
  EXPECT_DOUBLE_EQ(-180.0, Rad2Deg(-kPi));
  EXPECT_DOUBLE_EQ(-360.0, Rad2Deg(-kPi * 2.0));
}

TEST(math, Deg2Rad) {
  using math::Deg2Rad;
  EXPECT_DOUBLE_EQ(0.0, Deg2Rad(0.0));
  EXPECT_DOUBLE_EQ(kPi / 2, Deg2Rad(90.0));
  EXPECT_DOUBLE_EQ(kPi, Deg2Rad(180.0));
  EXPECT_DOUBLE_EQ(kPi * 2.0, Deg2Rad(360.0));
  EXPECT_DOUBLE_EQ(-kPi / 2.0, Deg2Rad(-90.0));
  EXPECT_DOUBLE_EQ(-kPi, Deg2Rad(-180.0));
  EXPECT_DOUBLE_EQ(-kPi * 2.0, Deg2Rad(-360.0));
}

TEST(math, CosineTheorem) {
  using math::CosineTheorem;
  EXPECT_DOUBLE_EQ(1., CosineTheorem(1., 1., kPi / 3.));
}
