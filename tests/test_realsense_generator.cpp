#include <gtest/gtest.h>
#include "lib/realsense_generator.hpp"

TEST(RealsenseGeneratorTest, ReturnsFov) {
  auto [_, fovh, fovv] = setup_device();
  EXPECT_NE(fovh, 0.0f);
  EXPECT_NE(fovv, 0.0f);
}
