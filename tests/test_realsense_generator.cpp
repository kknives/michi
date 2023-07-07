#include <gtest/gtest.h>
#include "realsense_generator.hpp"
#include <system_error>

TEST(RealsenseGeneratorTest, ReturnsFov) {
  auto [_, fovh, fovv] = setup_device().or_else([] (std::error_code e){ FAIL() << e.message(); }).value();
  EXPECT_NE(fovh, 0.0f);
  EXPECT_NE(fovv, 0.0f);
}
