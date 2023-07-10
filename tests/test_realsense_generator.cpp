#include <gtest/gtest.h>
#include "realsense_generator.hpp"
#include <system_error>

TEST(RealsenseGeneratorTest, ReturnsFov) {
  auto [pipe, fovh, fovv] = setup_device().or_else([] (std::error_code e){ FAIL() << e.message(); }).value();
  EXPECT_NE(fovh, 0.0f);
  EXPECT_NE(fovv, 0.0f);
  pipe.stop();
}

// TEST(RealsenseGeneratorTest, ReturnNonTrivialPoints) {
//   auto [pipe, fovh, fovv] = setup_device().or_else([] (std::error_code e){ FAIL() << e.message(); }).value();
//   auto points_sequence = next_cloud_and_image(pipe);
//   for (int i = 0; i < 10; i++) {
//     auto [points, frame] = *cppcoro::sync_wait(begin(points_sequence));
//     EXPECT_GT(points.size(), 0);
//   }
// }
