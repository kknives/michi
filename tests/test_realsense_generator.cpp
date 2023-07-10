#include <gtest/gtest.h>
#include "realsense_generator.hpp"
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <system_error>

using tDone = std::tuple<>;

TEST(RealsenseGeneratorTest, ReturnsFov) {
  setup_device().map([] (std::tuple<rs2::pipeline, float, float> v) {
    auto [pipe, fovh, fovv] = v;
    EXPECT_NE(fovh, 0.0f);
    EXPECT_NE(fovv, 0.0f);
    pipe.stop();
    return tDone();
  }).or_else([] (std::error_code e){ FAIL() << e.message(); });
}

// TEST(RealsenseGeneratorTest, ReturnNonTrivialPoints) {
//   auto [pipe, fovh, fovv] = setup_device().or_else([] (std::error_code e){ FAIL() << e.message(); }).value();
//   auto points_sequence = next_cloud_and_image(pipe);
//   for (int i = 0; i < 10; i++) {
//     auto [points, frame] = *cppcoro::sync_wait(begin(points_sequence));
//     EXPECT_GT(points.size(), 0);
//   }
// }
