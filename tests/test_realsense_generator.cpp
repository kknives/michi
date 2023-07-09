#include <catch2/catch_test_macros.hpp>
#include "realsense_generator.hpp"
#include <system_error>
#include "cppcoro/sync_wait.hpp"

TEST_CASE("A non-zero FOV is returned", "[requires_device]") {
  auto [pipe, fovh, fovv] = setup_device().or_else([] (std::error_code e){ FAIL( e.message() ); }).value();
  REQUIRE(fovh > 0.0f);
  REQUIRE(fovv > 0.0f);
  pipe.stop();
}

TEST_CASE("Generator returns a non-trivial pointset", "[requires_device]") {
  auto [pipe, fovh, fovv] = setup_device().or_else([] (std::error_code e){ FAIL( e.message() ); }).value();
  auto points_sequence = next_cloud_and_image(pipe);
  for (int i = 0; i < 10; i++) {
    auto [points, frame] = *cppcoro::sync_wait(begin(points_sequence));
    REQUIRE(points.size() > 0);
  }
}
