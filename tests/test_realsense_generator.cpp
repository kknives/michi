#include <exception>
#include <gtest/gtest.h>
#include "realsense_generator.hpp"
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include <system_error>
#include <asio.hpp>

using namespace std::literals::chrono_literals;
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

int main(int argc, char** argv) {
  try {
    auto logger = spdlog::basic_logger_mt("logger", "test.log");
    spdlog::set_default_logger(logger);
    ::testing::InitGoogleTest(&argc, argv);

    spdlog::info("---realsense testing start---");
    int result = RUN_ALL_TESTS();
    spdlog::info("---realsense testing done----");
    return result;
  }
  catch (const spdlog::spdlog_ex &ex) {
    std::cerr << "Log init failed: " << ex.what() << "\n";
  }
}

TEST(RealsenseGeneratorTest, ReturnNonTrivialPoints) {
  asio::io_context io_ctx(1);
  auto [pipe, fovh, fovv] = setup_device().or_else([] (std::error_code e){ FAIL() << e.message(); }).value();
  auto rs_dev = RealsenseDevice(pipe);

  for (int i = 0; i < 5; i++)
  asio::co_spawn(io_ctx, rs_dev.async_get_points(io_ctx),
    [](std::exception_ptr p, rs2::points points) {
      if (p) {
        try { std::rethrow_exception(p); }
        catch (const std::exception& e) {
         FAIL() << "RealsenseDevice coroutine threw exception: " << e.what() << "\n";
        }
        return;
      }
      EXPECT_GT(points.size(), 0);
      spdlog::info("Got points of size {}", points.size());
  });

  io_ctx.run_for(60s);
}
