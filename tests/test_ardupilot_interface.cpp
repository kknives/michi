#include <asio/detached.hpp>
#include <exception>
#include <gtest/gtest.h>
#include "ardupilot_interface.hpp"

using namespace std::literals::chrono_literals;
TEST(ArdupilotInterfaceTest, HeartBeats) {
  asio::io_context io_ctx;
  MavlinkInterface mi(asio::serial_port(io_ctx, "/dev/ttyACM0"));
  asio::co_spawn(io_ctx, mi.init(), asio::detached);
  for (int i = 0; i < 4; i++) {
    asio::co_spawn(io_ctx, mi.heartbeat(), [](std::exception_ptr p, tResult<void> r) {
      if (p) {
        try { std::rethrow_exception(p); }
        catch(const std::exception& e) {
          FAIL() << "Heartbeat coroutine threw exception: " << e.what() << "\n";
        }
      }
      r.map_error([] (std::error_code e) {
        FAIL() << "Heartbeat coroutine faced error: " << e.category().name() << ": " << e.message() << "\n";
      });
    });
  }
  io_ctx.run_for(5s);
}