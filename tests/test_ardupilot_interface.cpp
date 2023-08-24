#include <asio/detached.hpp>
#include <exception>
#include <gtest/gtest.h>
#include <thread>
#include "ardupilot_interface.hpp"

using namespace std::literals::chrono_literals;
TEST(ArdupilotInterfaceTest, HeartBeats) {
  asio::io_context io_ctx(2);
  tcp::socket sitl(io_ctx);
  sitl.connect(*tcp::resolver(io_ctx).resolve("0.0.0.0", "5760", tcp::resolver::passive));
  MavlinkInterface mi(std::move(sitl));
  asio::co_spawn(io_ctx, mi.receive_message_loop(), [](std::exception_ptr p, tResult<void> r) {
    if (p) {
      try { std::rethrow_exception(p); }
      catch(const std::exception& e) {
        FAIL() << "receive_message_loop coroutine threw exception: " << e.what() << "\n";
      }
    }
    r.map_error([] (std::error_code e) {
      FAIL() << "receive_message_loop coroutine faced error: " << e.category().name() << ": " << e.message() << "\n";
    });
  });
    asio::co_spawn(io_ctx, heartbeat_loop(mi), [](std::exception_ptr p, tResult<void> r) {
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
  asio::co_spawn(io_ctx, mi.set_guided_mode_armed(), [](std::exception_ptr p, tResult<void> r) {
    if (p) {
      try { std::rethrow_exception(p); }
      catch(const std::exception& e) {
        FAIL() << "Set guided mode coroutine threw exception: " << e.what() << '\n';
      }
    }
    r.map_error([](std::error_code e) {
      FAIL() << "Set guided mode coroutine faced error: " << e.category().name() << ": " << e.message() << "\n";
    });
  });
  io_ctx.run_for(5s);
}