#pragma once

#include "ardupilot_interface.hpp"
#include "expected.hpp"
#include <asio/async_result.hpp>
#include <asio/io_context.hpp>
#include <asio/steady_timer.hpp>
#include <compare>
#include <librealsense2/h/rs_sensor.h>
#include <spdlog/spdlog.h>
#include <coroutine>
#include <optional>
#include <chrono>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <thread>
#include <type_traits>
#include <utility>
#include <ranges>
#include <system_error>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <asio.hpp>

using namespace std::literals::chrono_literals;
template <typename T>
using tResult = tl::expected<T, std::error_code>;
using tl::make_unexpected;

enum class DeviceErrc {
  // 0 imples success
  NoDeviceConnected = 10, // Setup error
  LibrsError = 20, // librealsense gave back an error
};
struct DeviceErrCategory : std::error_category {
  const char* name() const noexcept override {
    return "DeviceIO";
  }
  std::string message(int ev) const override {
    switch (static_cast<DeviceErrc>(ev)) {
      case DeviceErrc::NoDeviceConnected:
      return "no device connected: cannot acquire data";
      case DeviceErrc::LibrsError:
      return "failure in librealsense";
      default:
      return "(unrecognized error)";
    }
  }
};
const DeviceErrCategory deverrc_category;
std::error_code make_error_code(DeviceErrc e) {
  return {static_cast<int>(e), deverrc_category};
}
namespace std {
  template <>
  struct is_error_code_enum<DeviceErrc> : true_type {};
}
auto setup_device() noexcept -> tResult<std::tuple<rs2::pipeline, float, float>> {
  try{
    rs2::pipeline pipe;
    rs2::config stream_config;
    rs2::context ctx;
    float fov[2];

    auto devices = ctx.query_devices();
    if (devices.size() == 0) return make_unexpected(DeviceErrc::NoDeviceConnected);
    stream_config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 0, 424, 240, rs2_format::RS2_FORMAT_Z16, 30);
    stream_config.enable_stream(rs2_stream::RS2_STREAM_COLOR); // Choose resolution here
    rs2::pipeline_profile selection = pipe.start(stream_config);
    auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto i = depth_stream.get_intrinsics();
    rs2_fov(&i, fov);
    fov[0] = (fov[0] * M_PI)/180.0f;
    fov[1] = (fov[1] * M_PI)/180.0f;
    return std::tie(pipe, fov[0], fov[1]);
  }
  catch (const std::exception& e) {
    spdlog::error("Exception in setup_device(): {}", e.what());
    return make_unexpected(DeviceErrc::LibrsError);
  }
}

class RealsenseDevice {
  // TODO: remove io_ctx
  auto async_update(const asio::any_io_executor& io_ctx) -> asio::awaitable<void> {
    asio::steady_timer timer(co_await asio::this_coro::executor);
    while (not pipe.poll_for_frames(&frames)) {
      timer.expires_after(34ms);
      co_await timer.async_wait(use_nothrow_awaitable);
    }
  }
  
  public:
  RealsenseDevice(rs2::pipeline& pipe) : pipe{pipe} {}
  auto async_get_rgb_frame(const asio::any_io_executor& io_ctx) -> asio::awaitable<rs2::frame> {
    rs2::frame rgb_frame = frames.first_or_default(RS2_STREAM_COLOR);
    while (not rgb_frame) {
      co_await async_update(io_ctx);
      rgb_frame = frames.first_or_default(RS2_STREAM_COLOR);
    }
    co_return rgb_frame;
  }
  auto async_get_depth_frame() -> asio::awaitable<rs2::frame> {
    rs2::frame depth = frames.first_or_default(RS2_STREAM_DEPTH);
    while (not depth) {
      co_await async_update(co_await asio::this_coro::executor);
      depth = frames.first_or_default(RS2_STREAM_DEPTH);
    }
    // Decimation > Spatial > Temporal > Threshold
    depth = dec_filter.process(depth);
    depth = temp_filter.process(depth);
    co_return depth;
  }
  auto async_get_points(const asio::any_io_executor& io_ctx) -> asio::awaitable<rs2::points>{
    rs2::frame depth = co_await async_get_depth_frame();
    co_return pc.calculate(depth);
  }
  private:
  rs2::pipeline pipe;
  rs2::frameset frames;

  rs2::decimation_filter dec_filter;
  rs2::temporal_filter temp_filter;
  rs2::pointcloud pc;
};
