#pragma once

#include "expected.hpp"
#include <asio/async_result.hpp>
#include <asio/io_context.hpp>
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

#include <Eigen/Geometry>

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
  auto async_update(asio::io_context& io_ctx) -> asio::awaitable<void> {
    // FIXME: Sync to frame time using an asio::timer instead of sleeping
    while (not pipe.poll_for_frames(&frames)) {
      co_await asio::this_coro::executor;
      std::this_thread::sleep_for(10ms);
    }

    rs2::frame depth = frames.first(RS2_STREAM_DEPTH);
    // Decimation > Spatial > Temporal > Threshold
    depth = dec_filter.process(depth);
    depth = temp_filter.process(depth);

    points.emplace(pc.calculate(depth));

    rs2::frame color = frames.first(RS2_STREAM_COLOR);
    rgb_frame.emplace(color);
  }
  
  public:
  RealsenseDevice(rs2::pipeline& pipe) : pipe{pipe} {}
  auto async_get_rgb_frame(asio::io_context& io_ctx) -> asio::awaitable<rs2::frame> {
    if (not rgb_frame.has_value()) co_await async_update(io_ctx);
    co_return *std::exchange(rgb_frame, std::nullopt);
  }
  auto async_get_points(asio::io_context& io_ctx) -> asio::awaitable<rs2::points>{
    if (not points.has_value()) co_await async_update(io_ctx);
    co_return *std::exchange(points, std::nullopt);
  }
  private:
  rs2::pipeline pipe;
  rs2::frameset frames;
  std::optional<rs2::frame> rgb_frame;
  std::optional<rs2::points> points;

  rs2::decimation_filter dec_filter;
  rs2::temporal_filter temp_filter;
  rs2::pointcloud pc;
};
