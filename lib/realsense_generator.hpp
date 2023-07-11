#include "expected.hpp"
#include <spdlog/spdlog.h>
#include <coroutine>
#include <iostream>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <type_traits>
#include <utility>
#include <ranges>
#include <system_error>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <Eigen/Geometry>

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

struct PipelineReady {
  public:
  PipelineReady(rs2::pipeline& pipe, rs2::frameset& frames) : pipe{pipe}, frames{frames} {}
  bool await_ready() {
    return pipe.poll_for_frames(&frames);
  }
  void await_suspend(std::coroutine_handle<>) {}
  void await_resume() {}
  rs2::pipeline &pipe;
  rs2::frameset frames;
};

// auto next_cloud_and_image(rs2::pipeline pipe) -> cppcoro::async_generator<std::tuple<rs2::points, rs2::frame>> {
//   rs2::frameset frames;
//   while (true) {
//     co_await PipelineReady{pipe, frames};
//     rs2::frame depth = frames.get_depth_frame();

//     rs2::decimation_filter dec_filter;
//     rs2::temporal_filter temp_filter;
//     depth = dec_filter.process(depth);
//     depth = temp_filter.process(depth);

//     rs2::pointcloud pc;
//     rs2::points points = pc.calculate(depth);
//     co_yield std::tie(points, depth);
//   }
// }
