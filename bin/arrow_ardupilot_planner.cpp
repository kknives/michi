#include "realsense_generator.hpp"
#include "classification_model.hpp"
#include "mobilenet_arrow.hpp"
// #include "arrow_global_planner.hpp"
#include <asio/detached.hpp>
#include <asio/this_coro.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>
#include <asio.hpp>

const char* banner = R"Banner(
      >>                     >>               >======>    >=>
     >>=>                   >>=>              >=>    >=>  >=>
    >> >=>     >> >==>     >> >=>     >> >==> >=>    >=>  >=>    >=> >=>  >==>>==>  >==>>==>    >==>    >> >==>
   >=>  >=>     >=>       >=>  >=>     >=>    >======>    >=>  >=>   >=>   >=>  >=>  >=>  >=> >>   >=>   >=>
  >=====>>=>    >=>      >=====>>=>    >=>    >=>         >=> >=>    >=>   >=>  >=>  >=>  >=> >>===>>=>  >=>
 >=>      >=>   >=>     >=>      >=>   >=>    >=>         >=>  >=>   >=>   >=>  >=>  >=>  >=> >>         >=>
>=>        >=> >==>    >=>        >=> >==>    >=>        >==>   >==>>>==> >==>  >=> >==>  >=>  >====>   >==>
)Banner";

struct ArrowState {
};

struct Parameters {
  float arrow_min_approach_mtr;
  float cone_min_approach_mtr;
  float arrow_hold_ms;  
  float arrow_careful_vel_ms;
  float arrow_approach_vel_ms;
};

using fmt::print;

auto locate_obstacles(RealsenseDevice& rs_dev, MavlinkInterface& mi) -> asio::awaitable<void> {
  asio::steady_timer timer(co_await asio::this_coro::executor);
  
  for (;;) {
    auto points = co_await rs_dev.async_get_points(co_await asio::this_coro::executor);
    // TODO: Bring in the method from rs_pcl_color and use MAVLinkInterface
  }
}

auto mission() -> asio::awaitable<void> {
  auto this_exec = co_await asio::this_coro::executor;

  auto [rs_pipe, fovh, fovv] = *setup_device().or_else([] (std::error_code e) {
    spdlog::error("Couldn't setup realsense device: {}", e.message());
  });
  auto rs_dev = RealsenseDevice(rs_pipe);
  auto classifier = ClassificationModel(MobilenetArrowClassifier("lib/saved_model_checkpoint4.onnx"));
  auto mi = MavlinkInterface(asio::serial_port(this_exec, "/dev/pts/13"));

  bool done = false;
  // std::optional<Target> current_target;
  asio::co_spawn(this_exec, locate_obstacles(rs_dev, mi), asio::detached);
  asio::co_spawn(this_exec, heartbeat_loop(mi), asio::detached);
  asio::co_spawn(this_exec, mi.receive_message_loop(), asio::detached);

  // Mission Loop
  for (;;) {
    auto rgb_frame = co_await rs_dev.async_get_rgb_frame(this_exec);
    cv::Mat image(cv::Size(640, 480), CV_8UC3, (void*)rgb_frame.get_data(), cv::Mat::AUTO_STEP);
    auto object_found = classify(classifier, image);

    if (object_found == 1 or object_found == 2) {
      // Arrow found
    }
    // Accept images as input
    // Check if there's an arrow
  }
}
int main() {
  print("{}\n",banner);
  spdlog::info("Starting ArrowArdupilotPlanner version {}", "version_string");

  // std::set_terminate(stacktrace_terminate);
  asio::io_context io_ctx;
  spdlog::trace("asio io_context setup");

  asio::co_spawn(io_ctx, mission(), asio::detached);

  spdlog::trace("running asio io_context");
  io_ctx.run();
}
