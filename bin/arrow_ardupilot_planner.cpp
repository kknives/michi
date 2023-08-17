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

auto locate_obstacles(RealsenseDevice& rs_dev) -> asio::awaitable<void> {
  asio::steady_timer timer(co_await asio::this_coro::executor);
  for (;;) {
    auto points = co_await rs_dev.async_get_points(co_await asio::this_coro::executor);
    // TODO: Bring in the method from rs_pcl_color and use MAVLinkInterface
  }
}
int main() {
  print("{}\n",banner);
  spdlog::info("Starting ArrowArdupilotPlanner version {}", "version_string");

  asio::io_context io_ctx;
  spdlog::trace("asio io_context setup");

  auto [rs_pipe, fovh, fovv] = *setup_device().or_else([] (std::error_code e) {
    spdlog::error("Couldn't setup realsense device: {}", e.message());
  });
  auto rs_dev = RealsenseDevice(rs_pipe);
  asio::co_spawn(io_ctx, locate_obstacles(rs_dev), asio::detached);

  // Mission Loop
  for (;;) {
    // Accept images as input
    // Check if there's an arrow
  }
  
  // asio::co_spawn(io_ctx, rs_dev)
  // spdlog::trace("Spawning realsense co-routine");
  // asio::co_spawn(io_ctx, rs_dev.)

  spdlog::trace("running asio io_context");
  io_ctx.run();
}
