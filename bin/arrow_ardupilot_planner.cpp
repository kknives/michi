#include <argparse/argparse.hpp>

#include "ardupilot_interface.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include "realsense_generator.hpp"
#include "classification_model.hpp"
#include "mobilenet_arrow.hpp"
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

using tLocation = std::tuple<double, double, double>;
struct Target {
  enum class Type {
    ARROW_LOCKED,
    CONE_LOCKED,
    ARROW_SIGHTED,
    CONE_SIGHTED,
    HEADING,
  };
  Type type;
  std::optional<tLocation> location;
  std::optional<float> heading;
};
struct ArrowState {
};

using fmt::print;
using tPclPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

static argparse::ArgumentParser args("ArrowArdupilotPlanner");

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
  auto classifier = ClassificationModel(MobilenetArrowClassifier(args.get("--model")));
  auto mi = MavlinkInterface(asio::serial_port(this_exec, args.get("ardupilot")));

  bool done = false;
  asio::co_spawn(this_exec, locate_obstacles(rs_dev, mi), asio::detached);
  asio::co_spawn(this_exec, heartbeat_loop(mi), asio::detached);
  asio::co_spawn(this_exec, mi.receive_message_loop(), asio::detached);

  Target current_target{.type=Target::Type::HEADING, .heading=0.0f};
  std::set<Target> visited_targets;

  // Mission Loop
  for (;;) {
    if (current_target.type == Target::Type::HEADING) {
      // Get the current frame and process it for potential targets
      auto rgb_frame = co_await rs_dev.async_get_rgb_frame(this_exec);
      cv::Mat image(cv::Size(640, 480), CV_8UC3, (void*)rgb_frame.get_data(), cv::Mat::AUTO_STEP);
      auto object_found = classify(classifier, image);

      if (object_found == 1 or object_found == 2) {
        // Arrow found
        current_target = Target{.type=Target::Type::ARROW_SIGHTED};
      }

      // If a target is found, check for duplicates against visited_targets
      // make a new target, if unique
      // If depth can lock it, set a target on it with AP
      // Otherwise, reduce velocity and change heading to match target

      // If no target is found, maintain heading and loop
      continue;
    }

    // There's already a target, check target
    if (current_target.type == Target::Type::ARROW_SIGHTED or current_target.type == Target::Type::CONE_SIGHTED) {
      // Check if realsense depth can give a lock on the location
      // Lock the target by setting a target on AP
      // If not, set the velocity low, match heading and carefully approach target
      continue;
    }
    if (current_target.type == Target::Type::ARROW_LOCKED or current_target.type == Target::Type::CONE_LOCKED) {
      // Check if the target approach, reduce velocity if needed,
      // Get the distance to target, cutoff appraoch and wait if needed
      // Don't continue, await the wait timer instead
      // Turn and set heading, clear current_target, push it to visited_targets
    }
  }
}

extern "C" void dump_stacktrace(uintptr_t const);
void stacktrace_terminate() {
  std::cout << "too baddd" << '\n';
  dump_stacktrace(reinterpret_cast<uintptr_t>(mission));
  std::abort();
}
int main(int argc, char* argv[]) {
  std::set_terminate(stacktrace_terminate);
  args.add_argument("ardupilot").help("Serial port (eg. /dev/ttyUSB0) connected to Pixhawk's TELEMETRY2");
  args.add_argument("-m", "--model").default_value(std::string("lib/saved_model_checkpoint4.onnx")).help("model to use for arrow classification");

  int log_verbosity = 0;
  args.add_argument("-V", "--verbose")
  .action([&](const auto &) {++log_verbosity;})
  .append()
  .default_value(false)
  .implicit_value(true)
  .nargs(0);
  
  try {
    args.parse_args(argc, argv);
  }
  catch (const std::runtime_error& err) {
    std::cerr << err.what() << '\n';
    std::cerr << args;
    return 1;
  }

  spdlog::set_level(spdlog::level::debug);
  print("{}\n",banner);
  spdlog::info("Starting ArrowArdupilotPlanner version {}", "version_string");

  asio::io_context io_ctx;
  spdlog::trace("asio io_context setup");

  asio::co_spawn(io_ctx, mission(), asio::detached);

  spdlog::trace("running asio io_context");
  io_ctx.run();
}
