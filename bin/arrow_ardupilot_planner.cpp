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

#include <pcl/common/angles.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/range_image/range_image.h>

const char* banner = R"Banner(
      >>                     >>               >======>    >=>
     >>=>                   >>=>              >=>    >=>  >=>
    >> >=>     >> >==>     >> >=>     >> >==> >=>    >=>  >=>    >=> >=>  >==>>==>  >==>>==>    >==>    >> >==>
   >=>  >=>     >=>       >=>  >=>     >=>    >======>    >=>  >=>   >=>   >=>  >=>  >=>  >=> >>   >=>   >=>
  >=====>>=>    >=>      >=====>>=>    >=>    >=>         >=> >=>    >=>   >=>  >=>  >=>  >=> >>===>>=>  >=>
 >=>      >=>   >=>     >=>      >=>   >=>    >=>         >=>  >=>   >=>   >=>  >=>  >=>  >=> >>         >=>
>=>        >=> >==>    >=>        >=> >==>    >=>        >==>   >==>>>==> >==>  >=> >==>  >=>  >====>   >==>
)Banner";

// TODO: Add hash function/PoSet relation
struct Target {
  enum class Type {
    ARROW_LOCKED,
    CONE_LOCKED,
    ARROW_SIGHTED,
    CONE_SIGHTED,
    HEADING,
  };
  Type type;
  std::optional<std::array<float, 3>> location;
  std::optional<float> heading;
};

using fmt::print;
using tPclPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

static argparse::ArgumentParser args("ArrowArdupilotPlanner");

void
calculate_obstacle_distances(tPclPtr pc,
                             std::array<uint16_t, 72>& distances,
                             std::span<float, 2> fov)
{
  Eigen::Affine3f rs_pose =
    static_cast<Eigen::Affine3f>(Eigen::Translation3f(0.0f, 0.0f, 0.0f));
  float angular_res = (float)(1.0f * (M_PI / 180.0f));
  pcl::RangeImage::CoordinateFrame coord_frame = pcl::RangeImage::CAMERA_FRAME;
  float noise_lvl = 0.0f;
  float min_range = 0.0f;
  int border = 0;
  pcl::RangeImage rg_img;
  rg_img.createFromPointCloud(*pc,
                              angular_res,
                              fov[0],
                              fov[1],
                              rs_pose,
                              coord_frame,
                              noise_lvl,
                              min_range,
                              border);
  float* goods = rg_img.getRangesArray();
  std::cout << rg_img << "\n";
  std::cout << goods << "\n";

  int rays = distances.size();
  for (int i = 1; i <= rays; i++) {
    pcl::PointWithRange ray;
    int idx = i * (88.0f / 72.0f);
    rg_img.get1dPointAverage(idx, 1, 1, 58, 58, ray);
    distances[i - 1] = uint16_t(ray.range);
  }
}

tPclPtr points_to_pcl(const rs2::points& points)
{
    tPclPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

auto locate_obstacles(RealsenseDevice& rs_dev, MavlinkInterface& mi, std::span<float, 2> fov) -> asio::awaitable<void> {
  asio::steady_timer timer(co_await asio::this_coro::executor);
  
  tPclPtr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass_filter;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  std::array<uint16_t, 72> distances;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  for (;;) {
    auto points = co_await rs_dev.async_get_points(co_await asio::this_coro::executor);
    auto pcl_points = points_to_pcl(points);
    pass_filter.setInputCloud(pcl_points);
    pass_filter.setFilterFieldName("z");
    pass_filter.setFilterLimits(0.0, 1.0);
    pass_filter.filter(*cloud_filtered);

    voxel_filter.setInputCloud(cloud_filtered);
    voxel_filter.setLeafSize(0.01f,0.01f,0.01f);
    voxel_filter.filter(*cloud_filtered);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*obstacle_cloud);

    calculate_obstacle_distances(pcl_points, distances, fov);

    co_await mi.set_obstacle_distance(
      std::span(distances), 88.0f / 72.0f, 17.5f, 300.0f, 15.0f);
  }
}
auto get_depth_lock(rs2::frame& depth_frame, std::span<float, 4> rect_vertices) -> std::optional<float> {
  cv::Point2f top_left(rect_vertices[0], rect_vertices[1]), bottom_right(rect_vertices[2], rect_vertices[3]);
  cv::Mat depth_frame_mat(cv::Size(640, 480), CV_8UC1, const_cast<void*>(depth_frame.get_data()), cv::Mat::AUTO_STEP);
  auto cropped = depth_frame_mat(cv::Rect(top_left, bottom_right));

  int valid_depths = 0;
  int mean_depth = 0;
  using tPixel = cv::Point_<uint8_t>;
  cropped.forEach<tPixel>([&valid_depths, &mean_depth](const tPixel &p, const int* pos) {
    valid_depths += (p.x != 0);
    mean_depth += p.x;
  });
  std::optional<float> distance;
  if (valid_depths >= (0.5*cropped.total())) {
    // Lock available
    distance.emplace(mean_depth/cropped.total());
  }
  return distance;
}

auto mission() -> asio::awaitable<void> {
  auto this_exec = co_await asio::this_coro::executor;

  auto [rs_pipe, fovh, fovv] = *setup_device().or_else([] (std::error_code e) {
    spdlog::error("Couldn't setup realsense device: {}", e.message());
  });
  std::array<float, 2> fov = {fovh, fovv};
  auto rs_dev = RealsenseDevice(rs_pipe);
  auto classifier = ClassificationModel(MobilenetArrowClassifier(args.get("--model")));
  auto mi = MavlinkInterface(tcp::socket(this_exec, *tcp::resolver(this_exec).resolve("0.0.0.0", "5760")));

  bool done = false;
  if (not args.get<bool>("--no-avoid"))
    asio::co_spawn(this_exec, locate_obstacles(rs_dev, mi, std::span(fov)), asio::detached);
  asio::co_spawn(this_exec, heartbeat_loop(mi), asio::detached);
  asio::co_spawn(this_exec, mi.receive_message_loop(), asio::detached);

  asio::steady_timer timer(this_exec);
  Target current_target{.type=Target::Type::HEADING, .heading=0.0f};
  std::set<Target> visited_targets;

  spdlog::info("Starting mission loop");
  // Mission Loop
  for (;;) {
    if (current_target.type == Target::Type::HEADING) {
      // Get the current frame and process it for potential targets
      auto rgb_frame = co_await rs_dev.async_get_rgb_frame(this_exec);
      auto depth_frame = co_await rs_dev.async_get_depth_frame();
      // If there's a segfault, this maybe to blame, removing const from const void*
      cv::Mat image(cv::Size(640, 480), CV_8UC3, const_cast<void*>(rgb_frame.get_data()), cv::Mat::AUTO_STEP);
      auto object_found = classify(classifier, image);

      if (object_found == 1 or object_found == 2) {
        spdlog::critical("Detected arrow, type {:d}", object_found);
        current_target = Target{.type=Target::Type::ARROW_SIGHTED};
        // Arrow found
        std::array<float, 4> crop_rectangle = get_bounding_box(classifier);

        auto lock_distance = get_depth_lock(depth_frame, crop_rectangle);
        if (lock_distance) {
          spdlog::critical("Obtained lock, distance {:f}", *lock_distance);
          // Set AP Target with distance
          current_target.type = Target::Type::ARROW_LOCKED;

          std::array<float, 3> forward_right_down{*lock_distance, 0.0f, 0.0f};
          std::array<float, 3> ref_arrow;
          auto current_position = mi.local_position();
          std::transform(begin(forward_right_down),
                         end(forward_right_down),
                         begin(current_position),
                         begin(ref_arrow),
                         std::plus<float>());
          current_target.location.emplace(ref_arrow);

          co_await mi.set_target_position_local(std::span(forward_right_down));
          continue;
        }
        
        std::array<float, 3> vel_forward_right_down{};
        co_await mi.set_target_velocity(std::span(vel_forward_right_down));
        continue;
      }

      // If a target is found, check for duplicates against visited_targets
      // make a new target, if unique
      // If depth can lock it, set a target on it with AP
      // Otherwise, reduce velocity and change heading to match target

      // If no target is found, maintain heading and loop
      std::array<float, 3> vel_forward_right_down{};
      co_await mi.set_target_velocity(std::span(vel_forward_right_down));
      continue;
    }

    // There's already a target, check target
    if (current_target.type == Target::Type::ARROW_SIGHTED or current_target.type == Target::Type::CONE_SIGHTED) {
      // Check if realsense depth can give a lock on the location
      // Lock the target by setting a target on AP
      // If not, set the velocity low, match heading and carefully approach target

      auto rgb_frame = co_await rs_dev.async_get_rgb_frame(this_exec);
      auto depth_frame = co_await rs_dev.async_get_depth_frame();
      cv::Mat image(cv::Size(640, 480), CV_8UC3, const_cast<void*>(rgb_frame.get_data()));
      size_t object_found = classify(classifier, image);
      // Arrows found
      if (object_found == 1 or object_found == 2) {
        spdlog::critical("Detected arrow, type {:d}", object_found);
        std::array<float, 4> crop_rectangle = get_bounding_box(classifier);
        auto lock_distance = get_depth_lock(depth_frame, crop_rectangle);
        if (lock_distance) {
          spdlog::critical("Obtained lock, distance {:f}", *lock_distance);
          current_target.type = Target::Type::ARROW_LOCKED;
          std::array<float, 3> forward_right_down{*lock_distance, 0.0f, 0.0f};
          std::array<float, 3> ref_arrow;
          auto current_position = mi.local_position();
          std::transform(begin(forward_right_down),
                         end(forward_right_down),
                         begin(current_position),
                         begin(ref_arrow), std::plus<float>());
          current_target.location.emplace(ref_arrow);

          co_await mi.set_target_position_local(std::span(forward_right_down));
          continue;
        }
      }
      std::array<float, 3> vel_forward_right_down{};
      co_await mi.set_target_velocity(std::span(vel_forward_right_down));
      continue;
    }
    if (current_target.type == Target::Type::ARROW_LOCKED or current_target.type == Target::Type::CONE_LOCKED) {
      // Check if the target approach, reduce velocity if needed,
      // Get the distance to target, cutoff appraoch and wait if needed
      // Don't continue, await the wait timer instead
      // Turn and set heading, clear current_target, push it to visited_targets
      auto current_position = mi.local_position();
      float approach_distance = std::inner_product(
        begin(current_position),
        end(current_position),
        begin(*current_target.location),
        0.0f,
        std::plus<>{},
        [](const float& a, const float& b) { return (a - b) * (a - b); });
      if (approach_distance < 10) {
        spdlog::critical("Target approach complete (distance {:f}), stopping", approach_distance);
        std::array<float, 3> stop_vel {0.0f, 0.0f, 0.0f};
        co_await mi.set_target_velocity(std::span(stop_vel));
        timer.expires_after(10s);
        co_await timer.async_wait(use_nothrow_awaitable);
        // TODO: Change heading
        // visited_targets.emplace(current_target);
        current_target = Target{.type=Target::Type::HEADING};
        continue;
      }
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
  args.add_argument("--no-avoid").default_value(false).implicit_value(true).help("Disable obstacle avoidance behaviour");

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
