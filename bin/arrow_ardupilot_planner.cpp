#include <argparse/argparse.hpp>

#include "ardupilot_interface.hpp"
#include <cstdint>
#include <opencv4/opencv2/opencv.hpp>
#include "common.hpp"
#include "opencv2/core.hpp"
#include "realsense_generator.hpp"
#include "classification_model.hpp"
#include "mobilenet_arrow.hpp"
#include "arrow_state_machine.hpp"
#include "yolov8_arrow.hpp"
#include <asio/detached.hpp>
#include <asio/this_coro.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>
#include <memory>

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

  float hfov_deg = (fov[0] * 180.0f) / M_PI;
  std::string store;
  int rays = distances.size();
  for (int i = 1; i <= rays; i++) {
    pcl::PointWithRange ray;
    int idx = i * (hfov_deg / 72.0f);
    rg_img.get1dPointAverage(idx, 1, 1, 58, 58, ray);
    if (std::isinf(ray.range)) distances[i-1] = UINT16_MAX;
    else distances[i - 1] = uint16_t(ray.range*100);
    distances[i - 1] = distances[i - 1] ? distances[i - 1] : UINT16_MAX;
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

auto locate_obstacles(rs2::points& points, auto& mi, std::span<float, 2> fov) -> asio::awaitable<void> {
  spdlog::debug("Inside locate_obstacles");
  asio::steady_timer timer(co_await asio::this_coro::executor);
  
  tPclPtr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass_filter;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  std::array<uint16_t, 72> distances;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // for (;;) {
    spdlog::debug("Got points");
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

    float hfov_deg = (fov[0] * 180.0f) / M_PI;
    float vfov_deg = (fov[1] * 180.0f) / M_PI;
    co_await mi->set_obstacle_distance(
      std::span(distances), hfov_deg / 72.0f, 17.5f, 300.0f, -0.5f * hfov_deg);
  //   timer.expires_after(1s);
  //   co_await timer.async_wait(use_nothrow_awaitable);
  // }
}

auto
mission2(auto& mi,
         std::shared_ptr<RealsenseDevice> rs_dev,
         std::span<float, 2> fov) -> asio::awaitable<void>
{
  auto this_exec = co_await asio::this_coro::executor;

  std::optional<ClassificationModel> uninit_classifier;
  if (args.get("--model") == "mohnish4") {
    uninit_classifier.emplace(ClassificationModel(MobilenetArrowClassifier::make_mohnish4_model(args.get("model_path"))));
  } else if (args.get("--model") == "waseem2") {
    uninit_classifier.emplace(ClassificationModel(MobilenetArrowClassifier::make_waseem2_model(args.get("model_path"))));
  } else {
    uninit_classifier.emplace(ClassificationModel(Yolov8ArrowClassifier::make_mohnish7_model(args.get("model_path"))));
  }
  ClassificationModel classifier(std::move(uninit_classifier.value()));
  co_await mi->set_guided_mode();
  co_await mi->set_armed();
  asio::steady_timer timer(this_exec);
  int targets = 0;

  ArrowStateMachine sm(classifier, args.get<float>("-t"));
  Vector3f last_target(0.0f, 0.0f, 0.0f);
  float last_yaw = 0.0f;
  spdlog::info("Starting mission2");
  while (true) {
    auto rgb_frame = co_await rs_dev->async_get_rgb_frame();
    auto depth_frame = co_await rs_dev->async_get_depth_frame();
    cv::Mat image(cv::Size(640, 480), CV_8UC3, const_cast<void*>(rgb_frame.get_data()));
    // cv::imwrite("/tmp/im"+std::to_string(i)+".jpg", depth_frame_mat);
    auto points = co_await rs_dev->async_get_points();

    // TODO: add a constexpr if to disable obstacle avoidance
    co_await locate_obstacles(points, mi, fov);

    float current_yaw_deg = mi->heading();
    // Initialize the monadic interface for the SM
    ImpureInterface sm_monad(mi->local_position(), current_yaw_deg);
    spdlog::info("YAW: {}", current_yaw_deg);
    if (sm.next(sm_monad, image, depth_frame)) {
      co_await mi->set_disarmed(); // disarm
      co_return;
    }
    if (sm_monad.output.delay_sec) {
      spdlog::critical("Arrived at target, HOLD for {} seconds",
                       sm_monad.output.delay_sec);
      co_await mi->set_hold_mode();
      timer.expires_after(std::chrono::seconds(sm_monad.output.delay_sec));
      co_await timer.async_wait(use_nothrow_awaitable);
      co_await mi->set_guided_mode();
    }
    spdlog::debug("Monad O/P target: {}", sm_monad.output.target_xyz_pos_local);
    if (sm_monad.output.target_xyz_pos_local != Vector3f(0.0f, 0.0f, 0.0f) and
        sm_monad.output.target_xyz_pos_local != last_target) {
      spdlog::critical("Changing targets, new: {}",
                       sm_monad.output.target_xyz_pos_local);
      std::array<float, 3> target_xyz{
        sm_monad.output.target_xyz_pos_local[0],
        sm_monad.output.target_xyz_pos_local[1],
        sm_monad.output.target_xyz_pos_local[2]
      };
      last_target = sm_monad.output.target_xyz_pos_local;
      targets++;
      co_await mi->set_target_position_local(target_xyz);
    }
    else {
      // set target yaw here
      float yaw_radian = (sm_monad.output.yaw* M_PI)/180.0f;
      Eigen::Quaternionf rot(Eigen::AngleAxis<float>(yaw_radian, Eigen::Vector3f::UnitZ()));
      std::array<float, 4> quaternion_parameters { rot.w(), rot.x(), rot.y(), rot.z() };
      co_await mi->set_target_attitude(quaternion_parameters, 0.1f);

      if (int(sm_monad.output.yaw) != int(current_yaw_deg)) {
      spdlog::critical(
        "Turning to {}°: {}", sm_monad.output.yaw, quaternion_parameters);
        // Wait for turning to complete
        // timer.expires_after(14s);
        // co_await timer.async_wait(use_nothrow_awaitable);
      }
    }
    if (targets == 0) {
      // Move the rover forward initially
      const float initial_forward_vel_x = 0.1f;
      std::array<float, 3> target_vel_xyz{ initial_forward_vel_x, 0.0f, 0.0f };
      co_await mi->set_target_velocity(target_vel_xyz);
    }
    timer.expires_after(800ms);
    co_await timer.async_wait(use_nothrow_awaitable);
  }
}

int main(int argc, char* argv[]) {
  args.add_argument("model_path").help("Path to arrow classification model (eg. w_model2.onnx)");
  args.add_argument("ardupilot").help("Serial port (eg. /dev/ttyUSB0) connected to Pixhawk's TELEMETRY2");
  args.add_argument("-m", "--model").default_value(std::string("yolov8")).action([](const std::string& value) {
    static const std::vector<std::string> choices = { "waseem2", "mohnish4", "yolov8"};
    if (std::find(choices.begin(), choices.end(), value) != choices.end()) {
      return value;
    }
    return std::string{ "yolov8" };
  }).help("model to use for arrow classification");
  args.add_argument("--no-avoid").default_value(false).implicit_value(true).help("Disable obstacle avoidance behaviour");
  args.add_argument("-t", "--threshold").default_value(0.7f).help("Threshold for arrow detections (confidence > threshold => arrow detected)").scan<'g', float>();

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

  print("{}\n",banner);
  spdlog::info("Starting ArrowArdupilotPlanner version {}", "version_string");
  switch (log_verbosity) {
    case 0:
    spdlog::set_level(spdlog::level::info);
    break;
    case 1:
    spdlog::info("Verbosity 1: Logging debug messages");
    spdlog::set_level(spdlog::level::debug);
    break;
    default:
    spdlog::info("Verbosity 2: Logging trace messages");
    spdlog::set_level(spdlog::level::trace);
  }

  asio::io_context io_ctx;
  spdlog::trace("asio io_context setup");

  tcp::socket ap_socket(io_ctx);
  ap_socket.connect(*tcp::resolver(io_ctx).resolve("0.0.0.0", "5762", tcp::resolver::passive));
  auto mi = std::make_shared<MavlinkInterface<tcp::socket>>((std::move(ap_socket)));

  auto [rs_pipe, fovh, fovv] = *setup_device().or_else([] (std::error_code e) {
    spdlog::error("Couldn't setup realsense device: {}", e.message());
  });
  std::array<float, 2> fov = {fovh, fovv};
  auto rs_dev = std::make_shared<RealsenseDevice>(rs_pipe, io_ctx);

  asio::co_spawn(
    io_ctx,
    mission2(mi, rs_dev, std::span(fov)),
    [](std::exception_ptr p) {
      if (p) {
        try {
          std::rethrow_exception(p);
        } catch (const std::exception& e) {
          spdlog::error("Mission coroutine threw exception: {}",
                        e.what());
        }
      }
  });
  asio::co_spawn(
    io_ctx,
    mi->loop(),
    [](std::exception_ptr p, tResult<void> r) {
      if (p) {
        try {
          std::rethrow_exception(p);
        } catch (const std::exception& e) {
          spdlog::error("Mavlink loop coroutine threw exception: {}",
                        e.what());
        }
      }
      r.map_error([](std::error_code e) {
        spdlog::error("Mavlink loop coroutine faced error: {}: {}",
                      e.category().name(),
                      e.message());
      });
  });


  spdlog::trace("running asio io_context");
  io_ctx.run();
}
