// In Honour of the boundless Rudra
// Shashwat Ganesh, R24

#include <argparse/argparse.hpp>
#include <asio/detached.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>
#include <Eigen/Dense>
#include <opencv4/opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <execution>
#include "common.hpp"
#include "ananta.hpp"
#include "gazebo_interface.hpp"
#include "realsense_generator.hpp"
#include "ekf.hpp"

using Eigen::Matrix
using fmt::print;
using tPointcloud = pcl::PointCloud<pcl::PointXYZ>;
using namespace std::literals::chrono_literals;
static argparse::ArgumentParser args("TubePlanner");

class RealsenseImuPolicy {
  Eigen::Matrix<float, 3, 2> m_imu;
  const Eigen::Matrix<float, 3, 3> m_rot;
  int m_reads = 0;
  public:
    RealsenseImuPolicy() : m_rot{{0, 0, 1},
                                 {1, 0, 0},
                                 {0, 1, 0}} {}
      protected:
    using If = RealsenseDevice;
    auto imu_update(std::shared_ptr<If> rs_dev) -> asio::awaitable<void> {
      m_imu = co_await rs_dev->async_get_imu();
      m_imu = m_rot * m_imu;
      m_reads = 2;
    }
    auto imu_linear_acceleration(std::shared_ptr<If> rs_dev) -> asio::awaitable<Eigen::Vector3f> {
      if (not m_reads) {
        co_await imu_update(rs_dev);
      }
      m_reads--;
      co_return m_imu.col(0);
    }
    auto imu_angular_velocity(std::shared_ptr<If> rs_dev) -> asio::awaitable<Eigen::Vector3f> {
      if (not m_reads) {
        co_await imu_update(rs_dev);
      }
      m_reads--;
      co_return m_imu.col(1);
    }
};
class RealsenseDepthCamPolicy {
  pcl::VoxelGrid<pcl::PointXYZ> m_voxel_filter;
  pcl::PassThrough<pcl::PointXYZ> m_passthrough_filter;
  tPointcloud::Ptr points_to_pcl(const rs2::points& points)
  {
      tPointcloud::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

      auto sp = points.get_profile().as<rs2::video_stream_profile>();
      cloud->width = sp.width();
      cloud->height = sp.height();
      cloud->is_dense = false;
      cloud->points.resize(points.size());
      auto ptr = points.get_vertices();
      std::transform(std::execution::par_unseq,
                     points.get_vertices(),
                     points.get_vertices() + points.size(),
                     cloud->points.begin(),
                     [](auto& ptr) {
                       pcl::PointXYZ p;
                       p.x = ptr.z;
                       p.y = -ptr.x;
                       p.z = -ptr.y;
                       return p;
                     });
      return cloud;
  }
  protected:
    using If = RealsenseDevice;
    auto async_get_rgb_frame(std::shared_ptr<If> rs_dev)
      -> asio::awaitable<cv::Mat>
    {
      auto rgb_frame = co_await rs_dev->async_get_rgb_frame();
      cv::Mat image(
        cv::Size(640, 480), CV_8UC3, const_cast<void*>(rgb_frame.get_data()));
      co_return image;
    }
    auto async_get_depth_frame(std::shared_ptr<If> rs_dev)
      -> asio::awaitable<cv::Mat>
    {
      auto depth_frame = co_await rs_dev->async_get_depth_frame();
      cv::Mat image(
        cv::Size(640, 480), CV_8UC3, const_cast<void*>(depth_frame.get_data()));
      co_return image;
    }
    auto async_get_pointcloud(std::shared_ptr<If> rs_dev)
      -> asio::awaitable<tPointcloud::Ptr>
    {
      auto points = co_await rs_dev->async_get_points();
      spdlog::debug("Got points");
      auto pcl_cloud = points_to_pcl(points);

      if (pcl_cloud->size() < 50'000) co_return pcl_cloud;
      spdlog::info("Before filtering: {} points", pcl_cloud->size());

      m_passthrough_filter.setInputCloud(pcl_cloud);
      m_passthrough_filter.setFilterFieldName("x");
      m_passthrough_filter.setFilterLimits(0.0, 10.0);
      m_passthrough_filter.filter(*pcl_cloud);

      m_passthrough_filter.setInputCloud(pcl_cloud);
      m_passthrough_filter.setFilterFieldName("z");
      m_passthrough_filter.setFilterLimits(-1.0, 1.0);
      m_passthrough_filter.filter(*pcl_cloud);

      m_voxel_filter.setInputCloud(pcl_cloud);
      m_voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f);
      m_voxel_filter.filter(*pcl_cloud);

      spdlog::info("After filtering: {} points", pcl_cloud->size());
      co_return pcl_cloud;
    }
};
class GazeboDepthCamPolicy {
protected:
  using If = GazeboInterface;
  auto async_get_pointcloud(std::shared_ptr<If> gi)
    -> asio::awaitable<tPointcloud::Ptr>
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

    gz::msgs::PointCloudPacked packed_msg = gi->depth_camera_pointcloud();
    spdlog::info("{}×{} pointcloud", packed_msg.height(), packed_msg.width());
    int point_step = packed_msg.point_step();
    int num_points = packed_msg.data().size() / point_step;

    cloud->width = num_points;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    for (int i = 0; i < num_points; ++i) {
      const char* point_data = packed_msg.data().c_str() + i * point_step;
      const float* point_fields = reinterpret_cast<const float*>(point_data);

      cloud->points[i].x = point_fields[0];
      cloud->points[i].y = point_fields[1];
      cloud->points[i].z = point_fields[2];
    }
    co_return cloud;
  }
};
class GazeboImuPolicy {
  protected:
    using If = GazeboInterface;
    auto imu_linear_acceleration(std::shared_ptr<If> gi) -> asio::awaitable<Eigen::Vector3f> {
      co_return gi->imu_linear_acceleration();
    }
    auto imu_angular_velocity(std::shared_ptr<If> gi) -> asio::awaitable<Eigen::Vector3f> {
      co_return gi->imu_angular_velocity();
    }
};
class BlankOdomPolicy {
  protected:
    using If = std::nullptr_t;
    auto odometry_position(std::shared_ptr<If> gi) -> Eigen::Vector3f {
      return Eigen::Vector3f(0,0,0);
    }
    auto set_target_velocity(std::shared_ptr<If> gi,
                             Eigen::Vector3f linear,
                             Eigen::Vector3f angular) -> void
    {
      return;
    }
};
class GazeboOdomPolicy {
  protected:
    using If = GazeboInterface;
    auto odometry_position(std::shared_ptr<If> gi) -> Eigen::Vector3f {
      return gi->odometry_position();
    }
    auto set_target_velocity(std::shared_ptr<If> gi,
                             Eigen::Vector3f linear,
                             Eigen::Vector3f angular) -> void
    {
      return gi->set_target_velocity(linear, angular);
    }
};
template<typename DepthCamPolicy, typename BaseImuPolicy, typename OdomPolicy>
class AnantaMission
  : public DepthCamPolicy
  , public BaseImuPolicy
  , public OdomPolicy
{
  using DepthCamPolicy::async_get_pointcloud;
  using BaseImuPolicy::imu_linear_acceleration;
  using BaseImuPolicy::imu_angular_velocity;
  using OdomPolicy::odometry_position;
  using OdomPolicy::set_target_velocity;

  octomap::OcTree m_tree;
  octomap::Pointcloud m_map_cloud;
  std::shared_ptr<typename DepthCamPolicy::If> m_ci;
  std::shared_ptr<typename BaseImuPolicy::If> m_imu_if;
  std::shared_ptr<typename OdomPolicy::If> m_odom_if;
  int m_iterations;
public:
  AnantaMission(std::shared_ptr<typename DepthCamPolicy::If> ci,
                std::shared_ptr<typename BaseImuPolicy::If> imu_if,
                std::shared_ptr<typename OdomPolicy::If> odom_if)
    : m_tree(args.get<float>("-t"))
    , m_iterations(0)
    , m_ci(ci)
    , m_imu_if(imu_if)
    , m_odom_if(odom_if)
  {
  }
  auto loop() -> asio::awaitable<void>
  {
    auto this_exec = co_await asio::this_coro::executor;
    auto localization = EKF();
    asio::steady_timer timer(this_exec);

    timer.expires_after(3s);
    co_await timer.async_wait(use_nothrow_awaitable);
    while (true) {
      auto linear_accel = co_await imu_linear_acceleration(m_imu_if);
      auto angular_vel =  co_await imu_angular_velocity(m_imu_if);
      spdlog::info("IMU vel {} gyro {}",
                   linear_accel,
                   angular_vel);
      Eigen::Matrix<float, 2, 1> u =
        localization.control_input(linear_accel,
                                   angular_vel,
                                   odometry_position(m_odom_if));
      spdlog::info("Control input: {}", u);
      Eigen::Matrix<float, 4, 1> position_est;
      Eigen::Matrix<float, 4, 4> position_cov;
      std::tie(position_est, position_cov) = localization.run_ekf(u);
      spdlog::info("Position estimate: {}", position_est);

      octomap::point3d map_current_pos(
        position_est(0), position_est(1), 0);
      auto cam_cloud = co_await async_get_pointcloud(m_ci);
      m_map_cloud.reserve(cam_cloud->points.size());
      int nulls = 0;
      // std::transform(std::execution::par,
      //               cam_cloud->points.begin(),
      //               cam_cloud->points.end(),
      //               m_map_cloud.begin(),
      //               [](auto& point) {
      //                 // if (std::isinf(point.x) or std::isinf(point.y) or
      //                 //     std::isinf(point.z))
      //                 //   return;
      //                 return octomap::point3d(point.x, point.y, point.z);
      //               });
      for (const auto& point : cam_cloud->points) {
        if (std::isinf(point.x) or std::isinf(point.y) or
        std::isinf(point.z)) {
          nulls++;
          continue;
        }
        m_map_cloud.push_back(point.x, point.y, point.z);
      }
      m_tree.insertPointCloud(m_map_cloud, map_current_pos);
      spdlog::info("Got {} nulls. Inserted a cloud!", nulls);
      m_map_cloud.clear();

      m_iterations++;
      if (m_iterations == 1)
        spdlog::info("Wrote tree: {}", m_tree.write("m_tree.ot"));
      if constexpr (std::is_same<DepthCamPolicy, GazeboDepthCamPolicy>::value) {
        timer.expires_after(10ms);
        co_await timer.async_wait(use_nothrow_awaitable);
      }
    }

    co_return;
  }
};
int main(int argc, char* argv[]) {
  args.add_argument("model_path").help("Path to object classification model");
  args.add_argument("--sim").default_value(false).implicit_value(true).help(
    "Run in simulation mode");
  args.add_argument("-d", "--device")
    .default_value(std::string("/dev/ttyUSB0"))
    .help("Serial port to talk to rover");
  args.add_argument("-t", "--tree")
    .default_value(0.03f)
    .help("Occupancy map resolution (in metres)").scan<'g', float>();

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
  std::any mission;
  if (args.get<bool>("--sim")) {
    print("Starting in SIM mode\n\n");
    auto gi = std::make_shared<GazeboInterface>();

    mission = std::make_shared<
      AnantaMission<GazeboDepthCamPolicy, GazeboImuPolicy, GazeboOdomPolicy>>(gi, gi, gi);
    asio::co_spawn(io_ctx, gi->loop(), [](std::exception_ptr p) {
      if (p) {
        try {
          std::rethrow_exception(p);
        } catch (const std::exception& e) {
          spdlog::error("GazeboInterface loop coroutine threw exception: {}",
                        e.what());
        }
      }
    });
    asio::co_spawn(
      io_ctx,
      std::any_cast<std::shared_ptr<AnantaMission<GazeboDepthCamPolicy,
                                                  GazeboImuPolicy,
                                                  GazeboOdomPolicy>>>(mission)
        ->loop(),
      [](std::exception_ptr p) {
        if (p) {
          try {
            std::rethrow_exception(p);
          } catch (const std::exception& e) {
            spdlog::error("Mission loop coroutine threw exception: {}",
                          e.what());
          }
        }
      });
  }
  else {
    print("Starting in HW mode \n\n");
    // asio::serial_port dev_serial(io_ctx, args.get("-d"));
    // dev_serial.set_option(asio::serial_port_base::baud_rate(921600));
    // auto mi = std::make_shared<MavlinkInterface<asio::serial_port>>(std::move(dev_serial));

    auto [rs_pipe, fovh, fovv] =
      *setup_device(true).or_else([](std::error_code e) {
        spdlog::error("Couldn't setup realsense device: {}", e.message());
      });
    auto rs_dev = std::make_shared<RealsenseDevice>(rs_pipe, io_ctx);

    mission = std::make_shared<AnantaMission<RealsenseDepthCamPolicy,
                                             RealsenseImuPolicy,
                                             BlankOdomPolicy>>(rs_dev, rs_dev, std::make_shared<std::nullptr_t>());
    asio::co_spawn(
      io_ctx,
      std::any_cast<std::shared_ptr<AnantaMission<RealsenseDepthCamPolicy,
                                                  RealsenseImuPolicy,
                                                  BlankOdomPolicy>>>(mission)
        ->loop(),
      [](std::exception_ptr p) {
        if (p) {
          try {
            std::rethrow_exception(p);
          } catch (const std::exception& e) {
            spdlog::error("Mission loop coroutine threw exception: {}",
                          e.what());
          }
        }
      });
  }
  io_ctx.run();
}
