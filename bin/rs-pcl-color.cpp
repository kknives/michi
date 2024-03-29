// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <algorithm>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>


#include <Eigen/Geometry>

#include <pcl/common/angles.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/search/octree.h>
#include <pcl/range_image/range_image.h>

#include <pcl/visualization/cloud_viewer.h>


using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

void calculate_obstacle_distances(pcl_ptr pc, std::vector<float>& distances, float fov[2]);

pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

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

int main(int argc, char * argv[]) try
{
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    rs2::decimation_filter dec_filter;
    rs2::temporal_filter temp_filter;
    rs2::hole_filling_filter hole_filter;

    rs2::pipeline pipe;
    rs2::config stream_config;
    stream_config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, 0, 424, 240, rs2_format::RS2_FORMAT_Z16, 30);
    rs2::pipeline_profile selection = pipe.start(stream_config);
    auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto i = depth_stream.get_intrinsics();
    float fov[2];
    rs2_fov(&i, fov);
    fov[0] = (fov[0] * M_PI)/180.0f;
    fov[1] = (fov[1] * M_PI)/180.0f;
    std::cout << fov[0] << ", " << fov[1] << "\n";

    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();

    auto depth = frames.get_depth_frame();
    rs2::frame filtered = depth;

    filtered = dec_filter.process(filtered);
    filtered = temp_filter.process(filtered);
    filtered = hole_filter.process(filtered);

    depth = filtered;

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);

    auto pcl_points = points_to_pcl(points);

    pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_points);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*cloud_filtered);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud_filtered);
    voxel_filter.setLeafSize(0.01f,0.01f,0.01f);
    voxel_filter.filter(*cloud_filtered);
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr before_inliers (new pcl::PointIndices);
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::IndicesPtr indices (new std::vector <int>);

    // pcl::SACSegmentation<pcl::PointXYZ> seg;
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setDistanceThreshold(1.0);
    // seg.setInputCloud(cloud_filtered);
    // seg.segment(*inliers, *coefficients);
    
      pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
      pmf.setInputCloud (cloud_filtered);
      pmf.setMaxWindowSize (50);
      pmf.setSlope (1.0f);
      pmf.setInitialDistance (0.0f);
      pmf.setMaxDistance (3.0f);
      pmf.extract (before_inliers->indices);

pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::PointXYZ point;
    // std::cout << before_inliers->indices[0] << '\n';
  point.x = 1;

  point.y = 1;

  point.z = 0;

  foreground_points->points.push_back(point);

   pcl::MinCutSegmentation<pcl::PointXYZ> seg;
    seg.setInputCloud (cloud_filtered);
    seg.setIndices (before_inliers);
    seg.setForegroundPoints(foreground_points);
      seg.setSigma (0.25);
      seg.setRadius (3.0433856);

      seg.setNumberOfNeighbours (14);
    std::vector <pcl::PointIndices> clusters;

  seg.extract (clusters);

    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // pcl_ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);

    // extract.setInputCloud(cloud_filtered);
    // extract.setIndices(inliers);
    // extract.setNegative(true);
    // extract.filter(*cloud_p);

    // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(120.0f);
    // octree.setInputCloud(cloud_p);
    // octree.addPointsFromInputCloud();
    // pcl::PointIndices::Ptr region (new pcl::pointIndices);

    std::vector<float> distances(72);
    calculate_obstacle_distances(pcl_points, distances, fov);
    // octree.boxSearch

    pcl::visualization::CloudViewer viewer("CloudViewer");
    viewer.showCloud(pcl_points, "Filtered Cloud");
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
    viewer.showCloud(colored_cloud, "Ground Plane");
    std::cout << "I made it" << '\n';

    viewer.runOnVisualizationThreadOnce([&fov](pcl::visualization::PCLVisualizer& viewer) {
        Eigen::Affine3f m = Eigen::Affine3f::Identity();
        viewer.removeAllCoordinateSystems();
        viewer.addCoordinateSystem(1.0f, m);
        viewer.setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
        viewer.setCameraFieldOfView(1.01256);
        viewer.addCube(-0.166152f, 0.0f,
                       0.0f, 1.28f,
                       0.0f,  4.0f,
                       1.0f,  0.0f, 0.0f);
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
        // viewer.setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1.0f,0.1f,0.3f, "Ground Plane");
    });

    while(!viewer.wasStopped());

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void calculate_obstacle_distances(pcl_ptr pc, std::vector<float>& distances, float fov[2])
{
    Eigen::Affine3f rs_pose = (Eigen::Affine3f) Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    float angular_res = (float) (1.0f * (M_PI/180.0f));
    pcl::RangeImage::CoordinateFrame coord_frame = pcl::RangeImage::CAMERA_FRAME;
    float noise_lvl = 0.0f;
    float min_range = 0.0f;
    int border = 0;
    pcl::RangeImage rg_img;
    rg_img.createFromPointCloud(*pc, angular_res, fov[0], fov[1], rs_pose, coord_frame, noise_lvl, min_range, border);
    float * goods = rg_img.getRangesArray();
    std::cout<<rg_img<<"\n";
    std::cout << goods << "\n";

    int rays = distances.size();
    std::string store;
    for (int i = 1; i <= rays; i++) {
        pcl::PointWithRange ray;
        int idx = i*(88.0f/72.0f);
        rg_img.get1dPointAverage(idx, 1, 1, 58, 58, ray);
        distances[i-1] = ray.range;
        if (std::isinf(ray.range)) {
          store.append(" ");
        }
        else {
          store.append("▅");
        }
    }
    // float alpha = hfov / 72.0f;
    // float h = 4.0f;
    // float x = h*std::tan(hfov/2.0f);
    // std::iota(begin(distances), end(distances), 1);
    // std::transform(begin(distances), end(distances), begin(distances), [alpha](float a) { return a*alpha; });
    // std::transform(begin(distances), end(distances), begin(distances), [x, hfov, h](float a) { return x - (h*std::tan((hfov/2.0)-a));
    // });
    std::cout << "▏" << store << "▕\n";
    if (false) {
        printf("%f", goods[1]);
        for (int i = 1; i < rg_img.width * rg_img.height; i++) printf(" %f", *(goods+i));
    }
    std::copy(begin(distances), end(distances), std::ostream_iterator<float>(std::cout, " "));
    std::cout << "\n";
}
// Registers the state variable and callbacks to allow mouse control of the pointcloud
