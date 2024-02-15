#include <gtest/gtest.h>
#include "aruco_detector.hpp"

#include <gtest/gtest.h>
#include "aruco_detector.hpp"

#include <gtest/gtest.h>
#include "aruco_detector.hpp"

TEST(ArucoDetectorTest, TestGetPose) {
    ArucoParams params;
    ArucoDetector detector(params);

    cv::Mat image = cv::imread("tests/sample_aruco_42.jpg");
    std::cout << "Image size: " << image.size().width << "x" << image.size().height << std::endl;
    bool detection_success = detector.detect_markers(image);

    EXPECT_TRUE(detection_success);
    if (detection_success) {
        auto pose = detector.get_pose();

        cv::Vec3d RVEC = {3.14801, 0.0564981, -0.0176922};
        cv::Vec3d TVEC = {0.5484, -0.055659, 0.911743};

        EXPECT_NEAR(pose.first[0], RVEC[0], 1e-5);
        EXPECT_NEAR(pose.second[0], TVEC[0], 1e-5);
    }
}

TEST(ArucoDetectorTest, TestDetectedId) {
    ArucoParams params;
    ArucoDetector detector(params);

    cv::Mat test_image = cv::imread("tests/sample_aruco_42.jpg");
    bool detection_success = detector.detect_markers(test_image);

    EXPECT_TRUE(detection_success);
    if (detection_success) {
        auto detection_result = detector.get_detection_result();
        EXPECT_EQ(detection_result.ids[0], 42);
    }
}