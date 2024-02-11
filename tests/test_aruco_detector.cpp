#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "aruco_detector.hpp"

TEST(ArucoDetectorTest, DetectArucoMarkerID42) {
cv::Mat testImage = cv::imread("sample_aruco_42.jpg");

int dictionaryType = cv::aruco::DICT_4X4_50;
Aruco_Detector arucoDetector(testImage, dictionaryType);

auto detection = arucoDetector.detectAruco(0.6f);

ASSERT_FALSE(detection.empty());

bool markerId42Detected = false;
for (const auto& marker : detection) {
if (marker.id == 42) {
markerId42Detected = true;
break;
}
}

ASSERT_TRUE(markerId42Detected);
}
