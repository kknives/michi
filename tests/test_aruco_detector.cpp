#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include "aruco_detector.hpp"

TEST(ArucoDetectorTest, DetectArucoMarker) {
// Load test image containing Aruco marker with ID 42
cv::Mat testImage = cv::imread("sample_aruco_42.jpg");

// Create ArucoDetector object
int dictionaryType = cv::aruco::DICT_4X4_50;
Aruco_Detector arucoDetector(testImage, dictionaryType);

auto detection = arucoDetector.classify(testImage, 0.6f);

ASSERT_EQ(detection, ClassificationModel::Detection::ARUCO);
cv::Rect boundingBox = arucoDetector.get_bounding_box();

cv::Mat markerROI = testImage(boundingBox);

std::vector<int> markerIds;
std::vector<std::vector<cv::Point2f>> markerCorners;
cv::aruco::detectMarkers(markerROI, cv::aruco::getPredefinedDictionary(dictionaryType), markerCorners, markerIds);

// Check if marker with ID 42 is detected
bool markerId42Detected = false;
for (int id : markerIds) {
if (id == 42) {
markerId42Detected = true;
break;
}
}

// Assert that marker ID 42 is detected
ASSERT_TRUE(markerId42Detected);
}
