#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>

struct ArUcoDetectionResult {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
};

class Aruco_Detector {
public:
    Aruco_Detector(const cv::Mat& cameraMatrix, int dictionaryType,)
            : cameraMatrix(cameraMatrix), dictionaryType(dictionaryType) {}

    enum Detection {
        NONE = 0,
        ARUCO
    };

    Detection classify(cv::Mat& image, float threshold) {
        if (detectMarkers(image)) {
            return Detection::ARUCO;
        } else {
            return Detection::NONE;
        }
    }

    cv::Rect get_bounding_box() {
        return cv::Rect();
    }

private:
    cv::Mat cameraMatrix;
    int dictionaryType;
    float markerSize;
    ArUcoDetectionResult detectionResult;

    bool detectMarkers(cv::Mat& image) {
        cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionaryType);

        if (image.empty()) {
            std::cerr << "Error: Input image is empty." << std::endl;
            return false;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids, cv::aruco::DetectorParameters::create());

        if (!ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.15, cameraMatrix, distCoeffs, rvecs, tvecs);

            detectionResult.ids = ids;
            detectionResult.corners = corners;
            detectionResult.rvecs = rvecs;
            detectionResult.tvecs = tvecs;
            return true;
        }

        return false;
    }
};
