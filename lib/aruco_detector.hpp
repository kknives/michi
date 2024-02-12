#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <memory>
#include "classification_model.hpp"

struct ArucoParams {
    double markerSize;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
};

class Aruco_Detector {
public:
    Aruco_Detector(const cv::Mat& cameraMatrix, int dictionaryType, float markerSize = 0.15)
        : cameraMatrix(cameraMatrix), dictionaryType(dictionaryType), markerSize(markerSize) {}

    friend ClassificationModel::Detection model_classify(Aruco_Detector &detector, cv::Mat& image, float threshold) {
        ArucoParams params;
        if (detector.detectMarkers(image, params)) {
            return ClassificationModel::Detection::ARUCO;
        } else {
            return ClassificationModel::Detection::NONE;
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

    bool detectMarkers(cv::Mat& image, ArucoParams& params) {
        cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionaryType);

        if (image.empty()) {
            std::cerr << "Error: Input image is empty." << std::endl;
            return false;
        }

        cv::aruco::detectMarkers(image, dictionary, params.corners, params.ids, cv::aruco::DetectorParameters::create());

        if (!params.ids.empty()) {
            cv::aruco::estimatePoseSingleMarkers(params.corners, markerSize, cameraMatrix, distCoeffs, params.rvecs, params.tvecs);
            return true;
        }

        return false;
    }
};
