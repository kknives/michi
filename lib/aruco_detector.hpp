#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <memory>
#include "classification_model.hpp"

struct ArucoParams {
    cv::Mat camera_mat;
    cv::Mat distcoeffs;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    float markersize;

    ArucoParams()
            : cameraMatrix((cv::Mat_<double>(3, 3) << 216.357407, 0, 214.624283,
            0, 216.357407, 115.447563,
            0, 0, 1)),
              distCoeffs(cv::Mat::zeros(5, 1, CV_64F)),
              dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50)),
              markerSize(0.15) {}
};

class ArucoDetector;
ClassificationModel::Detection model_classify(ArucoDetector &detector, cv::Mat& image, float threshold);
cv::Rect ModelGetBoundingBox(const ArucoDetector& detector);

class ArucoDetector {
public:
    ArucoDetector(const ArucoParams& arucoParams)
            : m_aruco_params(arucoParams), cameraMatrix(m_aruco_params.cameraMatrix),
              dictionaryType(m_aruco_params.dictionaryType), markerSize(m_aruco_params.markerSize) {}

    friend ClassificationModel::Detection model_classify(ArucoDetector &detector, cv::Mat& image, float threshold);
    friend cv::Rect ModelGetBoundingBox(const ArucoDetector& detector);

    cv::Rect model_get_bounding_box() {
        assert(!detection_result.ids.empty() && "No markers detected");
        std::vector<cv::Point2f>& corners = detection_result.corners[0];
        cv::Rect boundingBox = cv::boundingRect(corners);
        return boundingBox;
    }

    std::pair<cv::Vec3d, cv::Vec3d> get_pose() {
        assert(!detection_result.ids.empty() && "No markers detected");
        cv::Vec3d rvec = m_detection_result.rvecs[0];
        cv::Vec3d tvec = m_detection_result.tvecs[0];
        return std::make_pair(rvec, tvec);
    }

private:
    ArucoParams m_aruco_params;
    ArUcoDetectionResult m_detection_result;

    bool ArucoDetector::DetectMarkers(cv::Mat& image) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<cv::Vec3d> rvecs, tvecs;

        if (image.empty()) {
            std::cerr << "Error: Input image is empty." << std::endl;
            return false;
        }

        cv::aruco::detectMarkers(image, m_aruco_params.dictionary, corners, ids);

        if (!ids.empty()) {
            cv::aruco::estimatePoseSingleMarkers(corners, m_aruco_params.markersize, m_aruco_params.camera_mat, m_aruco_params.distcoeffs, rvecs, tvecs);
            m_detection_result = {ids, corners, rvecs, tvecs};
            return true;
        }

        return false;
    }
};

ClassificationModel::Detection model_classify(ArucoDetector &detector, cv::Mat& image, float threshold) {
    ArucoParams params;
    if (detector.DetectMarkers(image, params)) {
        return ClassificationModel::Detection::ARUCO;
    } else {
        return ClassificationModel::Detection::NONE;
    }
}