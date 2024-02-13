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
            : camera_mat((cv::Mat_<double>(3, 3) << 216.357407, 0, 214.624283,
            0, 216.357407, 115.447563,
            0, 0, 1)),
              distcoeffs(cv::Mat::zeros(5, 1, CV_64F)),
              dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50)),
              markersize(0.15) {}
};

class ArucoDetector {
public:
    ArucoDetector(const ArucoParams& arucoParams)
            : m_aruco_params(arucoParams) {}

    // Changed member variables to match ArucoParams
    friend ClassificationModel::Detection model_classify(ArucoDetector &detector, cv::Mat& image, float threshold);
    friend cv::Rect ModelGetBoundingBox(const ArucoDetector& detector);

    cv::Rect model_get_bounding_box() {
        assert(!m_detection_result.ids.empty() && "No markers detected");
        std::vector<cv::Point2f>& corners = m_detection_result.corners[0];
        cv::Rect boundingBox = cv::boundingRect(corners);
        return boundingBox;
    }

    std::pair<cv::Vec3d, cv::Vec3d> get_pose() {
        assert(!m_detection_result.ids.empty() && "No markers detected");
        cv::Vec3d rvec = m_detection_result.rvecs[0];
        cv::Vec3d tvec = m_detection_result.tvecs[0];
        return std::make_pair(rvec, tvec);
    }

    bool DetectMarkers(cv::Mat& image) {
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

private:
    ArucoParams m_aruco_params;
    ArUcoDetectionResult m_detection_result;
};

ClassificationModel::Detection model_classify(ArucoDetector &detector, cv::Mat& image, float threshold) {
    if (detector.DetectMarkers(image)) {
        return ClassificationModel::Detection::ARUCO;
    } else {
        return ClassificationModel::Detection::NONE;
    }
}
