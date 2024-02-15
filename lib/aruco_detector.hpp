#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <memory>
#include <spdlog/spdlog.h>
#include "classification_model.hpp"

struct ArUcoDetectionResult {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
};

struct ArucoParams {
    cv::Mat camera_mat;
    cv::Mat distcoeffs;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    float markersize;

    ArucoParams()
            : camera_mat((cv::Mat_<double>(3, 3) << 604.5639, 0, 317.31656,
            0, 604.5807, 254.18544,
            0, 0, 1)),
              distcoeffs(cv::Mat::zeros(5, 1, CV_64F)),
              markersize(0.15) {
        dictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
    }
};

class ArucoDetector {
public:
    ArucoDetector(const ArucoParams& arucoParams)
            : m_aruco_params(arucoParams) {}

    friend ClassificationModel::Detection model_classify(ArucoDetector &detector, cv::Mat& image, float threshold);

    const ArUcoDetectionResult& get_detection_result() const {
        return m_detection_result;
    }

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

    bool detect_markers(cv::Mat& image) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<cv::Vec3d> rvecs, tvecs;

        if (image.empty()) {
            spdlog::error("Error: Input image is empty.");
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
    if (detector.detect_markers(image)) {
        return ClassificationModel::Detection::ARUCO;
    } else {
        return ClassificationModel::Detection::NONE;
    }
}
