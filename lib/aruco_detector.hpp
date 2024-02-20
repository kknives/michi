#pragma once

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <memory>
#include <spdlog/spdlog.h>
#include "classification_model.hpp"

struct ArucoDetectionResult {
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
              dictionary(cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50))),
              markersize(0.15) {
        //dictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
    }

};

class ArucoDetector {
public:
    explicit ArucoDetector(const ArucoParams& arucoParams)
                : m_aruco_params(arucoParams) {}


    friend ClassificationModel::Detection model_classify(ArucoDetector &detector, cv::Mat& image, float threshold);
    
    const ArucoDetectionResult& get_detection_result() const {
        return m_detection_result;
    }

    static ArucoDetector make_akash5_model(const std::string& s) {
        ArucoParams params;
        return ArucoDetector(params);
    }

    friend cv::Rect model_get_bounding_box(const ArucoDetector& detector) {
        auto m_detection_result = detector.get_detection_result();
        assert(!m_detection_result.ids.empty() && "No markers detected");
        std::vector<cv::Point2f>& corners = m_detection_result.corners[0];
        cv::Rect m_bounding_box = cv::boundingRect(corners);
        return m_bounding_box;
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

        if (image.empty()) {
            spdlog::error("Error: Input image is empty.");
            return false;
        }

        cv::aruco::detectMarkers(image, m_aruco_params.dictionary, corners, ids);

        if (!ids.empty()) {

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, m_aruco_params.markersize, m_aruco_params.camera_mat, m_aruco_params.distcoeffs, rvecs, tvecs);
            m_detection_result = {ids, corners, rvecs, tvecs};
            return true;
        }

        return false;
    }

private:
    ArucoParams m_aruco_params;
    ArucoDetectionResult m_detection_result;
};

ClassificationModel::Detection model_classify(ArucoDetector &detector, cv::Mat& image, float threshold) {
    if (detector.detect_markers(image)) {
        return ClassificationModel::Detection::ARUCO;
    } else {
        return ClassificationModel::Detection::NONE;
    }
}
