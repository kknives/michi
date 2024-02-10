#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <memory>
#include "classification_model.hpp"

class Aruco_Detector : public ClassificationModel::dClassification {
public:
    Aruco_Detector(cv::VideoCapture& videoCapture, int dictionaryType) : inputVideo(videoCapture), dictionaryType(dictionaryType) {}

    ClassificationModel::Detection classify(cv::Mat& image, float threshold) override {
        if (detectMarkers(image)) {
            return ClassificationModel::Detection::ARUCO;
        } else {
            return ClassificationModel::Detection::NONE;
        }
    }

    cv::Rect get_bounding_box() override {
        return cv::Rect();
    }

private:
    int dictionaryType;
    ArUcoDetectionResult detectionResult;

    void detectMarkers(cv::Mat& image) {
        //Camera/Marker Parameters
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 216.357407, 0, 214.624283,
                0, 216.357407, 115.447563,
                0, 0, 1);
        cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionaryType);

        if (image.empty()) {
            std::cerr << "Error: Input image is empty." << std::endl;
            return;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        // If at least one marker is detected
        if (!ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, 0.15, cameraMatrix, distCoeffs, rvecs, tvecs);

            // Populate the detection result
            detectionResult.ids = ids;
            detectionResult.corners = corners;
            detectionResult.rvecs = rvecs;
            detectionResult.tvecs = tvecs;
        }
    }
};
