#include <gtest/gtest.h>
#include "yolov8_arrow.hpp"
#include "classification_model.hpp"
#include <opencv4/opencv2/opencv.hpp>

TEST(test_yolov8_arrow, DetectsArrowCorrectly) {
  auto yac = Yolov8ArrowClassifier::make_mohnish7_model("lib/model7.onnx");
  cv::Mat image = cv::imread("tests/sample_right_arrow.jpg");
  EXPECT_EQ(model_classify(yac, image, 0.8f), ClassificationModel::Detection::ARROW_RIGHT);

  cv::Mat image2 = cv::imread("tests/sample_left_arrow.jpg");
  EXPECT_EQ(model_classify(yac, image2, 0.8f), ClassificationModel::Detection::ARROW_LEFT);
}
