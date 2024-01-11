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

TEST(test_yolov8_arrow, IdentifiesBoundingBoxCorrectly) {
  auto yac = Yolov8ArrowClassifier::make_mohnish7_model("lib/model7.onnx");
  cv::Mat image = cv::imread("tests/sample_right_arrow.jpg");

  EXPECT_EQ(model_classify(yac, image, 0.8f), ClassificationModel::Detection::ARROW_RIGHT);
  auto bb1 = model_get_bounding_box(yac);
  cv::Rect expected_bb1(235, 176, 159, 109);
  EXPECT_GT(bb1.area(), 0);
  EXPECT_GT(bb1.height, 0);
  EXPECT_GT(bb1.width, 0);
  EXPECT_EQ(bb1, expected_bb1);

  cv::Mat image2 = cv::imread("tests/sample_left_arrow.jpg");
  EXPECT_EQ(model_classify(yac, image2, 0.8f), ClassificationModel::Detection::ARROW_LEFT);

  auto bb2 = model_get_bounding_box(yac);
  cv::Rect expected_bb2(246, 160, 158, 112);
  EXPECT_GT(bb2.area(), 0);
  EXPECT_GT(bb2.height, 0);
  EXPECT_GT(bb2.width, 0);
  EXPECT_EQ(bb2, expected_bb2);
}
