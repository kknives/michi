#include <gtest/gtest.h>
#include "mobilenet_arrow.hpp"
#include "classification_model.hpp"
#include <opencv4/opencv2/opencv.hpp>

TEST(test_mobilenet_arrow, ProcessesImageFrameCorrectly) {
  auto mac = ClassificationModel(MobilenetArrowClassifier::make_waseem2_model("lib/w_model2.onnx"));
  cv::Mat image = cv::imread("tests/sample_right_arrow.jpg");
  EXPECT_EQ(classify(mac, image, 0.3), ClassificationModel::Detection::ARROW_RIGHT);

  cv::Mat image2 = cv::imread("tests/sample_left_arrow.jpg");
  EXPECT_EQ(classify(mac, image2, 0.3), ClassificationModel::Detection::ARROW_LEFT);
}

TEST(test_mobilenet_arrow, ReturnsBoundingBox) {
  auto mac = ClassificationModel(MobilenetArrowClassifier::make_waseem2_model("lib/w_model2.onnx"));
  cv::Mat image = cv::imread("tests/sample_left_arrow.jpg");
  EXPECT_EQ(classify(mac, image, 0.3), ClassificationModel::Detection::ARROW_LEFT);
  auto bb = get_bounding_box(mac);

  EXPECT_GT(bb.area(), 0);
  EXPECT_GT(bb.x, 0);
  EXPECT_GT(bb.y, 0);
}
