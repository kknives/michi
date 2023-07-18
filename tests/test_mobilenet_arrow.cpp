#include <gtest/gtest.h>
#include "mobilenet_arrow.hpp"
#include "classification_model.hpp"
#include <opencv4/opencv2/opencv.hpp>

TEST(test_mobilenet_arrow, ProcessesImageFrameCorrectly) {
  auto mac = ClassificationModel(MobilenetArrowClassifier("lib/saved_model_checkpoint4.onnx"));
  cv::Mat image = cv::imread("tests/sample_left_arrow.jpg");
  EXPECT_EQ(classify(mac, image), 1);

  // FIXME
  // cv::Mat image2 = cv::imread("tests/sample_right_arrow.jpg");
  // EXPECT_EQ(mac.classify(image2), 2);
}

TEST(test_mobilenet_arrow, ReturnsBoundingBox) {
  MobilenetArrowClassifier mac("lib/saved_model_checkpoint4.onnx");
  cv::Mat image = cv::imread("tests/sample_left_arrow.jpg");
  EXPECT_EQ(mac.classify(image), 1);
  auto bb = mac.get_bounding_box();
  EXPECT_GT(bb.size(), 0);
  EXPECT_LT(bb[0]*480, 480);
  EXPECT_LT(bb[1]*640, 640);
  EXPECT_LT(bb[2]*480, 480);
  EXPECT_LT(bb[3]*640, 640);
}
