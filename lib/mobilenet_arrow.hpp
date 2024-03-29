#pragma once

#include <algorithm>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <numeric>
#include <onnxruntime_c_api.h>
#include <onnxruntime_cxx_api.h>
#include <opencv4/opencv2/dnn/dnn.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <optional>
#include <span>
#include <spdlog/spdlog.h>
#include "classification_model.hpp"

const std::array<const char*, 1> MOBILENET_ARROW_INPUT_NAMES{ "input_tensor" };
const std::array<int64_t, 4> MOBILENET_ARROW_INPUT_SHAPE{ 1, 480, 640, 3 };
const std::array<const char*, 8> MOBILENET_ARROW_OUTPUT_NAMES{
  "detection_anchor_indices",    "detection_boxes",     "detection_classes",
  "detection_multiclass_scores", "detection_scores",    "num_detections",
  "raw_detection_boxes",         "raw_detection_scores"
};

template<size_t N>
constexpr int
product(const std::span<const std::int64_t, N> v)
{
  int total = 1;
  for (auto& i : v)
    total *= i;
  return total;
}

std::array<ClassificationModel::Detection, 4> WASEEM2_CLASSMAP = {
  ClassificationModel::Detection::NONE,
  ClassificationModel::Detection::CONE,
  ClassificationModel::Detection::ARROW_LEFT,
  ClassificationModel::Detection::ARROW_RIGHT,
};
std::array<ClassificationModel::Detection, 4> MOHNISH4_CLASSMAP= {
  ClassificationModel::Detection::NONE,
  ClassificationModel::Detection::ARROW_LEFT,
  ClassificationModel::Detection::ARROW_RIGHT,
  ClassificationModel::Detection::CONE,
};
class MobilenetArrowClassifier
{
  Ort::Env m_env;
  Ort::SessionOptions m_session_options;
  Ort::Session m_session;

  Ort::AllocatorWithDefaultOptions m_allocator;
  std::array<Ort::Value, 1> m_input_tensor;
  std::optional<cv::Rect> m_bounding_box;
  std::array<ClassificationModel::Detection, 4>& m_result_map;

public:
  MobilenetArrowClassifier(
    std::string const& s,
    std::array<ClassificationModel::Detection, 4>& class_to_detection_map)
    : m_env(ORT_LOGGING_LEVEL_WARNING, "MobilenetArrowClassifier")
    , m_session(m_env, s.c_str(), m_session_options)
    , m_input_tensor{ Ort::Value::CreateTensor<uint8_t>(
        m_allocator,
        MOBILENET_ARROW_INPUT_SHAPE.data(),
        MOBILENET_ARROW_INPUT_SHAPE.size()) }
    , m_result_map(class_to_detection_map)
  {
    spdlog::info("Initialized and loaded MobilenetArrow ONNX session");
  }
  static MobilenetArrowClassifier make_waseem2_model(const std::string& s) {
    return MobilenetArrowClassifier(s, WASEEM2_CLASSMAP);
  }
  static MobilenetArrowClassifier make_mohnish4_model(const std::string& s) {
    return MobilenetArrowClassifier(s, MOHNISH4_CLASSMAP);
  }
  friend ClassificationModel::Detection model_classify(
    MobilenetArrowClassifier& mac,
    cv::Mat& image,
    float threshold = 0.6f)
  {
    mac.m_bounding_box.reset();
    cv::Size original_image_size = image.size();
    // Image preprocessing
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    // Convert to NHWC
    image = image.reshape(
      1); // Use OpenCV's NCHW behaviour for adding the N "channel"
    assert(image.size.p[0] * image.size.p[1] ==
           product(std::span(MOBILENET_ARROW_INPUT_SHAPE)));
    auto dest = mac.m_input_tensor[0].GetTensorMutableData<uint8_t>();
    std::copy(image.begin<uint8_t>(), image.end<uint8_t>(), dest);

    auto output_tensors = mac.m_session.Run(Ort::RunOptions{ nullptr },
                                        MOBILENET_ARROW_INPUT_NAMES.data(),
                                        mac.m_input_tensor.data(),
                                        MOBILENET_ARROW_INPUT_NAMES.size(),
                                        MOBILENET_ARROW_OUTPUT_NAMES.data(),
                                        MOBILENET_ARROW_OUTPUT_NAMES.size());

    const float* ndetections = output_tensors[5].GetTensorData<float>();
    size_t detections = *ndetections;

    const float* scores = output_tensors[4].GetTensorData<float>();
    size_t best_detection =
      std::max_element(scores, scores + detections) - scores;
    const float* classes = output_tensors[2].GetTensorData<float>();
    spdlog::debug("Best detection at: {}, prediction {}, score {}",
                 best_detection,
                 classes[best_detection],
                 scores[best_detection]);
    if (scores[best_detection] < threshold) return ClassificationModel::Detection::NONE;

    const float* boxes = output_tensors[1].GetTensorData<float>();
    std::array<float, 4> bb_tl_br{*(boxes+4*best_detection), *(boxes+4*best_detection + 1), *(boxes+4*best_detection + 2), *(boxes+4*best_detection + 3)};
    spdlog::info("Bounding box {},{},{},{}", bb_tl_br[0],bb_tl_br[1],bb_tl_br[2],bb_tl_br[3]);
    cv::Rect bounding_box(original_image_size.width*bb_tl_br[1], original_image_size.height*bb_tl_br[0], (bb_tl_br[3]-bb_tl_br[1])*original_image_size.width, (bb_tl_br[2] - bb_tl_br[0])*original_image_size.height); // Example bounding box (x, y, width, height)
    mac.m_bounding_box.emplace(bounding_box);

    return mac.m_result_map[classes[best_detection]];
  }
  friend cv::Rect model_get_bounding_box(const MobilenetArrowClassifier& mac)
  {
    assert(mac.m_bounding_box.has_value());
    return mac.m_bounding_box.value();
  }
};
