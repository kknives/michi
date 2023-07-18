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

class MobilenetArrowClassifier
{
  Ort::Env m_env;
  Ort::SessionOptions m_session_options;
  Ort::Session m_session;

  Ort::AllocatorWithDefaultOptions m_allocator;
  std::array<Ort::Value, 1> m_input_tensor;
  std::optional<std::tuple<size_t, std::vector<Ort::Value>>> m_outputs;

public:
  MobilenetArrowClassifier(std::string const& s)
    : m_env(ORT_LOGGING_LEVEL_WARNING, "MobilenetArrowClassifier")
    , m_session(m_env, s.c_str(), m_session_options)
    , m_input_tensor{ Ort::Value::CreateTensor<uint8_t>(
        m_allocator,
        MOBILENET_ARROW_INPUT_SHAPE.data(),
        MOBILENET_ARROW_INPUT_SHAPE.size()) }
  {
    spdlog::info("Initialized and loaded MobilenetArrow ONNX session");
  }
  friend size_t model_classify(MobilenetArrowClassifier& mac, cv::Mat& image)
  {
    mac.m_outputs.reset();
    // Image preprocessing
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    // Convert to NHWC
    cv::transpose(image, image);
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

    mac.m_outputs.emplace(std::make_pair(
      best_detection, std::move(output_tensors)));
    return classes[best_detection];
  }
  friend std::array<float, 4> model_get_bounding_box(const MobilenetArrowClassifier& mac)
  {
    assert(mac.m_outputs.has_value());
    auto& [detection, output_tensor] = *mac.m_outputs;
    const float* boxes = output_tensor[1].GetTensorData<float>();
    // top, left, bottom, right
    std::array<float, 4> m{*(boxes+4*detection), *(boxes+4*detection + 1), *(boxes+4*detection + 2), *(boxes+4*detection + 3)};
    spdlog::info("Bounding box {},{},{},{}", m[0],m[1],m[2],m[3]);
    return m;
  }
};
