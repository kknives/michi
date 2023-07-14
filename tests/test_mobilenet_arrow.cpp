// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// Modified by Team RUDRA

#include "opencv2/dnn/dnn.hpp"
#include <algorithm>  // std::generate
#include <iterator>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <onnxruntime_cxx_api.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/dnn.hpp>

// pretty prints a shape dimension vector
std::string print_shape(const std::vector<std::int64_t>& v) {
  std::stringstream ss("");
  for (std::size_t i = 0; i < v.size() - 1; i++) ss << v[i] << "x";
  ss << v[v.size() - 1];
  return ss.str();
}

int calculate_product(const std::vector<std::int64_t>& v) {
  int total = 1;
  for (auto& i : v) total *= i;
  return total;
}

template <typename T>
Ort::Value vec_to_tensor(std::vector<T>& data, const std::vector<std::int64_t>& shape) {
  Ort::MemoryInfo mem_info =
      Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
  auto tensor = Ort::Value::CreateTensor<T>(mem_info, data.data(), data.size(), shape.data(), shape.size());
  return tensor;
}

#ifdef _WIN32
int wmain(int argc, ORTCHAR_T* argv[]) {
#else
int main(int argc, ORTCHAR_T* argv[]) {
#endif
  if (argc != 3) {
    std::cout << "Usage: ./onnx-api-example <onnx_model.onnx> <image.jpg>" << std::endl;
    return -1;
  }

  std::basic_string<ORTCHAR_T> model_file = argv[1];

  // onnxruntime setup
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "example-model-explorer");
  Ort::SessionOptions session_options;
  Ort::Session session = Ort::Session(env, model_file.c_str(), session_options);

  // print name/shape of inputs
  Ort::AllocatorWithDefaultOptions allocator;
  std::vector<std::string> input_names;
  std::vector<std::int64_t> input_shapes;
  std::cout << "Input Node Name/Shape (" << input_names.size() << "):" << std::endl;
  for (std::size_t i = 0; i < session.GetInputCount(); i++) {
    input_names.emplace_back(session.GetInputNameAllocated(i, allocator).get());
    input_shapes = session.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    std::cout << "\t" << input_names.at(i) << " : " << print_shape(input_shapes) << std::endl;
  }
  // some models might have negative shape values to indicate dynamic shape, e.g., for variable batch size.
  for (auto& s : input_shapes) {
    if (s < 0) {
      s = 1;
    }
  }

  // print name/shape of outputs
  std::vector<std::string> output_names;
  std::cout << "Output Node Name/Shape (" << output_names.size() << "):" << std::endl;
  for (std::size_t i = 0; i < session.GetOutputCount(); i++) {
    output_names.emplace_back(session.GetOutputNameAllocated(i, allocator).get());
    auto output_shapes = session.GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    std::cout << "\t" << output_names.at(i) << " : " << print_shape(output_shapes) << std::endl;
  }

  // Create a single Ort tensor of random numbers
  auto input_shape = input_shapes;
  auto total_number_elements = calculate_product(input_shape);

  // generate random numbers in the range [0, 255]
  std::vector<float> input_tensor_values{};
  std::vector<Ort::Value> input_tensors;

  cv::Mat image_bgr = cv::imread(argv[2], cv::IMREAD_COLOR);
  // cv::resize(image_bgr, image_bgr, cv::Size(input_shape[1], input_shape[2]));
  image_bgr.convertTo(image_bgr, CV_32F, 2.0f / 255.0f, -1.0f);
  cv::dnn::blobFromImage(image_bgr, 1.0, cv::Size(input_shape[1], input_shape[2]));
  // cv::dnn::blobFromImage(image_bgr, image_bgr);
  input_tensor_values.assign(image_bgr.begin<float>(), image_bgr.end<float>());
  
  input_tensors.emplace_back(vec_to_tensor<float>(input_tensor_values, input_shape));
  // double-check the dimensions of the input tensor
  assert(input_tensors[0].IsTensor() && input_tensors[0].GetTensorTypeAndShapeInfo().GetShape() == input_shape);
  std::cout << "\ninput_tensor shape: " << print_shape(input_tensors[0].GetTensorTypeAndShapeInfo().GetShape()) << std::endl;

  // pass data through model
  std::vector<const char*> input_names_char(input_names.size(), nullptr);
  std::transform(std::begin(input_names), std::end(input_names), std::begin(input_names_char),
                 [&](const std::string& str) { return str.c_str(); });

  std::vector<const char*> output_names_char(output_names.size(), nullptr);
  std::transform(std::begin(output_names), std::end(output_names), std::begin(output_names_char),
                 [&](const std::string& str) { return str.c_str(); });

  std::cout << "Running model..." << std::endl;
  try {
    auto output_tensors = session.Run(Ort::RunOptions{nullptr}, input_names_char.data(), input_tensors.data(),
                                      input_names_char.size(), output_names_char.data(), output_names_char.size());
    std::cout << "Done!" << std::endl;

    // double-check the dimensions of the output tensors
    // NOTE: the number of output tensors is equal to the number of output nodes specifed in the Run() call
    assert(output_tensors.size() == output_names.size() && output_tensors[0].IsTensor());
    // const float* detections = output_tensors[2].GetTensorData<float>();
    // int idetections = int(*detections);
    // std::cout << "No of detections: " << idetections << '\n';

    // auto score_shape = output_tensors[3].GetTensorTypeAndShapeInfo().GetShape();
    // std::cout << "Score tensor shape: ";
    // std::copy(score_shape.begin(), score_shape.end(), std::ostream_iterator<int64_t>(std::cout, "x"));
    // std::cout << '\n';
    // std::cout << "Scores for each detection: " << "\n   ";
    // const float* scores = output_tensors[3].GetTensorData<float>();
    // std::copy(scores, scores + idetections, std::ostream_iterator<float>(std::cout, ", "));
    // std::cout << '\n';

    // auto bbox_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
    // std::cout << "Bounding Box tensor shape: ";
    // std::copy(bbox_shape.begin(), bbox_shape.end(), std::ostream_iterator<int64_t>(std::cout, "x"));
    // std::cout << '\n';
    // std::cout << "BBox for each detection: " << "\n   ";
    // const float* bbox = output_tensors[0].GetTensorData<float>();
    // std::copy(bbox, bbox + idetections, std::ostream_iterator<float>(std::cout, ", "));
    // std::cout << '\n';
    
    // auto class_shape = output_tensors[1].GetTensorTypeAndShapeInfo().GetShape();
    // auto class_count = output_tensors[1].GetTensorTypeAndShapeInfo().GetElementCount();
    // std::cout << "Class tensor shape: ";
    // std::copy(class_shape.begin(), class_shape.end(), std::ostream_iterator<int64_t>(std::cout, "x"));
    // std::cout << '\n';
    // std::cout << "No of elements = " << class_count << '\n';
    // const float* classv = output_tensors[1].GetTensorData<float>();
    // for (int i = 0; i < class_count; i += 4) {
    //   std::cout << "[ ";
    //   std::cout << 320.0f*classv[i] << ", " << 320.0f*classv[i+1] << ", " <<
    //     320.0f*classv[i+2] << ", " << 320.0f*classv[i+3] << " ]\n";
    //   // std::copy(classv+i, classv+i+4, std::ostream_iterator<float>(std::cout, ", "));
    //   // std::cout << " ]" << '\n';
    // }
  } catch (const Ort::Exception& exception) {
      std::cout << "ERROR running model inference: " << exception.what() << std::endl;
    exit(-1);
  }
}