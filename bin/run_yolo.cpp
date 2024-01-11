// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
// Modified by Team RUDRA

#include <algorithm>  // std::generate
#include <cmath>
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

void scaleCoords(const cv::Size& imageShape, cv::Rect& coords, const cv::Size& imageOriginalShape)
{
    float gain = std::min((float)imageShape.height / (float)imageOriginalShape.height,
                          (float)imageShape.width / (float)imageOriginalShape.width);

    int pad[2] = {(int) (( (float)imageShape.width - (float)imageOriginalShape.width * gain) / 2.0f),
                  (int) (( (float)imageShape.height - (float)imageOriginalShape.height * gain) / 2.0f)};

    coords.x = (int) std::round(((float)(coords.x - pad[0]) / gain));
    coords.y = (int) std::round(((float)(coords.y - pad[1]) / gain));

    coords.width = (int) std::round(((float)coords.width / gain));
    coords.height = (int) std::round(((float)coords.height / gain));

}
void letterbox(const cv::Mat& image, cv::Mat& outImage,
                      const cv::Size& newShape = cv::Size(640, 640),
                      const cv::Scalar& color = cv::Scalar(114, 114, 114),
                      bool auto_ = false,
                      bool scaleFill = false,
                      bool scaleUp = true,
                      int stride = 32)
{
    cv::Size shape = image.size();
    float r = std::min((float)newShape.height / (float)shape.height,
                       (float)newShape.width / (float)shape.width);
    if (!scaleUp)
        r = std::min(r, 1.0f);

    float ratio[2] {r, r};
    int newUnpad[2] {(int)std::round((float)shape.width * r),
                     (int)std::round((float)shape.height * r)};

    auto dw = (float)(newShape.width - newUnpad[0]);
    auto dh = (float)(newShape.height - newUnpad[1]);

    if (auto_)
    {
        dw = (float)((int)dw % stride);
        dh = (float)((int)dh % stride);
    }
    else if (scaleFill)
    {
        dw = 0.0f;
        dh = 0.0f;
        newUnpad[0] = newShape.width;
        newUnpad[1] = newShape.height;
        ratio[0] = (float)newShape.width / (float)shape.width;
        ratio[1] = (float)newShape.height / (float)shape.height;
    }

    dw /= 2.0f;
    dh /= 2.0f;

    if (shape.width != newUnpad[0] && shape.height != newUnpad[1])
    {
        cv::resize(image, outImage, cv::Size(newUnpad[0], newUnpad[1]));
    }

    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));
    cv::copyMakeBorder(outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
}
void preprocessing(cv::Mat &image, float*& blob, std::vector<int64_t>& inputTensorShape)
{
    cv::Mat resizedImage, floatImage;
    cv::cvtColor(image, resizedImage, cv::COLOR_BGR2RGB);
    letterbox(resizedImage, resizedImage);

    inputTensorShape[2] = resizedImage.rows;
    inputTensorShape[3] = resizedImage.cols;

    resizedImage.convertTo(floatImage, CV_32FC3, 1 / 255.0);
    blob = new float[floatImage.cols * floatImage.rows * floatImage.channels()];
    cv::Size floatImageSize {floatImage.cols, floatImage.rows};

    // hwc -> chw
    std::vector<cv::Mat> chw(floatImage.channels());
    for (int i = 0; i < floatImage.channels(); ++i)
    {
        chw[i] = cv::Mat(floatImageSize, CV_32FC1, blob + i * floatImageSize.width * floatImageSize.height);
    }
    cv::split(floatImage, chw);
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

  std::vector<float> input_tensor_values{};
  std::vector<Ort::Value> input_tensors;

  cv::Mat image = cv::imread(argv[2]);
  cv::Size original_img_shape = image.size();
  float *blob = nullptr;
  preprocessing(image, blob, input_shape);
  // cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
  // letterbox(image, image);
  // image.convertTo(image, CV_32FC3);

  // image = image.reshape(1);
  // std::cout << "Image after cv: " << image.size << '\n';
  // assert(image.size.p[0] * image.size.p[1] == calculate_product(input_shape));

  input_tensor_values.assign(blob, blob+calculate_product(input_shape));
  std::cout << "Size of input_tensor_values " << input_tensor_values.size() << '\n';
  
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

    float* raw_outputs = output_tensors[0].GetTensorMutableData<float>();
    const std::vector<int64_t> output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
    const size_t output_count = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
    cv::Mat output0 = cv::Mat(cv::Size((int) output_shape[2], (int) output_shape[1]), CV_32F, raw_outputs).t();

    int elem_in_batch = int(output_shape[1]*output_shape[2]);

    const float conf_threshold = 0.8f;
    const float left_threshold = 0.8;
    const float right_threshold = 0.8;

    std::copy(output_shape.begin(), output_shape.end(), std::ostream_iterator<int64_t>(std::cout, " "));
    std::cout << '\n';

    std::vector<int> class_ids;
    std::vector<float> confs;
    std::vector<cv::Rect> boxes;
    const int class_names_num = 3;
    float* it = reinterpret_cast<float*>(output0.data);

    for (int r = 0; r < output0.rows; r++) {
      // float sc1 = it[0], sc2 = it[1], sc3 = it[2];
      cv::Mat scores(1, class_names_num, CV_32FC1, it + 4);
      cv::Point class_id;
      double max_conf;
      cv::minMaxLoc(scores, nullptr, &max_conf, nullptr, &class_id);

      if (max_conf < conf_threshold) {
        it += output_shape[1];
        continue;
      }
      int centerX = int(it[0]);
      int centerY = int(it[1]);
      int width = int(it[2]);
      int height = int(it[3]);

      int left = std::max(centerX - width/2, 0);
      int top = std::max(centerY - height/2, 0);

      boxes.emplace_back(left, top, width, height);
      class_ids.emplace_back(class_id.x);
      confs.emplace_back(max_conf);

      it += output_shape[1];
    }

    std::vector<int> nms_indices;
    cv::dnn::NMSBoxes(boxes, confs, 0.8f, 0.45f, nms_indices);

    std::cout << "NMS indices count: " << nms_indices.size() << '\n';
    for (int i : nms_indices) {
      scaleCoords(cv::Size(640, 640), boxes[i], original_img_shape);
      std::cout << "Detection confidence " << confs[i] << " Class " << class_ids[i] << "\nBounding box "
                << boxes[i].x << ", " << boxes[i].y << " by " << boxes[i].height
                << " × " << boxes[i].width << '\n';
      std::cout << '\n';
      cv::rectangle(image, boxes[i], cv::Scalar(0, 255, 0), 2);
      cv::imwrite("detection.jpg", image);
    }

  } catch (const Ort::Exception& exception) {
      std::cout << "ERROR running model inference: " << exception.what() << std::endl;
    exit(-1);
  }
}
