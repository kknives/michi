#pragma once

#include <string>
#include <vector>
#include <optional>

#include <onnxruntime_c_api.h>
#include <onnxruntime_cxx_api.h>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/dnn.hpp>
#include <spdlog/spdlog.h>

#include "classification_model.hpp"

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

using tDetectionMap = std::array<ClassificationModel::Detection, 3>;
struct Yolov8Params {
  std::array<const char*, 1> input_names;
  std::array<int64_t, 4> input_shape;
  std::array<int64_t, 3> output_shape;
  std::array<const char*, 1> output_names;
  tDetectionMap class_to_detection_map;
};
const Yolov8Params MOHNISH7 {
  .input_names = {"images"},
  .input_shape = {1,3,640,640},
  .output_shape = {1,7,8400},
  .output_names = {"output0"},
  .class_to_detection_map = tDetectionMap{
    ClassificationModel::Detection::ARROW_RIGHT,
    ClassificationModel::Detection::ARROW_LEFT,
    ClassificationModel::Detection::CONE,
  },
};

class Yolov8ArrowClassifier {
  Ort::Env m_env;
  Ort::SessionOptions m_session_options;
  Ort::Session m_session;

  Ort::AllocatorWithDefaultOptions m_allocator;
  std::array<Ort::Value, 1> m_input_tensor;
  std::optional<cv::Rect> m_output_bounding_box;
  const Yolov8Params& m_params;

  void scaleCoords(const cv::Size& imageShape,
                   cv::Rect& coords,
                   const cv::Size& imageOriginalShape)
  {
    float gain =
      std::min((float)imageShape.height / (float)imageOriginalShape.height,
               (float)imageShape.width / (float)imageOriginalShape.width);

    int pad[2] = { (int)(((float)imageShape.width -
                          (float)imageOriginalShape.width * gain) /
                         2.0f),
                   (int)(((float)imageShape.height -
                          (float)imageOriginalShape.height * gain) /
                         2.0f) };

    coords.x = (int)std::round(((float)(coords.x - pad[0]) / gain));
    coords.y = (int)std::round(((float)(coords.y - pad[1]) / gain));

    coords.width = (int)std::round(((float)coords.width / gain));
    coords.height = (int)std::round(((float)coords.height / gain));
  }
  void letterbox(const cv::Mat& image,
                 cv::Mat& outImage,
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

    float ratio[2]{ r, r };
    int newUnpad[2]{ (int)std::round((float)shape.width * r),
                     (int)std::round((float)shape.height * r) };

    auto dw = (float)(newShape.width - newUnpad[0]);
    auto dh = (float)(newShape.height - newUnpad[1]);

    if (auto_) {
      dw = (float)((int)dw % stride);
      dh = (float)((int)dh % stride);
    } else if (scaleFill) {
      dw = 0.0f;
      dh = 0.0f;
      newUnpad[0] = newShape.width;
      newUnpad[1] = newShape.height;
      ratio[0] = (float)newShape.width / (float)shape.width;
      ratio[1] = (float)newShape.height / (float)shape.height;
    }

    dw /= 2.0f;
    dh /= 2.0f;

    if (shape.width != newUnpad[0] && shape.height != newUnpad[1]) {
      cv::resize(image, outImage, cv::Size(newUnpad[0], newUnpad[1]));
    }

    int top = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left = int(std::round(dw - 0.1f));
    int right = int(std::round(dw + 0.1f));
    cv::copyMakeBorder(
      outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
  }
void preprocessing(cv::Mat &image, float*& blob, std::vector<int64_t>& inputTensorShape)
{
    cv::Mat resizedImage, floatImage;
    cv::cvtColor(image, resizedImage, cv::COLOR_BGR2RGB);
    letterbox(resizedImage, resizedImage);

    inputTensorShape[2] = resizedImage.rows;
    inputTensorShape[3] = resizedImage.cols;

    resizedImage.convertTo(floatImage, CV_32FC3, 1 / 255.0);
    cv::Size floatImageSize {floatImage.cols, floatImage.rows};

    // hwc -> chw
    std::vector<cv::Mat> chw(floatImage.channels());
    for (int i = 0; i < floatImage.channels(); ++i)
    {
        chw[i] = cv::Mat(floatImageSize, CV_32FC1, blob + i * floatImageSize.width * floatImageSize.height);
    }
    cv::split(floatImage, chw);
}

  public:
    Yolov8ArrowClassifier(const std::string& model_path,
                          const Yolov8Params& params)
      : m_env(ORT_LOGGING_LEVEL_WARNING, "Yolov8ArrowClassifier")
      , m_session(m_env, model_path.c_str(), m_session_options)
      , m_input_tensor{ Ort::Value::CreateTensor<float>(
          m_allocator,
          params.input_shape.data(),
          params.input_shape.size()) }
      , m_params(params)
    {
      spdlog::info("Initialized and loaded YOLOv8Arrow ONNX session");
    }
    static Yolov8ArrowClassifier make_mohnish7_model(
      const std::string& model_path)
    {
      return Yolov8ArrowClassifier(model_path, MOHNISH7);
    }
    friend ClassificationModel::Detection model_classify(
      Yolov8ArrowClassifier & yac, cv::Mat & image, float threshold = 0.8f)
    {
      yac.m_output_bounding_box.reset();
      cv::Size original_img_shape = image.size();
      float* blob_ptr = yac.m_input_tensor[0].GetTensorMutableData<float>();
      std::vector<int64_t> preprocessed_input_shape{1,3,-1,-1};
      yac.preprocessing(image, blob_ptr, preprocessed_input_shape);

      assert(yac.m_input_tensor[0].IsTensor() && yac.m_input_tensor[0].GetTensorTypeAndShapeInfo().GetShape() == preprocessed_input_shape);
      auto output_tensors = yac.m_session.Run(Ort::RunOptions{ nullptr },
                                        yac.m_params.input_names.data(),
                                        yac.m_input_tensor.data(),
                                        yac.m_params.input_names.size(),
                                        yac.m_params.output_names.data(),
                                        yac.m_params.output_names.size());

    const std::vector<int64_t> output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

      float* raw_output = output_tensors[0].GetTensorMutableData<float>();
      cv::Mat output_mat = cv::Mat(cv::Size((int)yac.m_params.output_shape[2],
                                            (int)yac.m_params.output_shape[1]),
                                   CV_32F,
                                   raw_output)
                             .t();
      std::vector<int> class_ids;
      std::vector<float> confs;
      std::vector<cv::Rect> boxes;
      const int class_names_num = 3;
      float* it = reinterpret_cast<float*>(output_mat.data);

      for (int r = 0; r < output_mat.rows; r++) {
        cv::Mat scores(1, class_names_num, CV_32FC1, it + 4);
        cv::Point class_id;
        double max_conf;
        cv::minMaxLoc(scores, nullptr, &max_conf, nullptr, &class_id);

        if (max_conf < threshold) {
          it += yac.m_params.output_shape[1];
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

        it += yac.m_params.output_shape[1];
      }
      std::vector<int> nms_indices;
      cv::dnn::NMSBoxes(boxes, confs, threshold, 0.45f, nms_indices);
      if (nms_indices.size() == 0) return ClassificationModel::Detection::NONE;

      yac.scaleCoords(
        cv::Size(preprocessed_input_shape[2], preprocessed_input_shape[3]),
        boxes[nms_indices.front()],
        original_img_shape);
      yac.m_output_bounding_box.emplace(boxes[nms_indices.front()]);
      return yac.m_params.class_to_detection_map[class_ids[nms_indices.front()]];
    }

    friend cv::Rect model_get_bounding_box(
      const Yolov8ArrowClassifier& yac)
    {
      assert(yac.m_output_bounding_box.has_value());
      return yac.m_output_bounding_box.value();
      // auto bb_mat = *yac.m_output_bounding_box;
      // std::array<float, 4> bb{bb_mat.tl().x, bb_mat.tl().y, bb_mat.br().x, bb_mat.br().y};
      // return yac.m_output_bounding_box->tl()
    }
  };
