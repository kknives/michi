#pragma once

#include <opencv4/opencv2/opencv.hpp>
#include <memory>
class ClassificationModel {
  public:
  enum class Detection {
    NONE = 0,
    ARROW_LEFT,
    ARROW_RIGHT,
    CONE,
    ARUCO
  };

  private:
  // Design
  struct dClassification{
    virtual ~dClassification() {}
    virtual Detection classify(cv::Mat& image, float threshold) = 0;
    virtual cv::Rect get_bounding_box() = 0;
  };

  template <typename T>
  // Concrete
  struct cClassification : public dClassification {
    Detection classify(cv::Mat& image, float threshold) override {
      return model_classify(m_value, image, threshold);
    }
    cv::Rect get_bounding_box() override {
      return model_get_bounding_box(m_value);
    }

    cClassification(T&& t) : m_value(std::move(t)) {}
    T m_value;
  };
  friend Detection classify(ClassificationModel& model, cv::Mat& image, float threshold=0.6f) {
    return model.m_value->classify(image, threshold);
  }
  friend cv::Rect get_bounding_box(const ClassificationModel& model) {
    return model.m_value->get_bounding_box();
  }
  std::unique_ptr<dClassification> m_value;

  public:
  template <typename T>
  ClassificationModel(T t) : m_value{new cClassification<T>(std::move(t))}{
  }
};
