#pragma once

#include <opencv4/opencv2/opencv.hpp>
#include <memory>
class ClassificationModel {
  private:
  // Design
  struct dClassification{
    virtual ~dClassification() {}
    virtual size_t classify(cv::Mat& image, float threshold) = 0;
    virtual std::array<float, 4> get_bounding_box() = 0;
  };

  template <typename T>
  // Concrete
  struct cClassification : public dClassification {
    size_t classify(cv::Mat& image, float threshold) override {
      return model_classify(m_value, image, threshold);
    }
    std::array<float, 4> get_bounding_box() override {
      return model_get_bounding_box(m_value);
    }

    cClassification(T&& t) : m_value(std::move(t)) {}
    T m_value;
  };
  friend size_t classify(ClassificationModel& model, cv::Mat& image, float threshold=0.6f) {
    return model.m_value->classify(image, threshold);
  }
  friend std::array<float, 4> get_bounding_box(const ClassificationModel& model) {
    return model.m_value->get_bounding_box();
  }
  std::unique_ptr<dClassification> m_value;

  public:
  template <typename T>
  ClassificationModel(T t) : m_value{new cClassification<T>(std::move(t))}{
  }
};
