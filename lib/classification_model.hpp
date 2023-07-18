#pragma once

#include <opencv4/opencv2/opencv.hpp>
#include <memory>
class ClassificationModel {
  private:
  struct dClassification{
    virtual ~dClassification() {}
    virtual size_t classify(cv::Mat& image) = 0;
  };

  template <typename T>
  struct cClassification : public dClassification {
    size_t classify(cv::Mat& image) override {
      return m_value.classify(image);
    }
    cClassification(T&& t) : m_value(std::move(t)) {}
    T m_value;
  };
  friend size_t classify(ClassificationModel& model, cv::Mat& image) {
    return model.m_value->classify(image);
  }
  std::unique_ptr<dClassification> m_value;

  public:
  template <typename T>
  ClassificationModel(T t) : m_value{new cClassification<T>(std::move(t))}{
  }
};
