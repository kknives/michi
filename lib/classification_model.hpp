#pragma once

#include <opencv4/opencv2/opencv.hpp>
#include <memory>
class ClassificationModel {
  public:
  template <typename T>
  ClassificationModel(T&& value);

  void function() const;
  private:
  struct dClassification{
    virtual ~dClassification() {}
    virtual size_t classify(cv::Mat& image) const = 0;
  };

  template <typename T>
  struct cClassification : public dClassification {
    size_t classify(cv::Mat& image) const override {
      return classify(m_value, image);
    }
    T m_value;
  };
  friend size_t classify(const ClassificationModel& model, cv::Mat& image) {
    return model.m_value->classify(image);
  }
  std::unique_ptr<dClassification> m_value;

  public:
  template <typename T>
  ClassificationModel(T t) {
    m_value = std::make_unique<cClassification<T>>(std::move(t));
  }
};
