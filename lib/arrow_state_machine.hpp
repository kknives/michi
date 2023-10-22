#pragma once
#include <cmath>
#include <unordered_map>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <Eigen/Dense>

#include "classification_model.hpp"
#include "mobilenet_arrow.hpp"

const static double EARTH_RADIUS_KM = 6372.8;
struct Position {
  double deg_lat, deg_lon;
  inline double to_radian(double angle) {
    return M_PI * angle / 180.0;
  }
  // From https://rosettacode.org/wiki/Haversine_formula#C++
  double haversine_distance_metres(const Position& p2) {
  	double latRad1 = to_radian(deg_lat);
  	double latRad2 = to_radian(p2.deg_lat);
  	double lonRad1 = to_radian(deg_lon);
  	double lonRad2 = to_radian(p2.deg_lon);

  	double diffLa = latRad2 - latRad1;
  	double doffLo = lonRad2 - lonRad1;

  	double computation = asin(sqrt(sin(diffLa / 2) * sin(diffLa / 2) + cos(latRad1) * cos(latRad2) * sin(doffLo / 2) * sin(doffLo / 2)));
  	return 2 * EARTH_RADIUS_KM * computation * 1000;
  }
  bool operator==(const Position& other) {
    return std::abs(haversine_distance_metres(other)) < 30;
  }
};
using Eigen::Vector3f;
struct Objective {
  enum class Type {
    ARROW_LEFT,
    ARROW_RIGHT,
    CONE,
  };
  Type type;
  float approach_heading; // Compass heading when this objective was approached
  float target_heading; // Target heading after this objective was reached
  Vector3f location;

  float distance_to(const Objective& o) {
    auto diff = location - o.location;
    return diff.norm();
  }
  float distance_to(const Vector3f& other_location) {
    auto diff = location - other_location;
    return diff.norm();
  }
};
auto get_depth_lock(cv::Mat& depth_frame_mat, std::span<float, 4> rect_vertices) -> std::optional<float> {
  cv::Point2f top_left(rect_vertices[0]*640, rect_vertices[1]*480), bottom_right(rect_vertices[2]*640, rect_vertices[3]*480);
  auto cropped = depth_frame_mat(cv::Rect(top_left, bottom_right));

  int valid_depths = 0;
  using tPixel = cv::Point_<uint8_t>;
  valid_depths = cv::countNonZero(cropped);
  std::optional<float> distance;
  if (valid_depths >= (0.5*cropped.total())) {
    std::cout << cv::sum(cropped) << '\n';
    int depth_sum = cv::sum(cropped)[0];
    // Lock available
    distance.emplace(depth_sum/valid_depths);
  }
  return distance;
}

struct ImpureInterface{
  struct InputState {
    uint16_t dist_to_target;
    std::array<float, 3> xyz;
  } input;
  struct Outputs {
    int delay_sec;
    Vector3f target_xyz_pos_local;
    float yaw;
  } output;
};
class ArrowStateMachine {
  std::vector<Objective> m_objectives;
  using tObjectiveId = int;

  // std::unordered_map<Position, tObjectiveId> m_reached;
  ClassificationModel m_detector;
  float m_detector_threshold;

  std::optional<tObjectiveId> m_current_obj;
  std::optional<uint16_t> m_current_dist_to_obj;
  Vector3f m_current_pos;
  float m_current_heading;

  ImpureInterface::InputState m_state;
  void set_outputs(ImpureInterface& i, float yaw = 0, int delay_sec = 0) {
    i.output = {.delay_sec = delay_sec, .target_xyz_pos_local = Vector3f(0,0,0), .yaw = yaw};
    if (m_current_obj) {
      i.output.target_xyz_pos_local = m_objectives[*m_current_obj].location;
    }
  }
  void update_state(const ImpureInterface::InputState& i) {
    m_current_pos = Vector3f(i.xyz[0], i.xyz[1], i.xyz[2]);
    m_current_dist_to_obj.emplace(i.dist_to_target);
  }
  void seek(cv::Mat& rgb_image, cv::Mat& depth_image) {
    // What happens when an objective is detected
    switch (classify(m_detector, rgb_image, m_detector_threshold)) {
      case ClassificationModel::Detection::CONE:
      m_objectives.emplace_back(Objective::Type::CONE, m_current_heading, 0.0f);
      break;
      case ClassificationModel::Detection::ARROW_LEFT:
      m_objectives.emplace_back(Objective::Type::ARROW_LEFT, m_current_heading, 90.0f);
      break;
      case ClassificationModel::Detection::ARROW_RIGHT:
      m_objectives.emplace_back(Objective::Type::ARROW_RIGHT, m_current_heading, -90.0f);
      break;

      case ClassificationModel::Detection::NONE:
      return;
    }
    m_current_obj.emplace(m_objectives.size() - 1);
    // Try to estimate position
    std::array<float, 4> bb = get_bounding_box(m_detector);
    if (auto dist = get_depth_lock(depth_image, bb); dist.has_value()) {
      // Set target location to this distance
      m_objectives.back().location = m_current_pos + Vector3f(*dist, 0.0f, 0.0f);
      // Set yaw target
    } else {
      // Forget you saw anything
      m_objectives.pop_back();
      m_current_obj.reset();
    }
  }
  public:
  bool next(ImpureInterface& i, cv::Mat& rgb_image, cv::Mat& depth_image) {
    update_state(i.input);
    if (m_current_obj) {
      // if objective reached, then set new target heading
      if (m_current_dist_to_obj < 2) {
        if (m_objectives[*m_current_obj].type == Objective::Type::CONE) return true;
        float heading_target = m_objectives[*m_current_obj].target_heading;
        m_current_obj.reset();
        set_outputs(i, heading_target, 10);
        return false;
      }
      return false;
    } else {
      seek(rgb_image, depth_image);
      set_outputs(i);
      return false;
    }
  }
};
