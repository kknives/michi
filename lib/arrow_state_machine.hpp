#pragma once
#include <cmath>
#include <unordered_map>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include "classification_model.hpp"
#include "mobilenet_arrow.hpp"

using LatLonDeg = Eigen::Vector2f;
using Eigen::Vector3f;
struct Objective {
  enum class Type {
    ARROW_LEFT,
    ARROW_RIGHT,
    CONE,
    DIRECTION,
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
struct ImpureInterface{
  struct InputState {
    std::span<float, 3> xyz;
    float heading_deg;
  } input;
  struct Outputs {
    int delay_sec;
    Vector3f target_xyz_pos_local;
    float yaw;
  } output;
  ImpureInterface(std::span<float, 3> input_xyz, float yaw_deg) : input{.xyz = input_xyz, .heading_deg = yaw_deg} {}
};
class ArrowStateMachine {
  std::vector<Objective> m_objectives;
  using tObjectiveId = int;

  // std::unordered_map<Position, tObjectiveId> m_reached;
  ClassificationModel m_detector;
  float m_detector_threshold;

  std::optional<tObjectiveId> m_current_obj;
  std::optional<float> m_current_dist_to_obj;
  Vector3f m_current_pos;
  float m_current_heading_deg;

  auto get_pose_lock(cv::Mat& rgb_image,
                     std::span<float, 4> rect_vertices,
                     const cv::Mat& camera_matrix,
                     std::span<float, 4> distance_coeff)
    -> std::optional<double>
  {
    // bool res = cv::solvePnP(rgb_image, )
    // TODO: complete this
    return std::optional<double>();
  }
  auto get_depth_lock(rs2::depth_frame& depth_frame,
                      cv::Rect rect_vertices) -> std::optional<float>
  {
    std::optional<float> distance;
    int count = 0, valid = 0;
    float dist_sum = 0.0f;
    spdlog::debug("Rectangle: {} {}, {} {}, size: {}×{}",
                  rect_vertices.tl().x,
                  rect_vertices.tl().y,
                  rect_vertices.br().x,
                  rect_vertices.br().y,
                  rect_vertices.height, rect_vertices.width);
    for (int i = std::max(rect_vertices.x, 0); i < std::min(rect_vertices.x + rect_vertices.width, depth_frame.get_width()); i++) {
      for (int j = std::max(rect_vertices.y, 0); j < std::min(rect_vertices.y + rect_vertices.height, depth_frame.get_height()); j++) {
        float dist = depth_frame.get_distance(i, j);
        if (int(dist*1000) != 0) valid++;
        count++;
        dist_sum += dist;
      }
    }
    if (count*0.5 < valid) distance.emplace(dist_sum/count);
    spdlog::debug("Depth lock done");
    return distance;
  }

  void set_outputs(ImpureInterface& i, float yaw = 0, int delay_sec = 0, bool send_obj = false) {
    i.output = { .delay_sec = delay_sec,
                 .target_xyz_pos_local = Vector3f(0, 0, 0),
                 .yaw = yaw };
    if (send_obj and m_current_obj) {
      i.output.target_xyz_pos_local = m_objectives[*m_current_obj].location;
    }
  }
  void update_state(const ImpureInterface::InputState& i) {
    m_current_pos = Vector3f(i.xyz[0], i.xyz[1], i.xyz[2]);
    m_current_heading_deg = i.heading_deg;
    spdlog::debug("Got heading {}",m_current_heading_deg);
    if (m_current_obj) {
      m_current_dist_to_obj.emplace(m_objectives[*m_current_obj].distance_to(m_current_pos));
    }
  }
  bool seek(cv::Mat& rgb_image, rs2::depth_frame& depth_image, bool strong_only = false) {
    // What happens when an objective is detected
    switch (classify(m_detector, rgb_image, m_detector_threshold)) {
      case ClassificationModel::Detection::CONE:
      m_objectives.emplace_back(Objective::Type::CONE, m_current_heading_deg, m_current_heading_deg);
      spdlog::critical("Sighted CONE");
      break;
      case ClassificationModel::Detection::ARROW_LEFT:
      m_objectives.emplace_back(Objective::Type::ARROW_LEFT, m_current_heading_deg, m_current_heading_deg-90.0f);
      spdlog::critical("Saw LEFT");
      break;
      case ClassificationModel::Detection::ARROW_RIGHT:
      m_objectives.emplace_back(Objective::Type::ARROW_RIGHT, m_current_heading_deg, m_current_heading_deg+90.0f);
      spdlog::critical("Saw RIGHT");
      break;

      case ClassificationModel::Detection::NONE:
      if (strong_only) return true;
      float target_heading_deg = m_current_heading_deg;;
      if (m_objectives.size() > 0) target_heading_deg = m_objectives.back().target_heading;

      m_objectives.emplace_back(Objective::Type::DIRECTION, m_current_heading_deg, target_heading_deg);
      float heading_radian = (m_current_heading_deg*M_PI) / 180.0f;
      float distance_to_wp_metres = 5.0f;
      m_objectives.back().location = m_current_pos + distance_to_wp_metres*Vector3f(std::cos(heading_radian), std::sin(heading_radian), 0.0f); 
      m_current_obj.emplace(m_objectives.size()-1);
      spdlog::critical("Setting waypoint");
      return true;
    }
    m_current_obj.emplace(m_objectives.size() - 1);
    // Try to estimate position
    cv::Rect bb = get_bounding_box(m_detector);
    if (auto dist = get_depth_lock(depth_image, bb); dist.has_value()) {
      // Set target location to this distance
      m_objectives.back().location = m_current_pos + Vector3f(*dist, 0.0f, 0.0f);
      spdlog::critical("Target at {}m away", *dist);
      // Set yaw target
      return true;
    } else {
      // Forget you saw anything
      spdlog::warn("Could not get depth lock, forgetting");
      m_objectives.pop_back();
      m_current_obj.reset();
      return false;
    }
  }
  public:
  bool next(ImpureInterface& i, cv::Mat& rgb_image, rs2::depth_frame& depth_image) {
    update_state(i.input);
    if (m_current_obj) {

      if (m_objectives[*m_current_obj].type == Objective::Type::DIRECTION) {
        seek(rgb_image, depth_image, true);
        if (m_objectives[*m_current_obj].type != Objective::Type::DIRECTION) {
          set_outputs(i, 0, 0, true);
          return false;
        }
      }
      // if objective reached, then set new target heading
      if (m_current_dist_to_obj < 2) {
        spdlog::critical("Current target reached");
        if (m_objectives[*m_current_obj].type == Objective::Type::CONE) return true;
        float heading_target = m_objectives[*m_current_obj].target_heading;
        int delay = 10;
        m_current_obj.reset();
        if (m_objectives[*m_current_obj].type == Objective::Type::DIRECTION) {
          delay = 0;
          // heading_target = 0;
        }
        set_outputs(i, heading_target, delay);
        return false;
      }
      spdlog::info("Distance to target: {}", *m_current_dist_to_obj);
      return false;
    } else {
      set_outputs(i, 0, 0,
      seek(rgb_image, depth_image));
      return false;
    }
  }
  ArrowStateMachine(ClassificationModel& m, float detection_threshold = 0.6f)
    : m_detector(std::move(m))
    , m_detector_threshold(detection_threshold)
  {
  }
};
