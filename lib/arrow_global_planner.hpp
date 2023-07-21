#pragma once

#include "state.hpp"
#include <limits>
#include <cmath>

class ArrowPlanner {
  float arrow_min_approach;
  float cone_min_approach;
  int hold_ms;
  public:
  auto get_goal(State& state) -> void {
    auto front = object_in_view(state);
    if (not front.has_value()) {
      // return continue-as-usual goal
    }
    auto [object, approach, angle] = *front;
    if (object == ObjectType::ARROW_LEFT or object == ObjectType::ARROW_RIGHT) {
      if (std::isinf(approach)) {
        // slow down
        set_position_target_local({.vel = 0.001});
      }
      if (int(approach*1000) > int(arrow_min_approach*1000)) {
        set_position_target_local({.vel = 0.00001});
        // slow down even more, prepare to stop
        // stop
        // turn
        // set new heading as goal
      }
      // check if this arrow has been visited
      // return goal
    }
    if (auto cone = cone_in_view(state); arrows == 5) {
      set_position_target_local({.x = 0.001});
      // Approach cone and stop
      // return goal
    }
  }
  
};

