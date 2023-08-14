#pragma once

#include "state.hpp"
#include <asio/steady_timer.hpp>
#include <chrono>
#include <limits>
#include <cmath>
#include "ardupilot_interface.hpp"

class ArrowPlanner {
  float m_arrow_min_approach;
  float m_cone_min_approach;
  int m_hold_ms;
  float m_careful_vel_ms = 0.0001;
  float m_approach_vel_ms = 0.00001;
  public:
  auto set_goal(MavlinkInterface& mi, State& state) -> asio::awaitable<tResult<void>>{
    auto front = object_in_view(state);
    if (not front.has_value()) {
      // return continue-as-usual goal
      // set velocity here, if slowed down
      co_return;
    }
    auto [object, approach, angle] = *front;
    if (object == ObjectType::ARROW_LEFT or object == ObjectType::ARROW_RIGHT) {
      if (std::isinf(approach)) {
        // slow down
        co_await mi.set_target_velocity({m_careful_vel_ms, 0.0f, 0.0f});
      }
      if (int(approach*1000) > int(m_arrow_min_approach*1000)) {
        co_await mi.set_target_velocity({m_approach_vel_ms, 0.0f, 0.0f});

        asio::steady_timer timer(co_await asio::this_coro::executor);
        timer.expires_at(steady_clock::now()+2s);
        co_await timer.async_wait(use_nothrow_awaitable);

        co_await mi.set_target_velocity({0.0f, 0.0f, 0.0f});
        timer.expires_at(steady_clock::now()+12s);
        co_await timer.async_wait(use_nothrow_awaitable);

        // compute rotation quaternion
        co_await mi.set_target_attitude({0.0f, 0.0f, 0.0f, 0.0f}, 0.5, 0.5);

        // slow down even more, prepare to stop
        // stop
        // turn
        // set new heading as goal
      }
      // check if this arrow has been visited
      // return goal
    }
    if (auto cone = cone_in_view(state); arrows == 5) {
      co_await mi.set_target_velocity({m_approach_vel_ms, 0.0f, 0.0f});
      // Approach cone and stop
      // return goal
    }
  }
  
};

