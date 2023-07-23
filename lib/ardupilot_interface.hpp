#pragma once

#include "mavlink_types.h"
#include <algorithm>
#include <asio.hpp>
#include <asio/serial_port.hpp>
#include <asio/experimental/as_tuple.hpp>
#include <asio/experimental/awaitable_operators.hpp>
#include <asio/write.hpp>
#include <chrono>
#include <cmath>
#include <mavlink/common/mavlink.h>
#include <memory>
#include <span>

using namespace std::chrono;
using namespace asio::experimental::awaitable_operators;

constexpr auto use_nothrow_awaitable = asio::experimental::as_tuple(asio::use_awaitable);
const float INVALID = 0.0f;
class MavlinkInterface
{
  asio::serial_port m_uart;
  time_point<steady_clock> m_start;

  uint8_t m_targets_channel = 1;
  uint8_t m_heartbeat_channel = 0;
  uint8_t m_positions_channel = 2;

  // Guidance computer shares the system id with the autopilot => same system
  uint8_t m_system_id = 0;
  uint8_t m_component_id = 1;
  uint8_t m_my_id = 5;

  // Use field masks
  uint32_t USE_POSITION = 0x0DFC;
  uint32_t USE_VELOCITY = 0x0DE7;
  uint32_t USE_YAW = 0x09FF;

  inline auto get_uptime() -> uint32_t {
    return duration_cast<milliseconds>(steady_clock::now() - m_start).count();
  }
public:
  MavlinkInterface(asio::serial_port sp)
    : m_uart{ std::move(sp) }, m_start{steady_clock::now()}
  {
  }
  auto set_target_position_local(std::span<float, 3> xyz) -> asio::awaitable<void>
  {
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack_chan(
      m_system_id,
      m_my_id,
      m_targets_channel,
      &msg,
      get_uptime(),
      m_system_id,
      m_component_id,
      MAV_FRAME_BODY_OFFSET_NED,
      USE_POSITION,
      xyz[0],
      xyz[1],
      xyz[2],
      INVALID,
      INVALID,
      INVALID,
      INVALID,
      INVALID,
      INVALID,
      INVALID,
      INVALID);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    auto len = mavlink_msg_to_send_buffer(buffer, &msg);
    auto [error, written] = co_await asio::async_write(m_uart, asio::buffer(buffer, len), use_nothrow_awaitable);
    // consider timeout and error handling
  }
  void set_target_pos_vel_accel_local()
  {
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(0,
                                                   0,
                                                   &msg,
                                                   0,
                                                   0,
                                                   0,
                                                   MAV_FRAME_LOCAL_OFFSET_NED,
                                                   0x0DF8,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   0);
  }
  void set_target_heading_local() {}
  void get_position() {}
};