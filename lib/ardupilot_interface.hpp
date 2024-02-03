#pragma once

#include <Eigen/Geometry>
#include "expected.hpp"
#include <algorithm>
#include <concepts>
#include <queue>
// #define ASIO_ENABLE_HANDLER_TRACKING 1
#include "common.hpp"
#include <chrono>
#include <asio/serial_port.hpp>
#include <asio/this_coro.hpp>
#include <asio/experimental/channel.hpp>
#include <asio/write.hpp>
#include <chrono>
#include <cmath>
#include <mavlink/common/mavlink.h>
#include <mavlink/mavlink_helpers.h>
#include <memory>
#include <span>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ranges.h>

using namespace std::literals::chrono_literals;
using asio::ip::tcp;

enum class MavlinkErrc
{
  Success = 0,
  NoHeartbeat = 1, // System Failure
  NoCommandAck,
  FailedWrite,
  FailedRead,
  TransmitTimeout = 10, // Timeouts
  ReceiveTimeout,
};
struct MavlinkErrCategory : std::error_category
{
  const char* name() const noexcept override
  {
    return "AutopilotCommunication";
  }
  std::string message(int ev) const override
  {
    switch (static_cast<MavlinkErrc>(ev)) {
      case MavlinkErrc::NoHeartbeat:
        return "no heartbeat received from autopilot";
      case MavlinkErrc::NoCommandAck:
        return "no ack received after command";
      case MavlinkErrc::FailedWrite:
        return "could not write, asio error";
      case MavlinkErrc::FailedRead:
        return "could not read, asio error";
      case MavlinkErrc::ReceiveTimeout:
        return "did not get response, timed out";
      case MavlinkErrc::TransmitTimeout:
        return "could not send message, timed out";
      default:
        return "(unrecognized error)";
    }
  }
};
const MavlinkErrCategory mavlinkerrc_category;
std::error_code
make_error_code(MavlinkErrc e)
{
  return { static_cast<int>(e), mavlinkerrc_category };
}
namespace std {
template<>
struct is_error_code_enum<MavlinkErrc> : true_type
{};
}

template<typename T>
using tResult = tl::expected<T, std::error_code>;
using tl::make_unexpected;
using namespace std::chrono;
const float INVALID = 0.0f;

struct NavStatus {
  int16_t m_desired_bearing_deg;
  int16_t m_target_bearing_deg;
  int16_t m_waypoint_dist_m;
};
struct ArdupilotState {
  std::array<float, 3> m_local_xyz;
  std::array<int32_t, 3> m_lat_lon_alt;
  std::array<float, 3> m_global_vel;
  std::array<float, 3> m_rpy;
  std::array<float, 3> m_rpy_vel;
  float m_heading_deg;
  NavStatus m_nav;
};

template <typename I>
  requires std::convertible_to<I, tcp::socket> || std::convertible_to<I, asio::serial_port>
class MavlinkInterface
{
  I m_uart;
  time_point<steady_clock> m_start;

  // Guidance computer shares the system id with the autopilot => same system
  uint8_t m_system_id = 1;
  uint8_t m_component_id = 1;
  uint8_t m_my_id = 5;

  // Use field masks
  uint32_t USE_POSITION = 0x0DFC;
  uint32_t USE_VELOCITY = 0x0DE7;
  uint32_t USE_YAW = 0x09FF;

  uint8_t m_channel = MAVLINK_COMM_0;

  // The thing we want to get from AP
  ArdupilotState m_ap_state;
  size_t REQUESTS_QUEUE_SIZE = 25;
  asio::experimental::channel<void(asio::error_code, mavlink_message_t)> m_ap_requests;

  inline auto get_uptime() -> uint32_t
  {
    return duration_cast<milliseconds>(steady_clock::now() - m_start).count();
  }
  auto send_message(const mavlink_message_t& msg)
    -> asio::awaitable<std::tuple<asio::error_code, std::size_t>>
  {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    auto len = mavlink_msg_to_send_buffer(buffer, &msg);
    auto res = co_await asio::async_write(
      m_uart, asio::buffer(buffer, len), use_nothrow_awaitable);
    co_return res;
  }
  auto update_local_position(const mavlink_message_t* msg) -> void {
    mavlink_local_position_ned_t pos;
    mavlink_msg_local_position_ned_decode(msg, &pos);
    m_ap_state.m_local_xyz = {pos.x, pos.y, pos.z};
  }
  auto update_global_position(const mavlink_message_t* msg) -> void {
    mavlink_global_position_int_cov_t pos;
    mavlink_msg_global_position_int_cov_decode(msg, &pos);
    m_ap_state.m_lat_lon_alt = {pos.lat, pos.lon, pos.alt};
    m_ap_state.m_global_vel = {pos.vx, pos.vy, pos.vz};
  }
  auto update_heading(const mavlink_message_t* msg) -> void {
    mavlink_global_position_int_t pos;
    mavlink_msg_global_position_int_decode(msg, &pos);
    m_ap_state.m_heading_deg = pos.hdg / 100.0f;
  }
  auto update_attitude(const mavlink_message_t* msg) -> void {
    mavlink_attitude_t att;
    mavlink_msg_attitude_decode(msg, &att);
    m_ap_state.m_rpy = {att.roll, att.pitch, att.yaw};
    m_ap_state.m_rpy_vel = {att.rollspeed, att.pitchspeed, att.yawspeed};
  }
  auto update_navigation_status(const mavlink_message_t* msg) -> void {
    mavlink_nav_controller_output_t nav;
    mavlink_msg_nav_controller_output_decode(msg, &nav);
    m_ap_state.m_nav.m_desired_bearing_deg = nav.nav_bearing;
    m_ap_state.m_nav.m_target_bearing_deg = nav.target_bearing;
    m_ap_state.m_nav.m_waypoint_dist_m = nav.wp_dist;
  }
  auto show_statustext(const mavlink_message_t* msg) -> void {
    mavlink_statustext_t stxt;
    mavlink_msg_statustext_decode(msg, &stxt);
    if (stxt.severity < MAV_SEVERITY_NOTICE) {
      spdlog::warn("AP Status: {}", stxt.text);
    } else
      spdlog::info("AP Status: {}", stxt.text);
  }
  auto handle_message(const mavlink_message_t* msg)
  {
    spdlog::trace("Got message with ID {}, system {}", msg->msgid,
    msg->sysid);
    if (msg->sysid != 1)
      return; // Only handling messages from autopilot
    switch (msg->msgid) {
      case MAVLINK_MSG_ID_STATUSTEXT:
        show_statustext(msg);
        break;
      case MAVLINK_MSG_ID_HEARTBEAT:
        spdlog::trace("Got heartbeat");
        break;
      case MAVLINK_MSG_ID_SYS_STATUS:
        // TODO: https://mavlink.io/en/messages/common.html#SYS_STATUS
        spdlog::trace("Got system status");
        break;
      case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        spdlog::trace("Got local_position_ned");
        update_local_position(msg);
        break;
      case MAVLINK_MSG_ID_ATTITUDE:
        spdlog::trace("Got attitude");
        update_attitude(msg);
        break;
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        spdlog::trace("Got Global Position");
        update_heading(msg);
        break;
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV:
        spdlog::trace("Got Global Position cov");
        update_global_position(msg);
        break;
      case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
        spdlog::info("Got Nav controller output");
        update_navigation_status(msg);
        break;
      case MAVLINK_MSG_ID_COMMAND_ACK:
        spdlog::info("Got ack");
        break;
      case MAVLINK_MSG_ID_RAW_IMU:
      case MAVLINK_MSG_ID_RC_CHANNELS_SCALED:
      case MAVLINK_MSG_ID_SCALED_IMU3:
      case MAVLINK_MSG_ID_SCALED_IMU2:
        spdlog::trace("Ignored message id {}", msg->msgid);
        break;
      default:
        spdlog::trace("Unhandled message id {}", msg->msgid);
    }
    spdlog::trace("State updated: {} {} {} {}", m_ap_state.m_lat_lon_alt, 
    m_ap_state.m_global_vel, m_ap_state.m_rpy, m_ap_state.m_rpy_vel);
  }
  auto receive_message() -> asio::awaitable<std::error_code> {
    std::vector<uint8_t> buffer(MAVLINK_MAX_PACKET_LEN);
    auto [error, len] = co_await m_uart.async_read_some(
      asio::buffer(buffer), use_nothrow_awaitable);
    if (error) {
      spdlog::trace("Read from m_uart failed, asio error: {}", error.message());   
      co_return error;
    }
    mavlink_message_t msg;
    mavlink_status_t status;
    for (int i = 0; i < len; i++) {
      if (mavlink_parse_char(m_channel, buffer[i], &msg, &status))
        break;
    }
    handle_message(&msg);
    co_return MavlinkErrc::Success;
  }

public:
  MavlinkInterface(I&& sp)
    : m_uart{ std::move(sp) }
    , m_start{ steady_clock::now() }
    , m_ap_requests(m_uart.get_executor(), REQUESTS_QUEUE_SIZE)
  {
    m_ap_state.m_nav = { .m_desired_bearing_deg = 0,
                         .m_target_bearing_deg = 0,
                         .m_waypoint_dist_m = 0 };
    // m_uart.set_option(asio::serial_port_base::baud_rate(115200));
  }
  auto loop() -> asio::awaitable<tResult<void>>
  {
    asio::steady_timer timer(m_uart.get_executor());

    mavlink_message_t hb_msg = heartbeat();
    auto [error, written] = co_await send_message(hb_msg);
    if (error) {
      spdlog::trace("Couldn't send first heartbeat, asio error: {}", error.message());
      co_return make_unexpected(MavlinkErrc::FailedWrite);
    }
    auto last_heartbeat = steady_clock::now();
    while (true) {
      if (steady_clock::now() > last_heartbeat + 1s) {
        hb_msg = heartbeat();
        tie(error, written) = co_await send_message(hb_msg);
        if (error) {
          spdlog::trace("Couldn't send heartbeat, asio error: {}", error.message());   
          co_return make_unexpected(MavlinkErrc::FailedWrite);
        }
      }
      auto result = co_await (m_ap_requests.async_receive(use_nothrow_awaitable) || receive_message());
      if (std::holds_alternative<std::tuple<asio::error_code, mavlink_message_t>>(result)) {
        mavlink_message_t this_msg;
        tie(error, this_msg) = std::get<std::tuple<asio::error_code, mavlink_message_t>>(result);
        if (error) {
          spdlog::trace("Couldn't send msg, id: {}, asio error: {}", static_cast<unsigned int>(this_msg.msgid), error.message());   
          co_return make_unexpected(MavlinkErrc::FailedWrite);
        }
        tie(error, written) = co_await send_message(this_msg);
        if (error) {
          spdlog::trace("Couldn't send msg, id: {}, asio error: {}", static_cast<unsigned int>(this_msg.msgid), error.message());   
          co_return make_unexpected(MavlinkErrc::FailedWrite);
        }
      } else if (std::holds_alternative<std::error_code>(result)) {
      auto error = std::get<std::error_code>(result);
       if (error) {
          spdlog::trace("Couldn't receive_message: {}: {}", error.category().name(), error.message());
        }
      }
      // timer.expires_after(20ms);
      // co_await timer.async_wait(use_nothrow_awaitable);
    }
  }
  auto init() -> asio::awaitable<tResult<void>>
  {
    using std::ignore;
    using std::tie;

    std::error_code error;
    mavlink_message_t msg;
    const uint16_t mav_cmd_set_message_interval = 511;
    const float disable = -1;
    // auto set_param_curried = std::bind_front(mavlink_msg_command_long_pack_chan,
    //                                          m_system_id,
    //                                          m_my_id,
    //                                          m_channel,
    //                                          &msg,
    //                                          m_system_id,
    //                                          m_component_id,
    //                                          mav_cmd_set_message_interval,
    //                                          1);
    // auto set_param_curried = std::bind_front(mavlink_msg_param_set_pack_chan,
    //                                          m_system_id,
    //                                          m_my_id,
    //                                          m_channel,
    //                                          &msg,
    //                                          m_system_id,
    //                                          m_component_id);
    bool ran_once = false;
    while (not ran_once) // RUN THIS ONLY ONCE
    {
      ran_once = true;
      auto len = mavlink_msg_command_long_pack_chan(m_system_id, m_my_id, m_channel, &msg, m_system_id, m_component_id, mav_cmd_set_message_interval, 0, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000, INVALID, INVALID, INVALID, INVALID, INVALID);
      spdlog::debug("Sending 10Hz rate for Global Position");
      tie(error) = co_await m_ap_requests.async_send(asio::error_code{}, msg, use_nothrow_awaitable);
    }
    if (error) {
      spdlog::error("Could not initialize ardupilot params, asio error: {}",
                    error.message());
      co_return make_unexpected(MavlinkErrc::FailedWrite);
    }
  }
  auto nav_status() -> NavStatus const {
    return m_ap_state.m_nav;
  }
  auto local_position() -> std::span<float, 3> const {
    return std::span(m_ap_state.m_local_xyz);
  }
  auto global_position() -> std::span<int32_t, 3> const {
    return std::span(m_ap_state.m_lat_lon_alt);
  }
  auto global_linear_velocity() -> std::span<float, 3> const {
    return std::span(m_ap_state.m_global_vel);
  }
  auto heading() -> float const {
    return m_ap_state.m_heading_deg;
  }
  auto orientation() -> std::span<float, 3> const {
    return std::span(m_ap_state.m_rpy);
  }
  auto set_armed(int disarm = 0) -> asio::awaitable<void> {
    mavlink_message_t msg;
    const uint16_t mav_cmd_component_arm_disarm = 400;
    auto len = mavlink_msg_command_long_pack_chan(m_system_id,
                                                  m_my_id,
                                                  m_channel,
                                                  &msg,
                                                  m_system_id,
                                                  m_component_id,
                                                  mav_cmd_component_arm_disarm,
                                                  0,
                                                  (disarm) ? 0 : 1,
                                                  0,
                                                  0,
                                                  0,
                                                  0,
                                                  0,
                                                  0);
    spdlog::info("Sending ARM");
    auto [error] = co_await m_ap_requests.async_send(asio::error_code{}, msg, use_nothrow_awaitable);
    if (error) {
      spdlog::error("Could not send ARM, asio error: {}", error.message());
      // co_return make_unexpected(MavlinkErrc::FailedWrite);
    }
  }
  auto set_disarmed() {
    return set_armed(1);
  }
  auto set_hold_mode() -> asio::awaitable<void> {
    mavlink_message_t msg;
    const uint16_t mav_cmd_do_set_mode = 176;
    asio::steady_timer timer(co_await asio::this_coro::executor);
    for (int i = 0; i < 1; i++) {
      auto len = mavlink_msg_command_long_pack_chan(m_system_id,
                                                    m_my_id,
                                                    m_channel,
                                                    &msg,
                                                    m_system_id,
                                                    m_component_id,
                                                    mav_cmd_do_set_mode,
                                                    i,
                                                    1,
                                                    4,
                                                    0,
                                                    0,
                                                    0,
                                                    0,
                                                    0);
      spdlog::info("Sending hold");
      auto [error] = co_await m_ap_requests.async_send(asio::error_code{}, msg, use_nothrow_awaitable);
      // m_msg_queue.emplace(msg);
      // timer.expires_after(60ms);
      // co_await timer.async_wait(use_nothrow_awaitable);
      // auto [error, written] = co_await send_message(msg);
      if (error) {
        spdlog::error("Could not send set HOLD mode, asio error: {}", error.message());
        // co_return make_unexpected(MavlinkErrc::FailedWrite);
      }
    }
  } 
  auto set_guided_mode() -> asio::awaitable<void> {
    mavlink_message_t msg;
    const uint16_t mav_cmd_do_set_mode = 176;
    asio::steady_timer timer(co_await asio::this_coro::executor);
    for (int i = 0; i < 1; i++) {
      auto len = mavlink_msg_command_long_pack_chan(m_system_id,
                                                    m_my_id,
                                                    m_channel,
                                                    &msg,
                                                    m_system_id,
                                                    m_component_id,
                                                    mav_cmd_do_set_mode,
                                                    i,
                                                    1,
                                                    15,
                                                    0,
                                                    0,
                                                    0,
                                                    0,
                                                    0);
      spdlog::info("Sending guided");
      auto [error] = co_await m_ap_requests.async_send(asio::error_code{}, msg, use_nothrow_awaitable);
      // m_msg_queue.emplace(msg);
      // timer.expires_after(60ms);
      // co_await timer.async_wait(use_nothrow_awaitable);
      // auto [error, written] = co_await send_message(msg);
      if (error) {
        spdlog::error("Could not send set GUIDED mode, asio error: {}", error.message());
        // co_return make_unexpected(MavlinkErrc::FailedWrite);
      }
    }
  }
  auto set_target_velocity(std::span<float, 3> velxyz)
    -> asio::awaitable<void>
  {
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack_chan(
      m_system_id,
      m_my_id,
      m_channel,
      &msg,
      get_uptime(),
      m_system_id,
      m_component_id,
      MAV_FRAME_BODY_OFFSET_NED,
      USE_VELOCITY,
      INVALID,
      INVALID,
      INVALID,
      velxyz[0],
      velxyz[1],
      velxyz[2],
      INVALID,
      INVALID,
      INVALID,
      INVALID,
      INVALID);
    auto [error] = co_await m_ap_requests.async_send(asio::error_code{}, msg, use_nothrow_awaitable);
    // m_msg_queue.emplace(msg);
    // asio::steady_timer timer(co_await asio::this_coro::executor);
    // timer.expires_after(60ms);
    // co_await timer.async_wait(use_nothrow_awaitable);
    // auto [error, written] = co_await send_message(msg);
    if (error) {
      spdlog::error("Could not send set_target, asio error: {}\n",
                    error.message());
      // co_return make_unexpected(MavlinkErrc::FailedWrite);
    }
  }
  // ArduPilot doesn't respond to this message it seems use set_target_attitude instead
  auto set_target_yaw(float yaw) -> asio::awaitable<void> {
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack_chan(
      m_system_id,
      m_my_id,
      m_channel,
      &msg,
      get_uptime(),
      m_system_id,
      m_component_id,
      MAV_FRAME_BODY_OFFSET_NED,
      USE_YAW,
      INVALID,
      INVALID,
      INVALID,
      INVALID,
      INVALID,
      INVALID,
      INVALID,
      INVALID,
      INVALID,
      yaw,
      INVALID);
    auto [error] = co_await m_ap_requests.async_send(asio::error_code{}, msg, use_nothrow_awaitable);
    if (error) {
      spdlog::error("Could not send set_target, asio error: {}",
                    error.message());
      // co_return make_unexpected(MavlinkErrc::FailedWrite);
    }
    spdlog::debug("Sent yaw target");
  }
  auto set_target_position_local(std::span<float, 3> xyz)
    -> asio::awaitable<void>
  {
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack_chan(
      m_system_id,
      m_my_id,
      m_channel,
      &msg,
      get_uptime(),
      m_system_id,
      m_component_id,
      MAV_FRAME_LOCAL_NED,
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
    auto [error] = co_await m_ap_requests.async_send(asio::error_code{}, msg, use_nothrow_awaitable);
    // m_msg_queue.emplace(msg);
    // asio::steady_timer timer(co_await asio::this_coro::executor);
    // timer.expires_after(60ms);
    // co_await timer.async_wait(use_nothrow_awaitable);
    // auto [error, written] = co_await send_message(msg);
    // // TODO: add cancellation and time out here
    if (error) {
      spdlog::error("Could not send set_target, asio error: {}",
                    error.message());
      // co_return make_unexpected(MavlinkErrc::FailedWrite);
    }
  }
  auto set_target_attitude(std::span<float, 4> rotation_quaternion,
                           float thrust) -> asio::awaitable<void>
  {
    mavlink_message_t msg;
    const int8_t USE_ATTITUDE_THRUST = 0x27;
    float unused_thrust_body_field[3] = { INVALID, INVALID, INVALID };
    mavlink_msg_set_attitude_target_pack_chan(m_system_id,
                                              m_my_id,
                                              m_channel,
                                              &msg,
                                              get_uptime(),
                                              m_system_id,
                                              m_component_id,
                                              USE_ATTITUDE_THRUST,
                                              rotation_quaternion.data(),
                                              INVALID,
                                              INVALID,
                                              INVALID,
                                              thrust,
                                              nullptr);
    auto [error] = co_await m_ap_requests.async_send(asio::error_code{}, msg, use_nothrow_awaitable);
    // m_msg_queue.emplace(msg);
    // asio::steady_timer timer(co_await asio::this_coro::executor);
    // timer.expires_after(60ms);
    // co_await timer.async_wait(use_nothrow_awaitable);
    // auto [error, written] = co_await send_message(msg);
    if (error) {
      spdlog::error("Could not send set_target_attitude, asio error: {}\n",
                    error.message());
      // co_return make_unexpected(MavlinkErrc::FailedWrite);
    }
  }
  auto set_obstacle_distance(std::span<uint16_t, 72> distances,
                             float increment,
                             float min_distance,
                             float max_distance,
                             float offset) -> asio::awaitable<void>
  {
    mavlink_message_t msg;
    mavlink_msg_obstacle_distance_pack_chan(m_system_id,
                                            m_my_id,
                                            m_channel,
                                            &msg,
                                            get_uptime(),
                                            MAV_DISTANCE_SENSOR_LASER,
                                            distances.data(),
                                            INVALID,
                                            min_distance,
                                            max_distance,
                                            increment,
                                            offset,
                                            MAV_FRAME_BODY_FRD);
    auto [error] = co_await m_ap_requests.async_send(asio::error_code{}, msg, use_nothrow_awaitable);
    // m_msg_queue.emplace(msg);
    // asio::steady_timer timer(co_await asio::this_coro::executor);
    // timer.expires_after(60ms);
    // co_await timer.async_wait(use_nothrow_awaitable);
    // auto [error, written] = co_await send_message(msg);
    if (error) {
      spdlog::error("Could not send obstacle_distance, asio error: {}\n",
                   error.message());
      // co_return make_unexpected(MavlinkErrc::FailedWrite);
    }
  }
  auto heartbeat() -> mavlink_message_t
  {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack_chan(m_system_id,
                                    m_my_id,
                                    m_channel,
                                    &msg,
                                    MAV_TYPE_ONBOARD_CONTROLLER,
                                    MAV_AUTOPILOT_GENERIC,
                                    0,
                                    0,
                                    MAV_STATE_UNINIT);
    return msg;
    // auto [error, written] = co_await send_message(msg);
    // if (error) {
    //   spdlog::error("Could not send heartbeat, asio error: {}",
    //                 error.message());
    //   co_return make_unexpected(MavlinkErrc::FailedWrite);
    // }
    // spdlog::info("Sent heartbeat");
  }
};
