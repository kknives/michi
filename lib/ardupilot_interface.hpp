#pragma once

#include "expected.hpp"
#include <algorithm>
#include <queue>
#include "common.hpp"
#include <chrono>
#include <asio/serial_port.hpp>
#include <asio/this_coro.hpp>
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

struct ArdupilotState {
  std::array<float, 3> m_local_xyz;
  std::array<int32_t, 3> m_lat_lon_alt;
  std::array<float, 3> m_global_vel;
  std::array<float, 3> m_rpy;
  std::array<float, 3> m_rpy_vel;
};

template <typename I>
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
  std::queue<mavlink_message_t> m_msg_queue;

  inline auto get_uptime() -> uint32_t
  {
    return duration_cast<milliseconds>(steady_clock::now() - m_start).count();
  }
  auto send_message(const mavlink_message_t& msg)
    -> asio::awaitable<std::tuple<asio::error_code, std::size_t>>
  {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    auto len = mavlink_msg_to_send_buffer(buffer, &msg);
    std::vector<uint8_t> printl(len);
    std::copy(buffer, buffer+len, begin(printl));
    spdlog::info("Raw msg of len {}: {:x}", len, fmt::join(printl, " "));
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
  auto update_attitude(const mavlink_message_t* msg) -> void {
    mavlink_attitude_t att;
    mavlink_msg_attitude_decode(msg, &att);
    m_ap_state.m_rpy = {att.roll, att.pitch, att.yaw};
    m_ap_state.m_rpy_vel = {att.rollspeed, att.pitchspeed, att.yawspeed};
  }
  auto handle_message(const mavlink_message_t* msg)
  {
    // spdlog::info("Got message with ID {}, system {}", msg->msgid,
    // msg->sysid);
    if (msg->sysid != 1)
      return; // Only handling messages from autopilot
    switch (msg->msgid) {
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
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV:
        spdlog::trace("Got Global Position");
        update_global_position(msg);
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
  auto receive_message() -> asio::awaitable<tResult<void>> {
    std::vector<uint8_t> buffer(MAVLINK_MAX_PACKET_LEN);
    auto [error, len] = co_await m_uart.async_read_some(
      asio::buffer(buffer), use_nothrow_awaitable);
    if (error)
      co_return make_unexpected(error);
    mavlink_message_t msg;
    mavlink_status_t status;
    for (int i = 0; i < len; i++) {
      if (mavlink_parse_char(m_channel, buffer[i], &msg, &status))
        break;
    }
    handle_message(&msg);
  }

public:
  MavlinkInterface(I && sp)
    : m_uart{ std::move(sp) }
    , m_start{ steady_clock::now() }
  {
    // m_uart.set_option(asio::serial_port_base::baud_rate(115200));
  }
  auto loop() -> asio::awaitable<tResult<void>> {
    mavlink_message_t hb_msg = heartbeat();
    auto [error, written] = co_await send_message(hb_msg);
    if (error) {
      spdlog::error("Couldn't send first heartbeat, asio error: {}", error.message());
      co_return make_unexpected(MavlinkErrc::FailedWrite);
    }
    auto last_heartbeat = steady_clock::now();
    while (true) {
      if (steady_clock::now() > last_heartbeat + 1s) {
        hb_msg = heartbeat();
        tie(error, written) = co_await send_message(hb_msg);
        if (error) spdlog::error("Couldn't send heartbeat, asio error: {}", error.message());   
      }
      if (not m_msg_queue.empty()) {
        auto this_msg = m_msg_queue.front();
        tie(error, written) = co_await send_message(this_msg);
        if (error) spdlog::error("Couldn't send msg, id: {}, asio error: {}", static_cast<unsigned int>(this_msg.msgid), error.message());   
        m_msg_queue.pop();
      }
      auto result = co_await receive_message();
      result.map_error([](std::error_code e) {
        spdlog::error("Couldn't receive_message: {}: {}", e.category().name(), e.message());
      });
    }
  }
  auto receive_message_loop() -> asio::awaitable<tResult<void>>
  {
    std::vector<uint8_t> buffer(8);
    asio::steady_timer timer(co_await asio::this_coro::executor);
    while (true) {
      auto [error, len] = co_await m_uart.async_read_some(
        asio::buffer(buffer), use_nothrow_awaitable);
      if (error)
        co_return make_unexpected(error);
      mavlink_message_t msg;
      mavlink_status_t status;
      for (int i = 0; i < len; i++) {
        if (mavlink_parse_char(m_channel, buffer[i], &msg, &status))
          break;
      }
      handle_message(&msg);
    }
  }
  auto init() -> asio::awaitable<tResult<void>>
  {
    using std::ignore;
    using std::tie;

    std::error_code error;
    mavlink_message_t msg;
    auto set_param_curried = std::bind_front(mavlink_msg_param_set_pack_chan,
                                             m_system_id,
                                             m_my_id,
                                             m_channel,
                                             &msg,
                                             m_system_id,
                                             m_component_id);
    bool ran_once = false;
    while (not ran_once) // RUN THIS ONLY ONCE
    {
      ran_once = true;
      set_param_curried("SR0_RAW_SENS", 0, MAV_PARAM_TYPE_INT16);
      tie(error, ignore) = co_await send_message(msg);
      if (error)
        break;

      set_param_curried("SR0_EXT_STAT", 0, MAV_PARAM_TYPE_INT16);
      tie(error, ignore) = co_await send_message(msg);
      if (error)
        break;

      set_param_curried("SR0_RC_CHAN", 0, MAV_PARAM_TYPE_INT16);
      tie(error, ignore) = co_await send_message(msg);
      if (error)
        break;

      set_param_curried("SR0_RAW_CTRL", 0, MAV_PARAM_TYPE_INT16);
      tie(error, ignore) = co_await send_message(msg);
      if (error)
        break;

      set_param_curried("SR0_POSITION", 0, MAV_PARAM_TYPE_INT16);
      tie(error, ignore) = co_await send_message(msg);
      if (error)
        break;

      set_param_curried("SR0_EXTRA1", 0, MAV_PARAM_TYPE_INT16);
      tie(error, ignore) = co_await send_message(msg);
      if (error)
        break;

      set_param_curried("SR0_EXTRA2", 0, MAV_PARAM_TYPE_INT16);
      tie(error, ignore) = co_await send_message(msg);
      if (error)
        break;

      set_param_curried("SR0_EXTRA3", 0, MAV_PARAM_TYPE_INT16);
      tie(error, ignore) = co_await send_message(msg);
      if (error)
        break;

      set_param_curried("SR0_PARAMS", 0, MAV_PARAM_TYPE_INT16);
      tie(error, ignore) = co_await send_message(msg);
      if (error)
        break;

      set_param_curried("SR0_ADSB", 0, MAV_PARAM_TYPE_INT16);
      // mavlink_msg_param_set_pack_chan(m_system_id, m_my_id,
      // m_channel, &msg, m_system_id, m_component_id, "SR0_ADSB", 0,
      // MAV_PARAM_TYPE_INT16);
      tie(error, ignore) = co_await send_message(msg);
      if (error)
        break;
      spdlog::info("Sent parameters");

      for (int i = 0; i < 4; i++) {
        const uint16_t mav_cmd_preflight_reboot_shutdown = 246;
        mavlink_msg_command_int_pack_chan(m_system_id,
                                          m_my_id,
                                          m_channel,
                                          &msg,
                                          m_system_id,
                                          m_component_id,
                                          MAV_FRAME_LOCAL_NED,
                                          mav_cmd_preflight_reboot_shutdown,
                                          0,  // Unused
                                          0,  // Unused
                                          1,  // Reboot autopilot
                                          0,  // Don't reboot companion computer
                                          0,  // Don't reboot componets
                                          0,  // For all components attached
                                          0,  // Unused
                                          0,  // Unused
                                          0); // Unused
        tie(error, ignore) = co_await send_message(msg);
        if (error)
          break;
        spdlog::info("Sent reboot");
      }
    }
    if (error) {
      spdlog::error("Could not initialize ardupilot params, asio error: {}",
                    error.message());
      co_return make_unexpected(MavlinkErrc::FailedWrite);
    }
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
  auto set_guided_mode_armed() -> void {
    mavlink_message_t msg;
    const uint16_t mav_cmd_do_set_mode = 176;
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
      m_msg_queue.emplace(msg);
      // auto [error, written] = co_await send_message(msg);
      // if (error) {
      //   spdlog::error("Could not send set and arm GUIDED mode, asio error: {}", error.message());
      //   co_return make_unexpected(MavlinkErrc::FailedWrite);
      // }
    }
  }
  auto set_target_velocity(std::span<float, 3> velxyz)
    -> void
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
    m_msg_queue.emplace(msg);
    // auto [error, written] = co_await send_message(msg);
    // if (error) {
    //   spdlog::error("Could not send set_target, asio error: {}\n",
    //                 error.message());
    //   co_return make_unexpected(MavlinkErrc::FailedWrite);
    // }
  }
  auto set_target_position_local(std::span<float, 3> xyz)
    -> void
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
    m_msg_queue.emplace(msg);
    // auto [error, written] = co_await send_message(msg);
    // // TODO: add cancellation and time out here
    // if (error) {
    //   spdlog::error("Could not send set_target, asio error: {}",
    //                 error.message());
    //   co_return make_unexpected(MavlinkErrc::FailedWrite);
    // }
  }
  auto set_target_attitude(std::span<float, 4> rotation_quaternion,
                           float yaw_rate,
                           float thrust) -> void
  {
    mavlink_message_t msg;
    const int8_t USE_YAWRATE_ATTITUDE_THRUST = 0x23;
    float unused_thrust_body_field[3] = { INVALID, INVALID, INVALID };
    mavlink_msg_set_attitude_target_pack_chan(m_system_id,
                                              m_my_id,
                                              m_channel,
                                              &msg,
                                              get_uptime(),
                                              m_system_id,
                                              m_component_id,
                                              USE_YAWRATE_ATTITUDE_THRUST,
                                              rotation_quaternion.data(),
                                              INVALID,
                                              INVALID,
                                              yaw_rate,
                                              thrust,
                                              unused_thrust_body_field);
    m_msg_queue.emplace(msg);
    // auto [error, written] = co_await send_message(msg);
    // if (error) {
    //   spdlog::error("Could not send set_attitude, asio error: {}\n",
    //                 error.message());
    //   co_return make_unexpected(MavlinkErrc::FailedWrite);
    // }
  }
  auto set_obstacle_distance(std::span<uint16_t, 72> distances,
                             float increment,
                             float min_distance,
                             float max_distance,
                             float offset) -> void
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
    m_msg_queue.emplace(msg);
    // auto [error, written] = co_await send_message(msg);
    // if (error) {
    //   spdlog::error("Could not send obstacle_distance, asio error: {}\n",
    //                error.message());
    //   co_return make_unexpected(MavlinkErrc::FailedWrite);
    // }
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

auto
heartbeat_loop(auto& mi) -> asio::awaitable<tResult<void>>
{
  asio::steady_timer timer(co_await asio::this_coro::executor);
  while (true) {
    timer.expires_at(steady_clock::now() + 1s);
    co_await timer.async_wait(use_nothrow_awaitable);
    auto result = co_await mi.heartbeat();
    if (not result)
      co_return result;
  }
}