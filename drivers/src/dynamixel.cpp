// Copyright 2026 Malia Labor and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <bitset>
#include <chrono>
#include <numeric>
#include <utility>

#include <libhal-util/bit.hpp>
#include <libhal-util/map.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>

#include <dynamixel.hpp>

namespace hal::actuator {

auto
position_raw_range(dynamixel_servo p_servo_type)
{
  switch (p_servo_type) {
    case dynamixel_servo::rx:
      return std::make_pair<u16, u16>(0, 1023);
    case dynamixel_servo::mx:
      return std::make_pair<u16, u16>(0, 4095);
  }
}

auto
max_degree_range(dynamixel_servo p_servo_type)
{
  switch (p_servo_type) {
    case dynamixel_servo::rx:
      return std::make_pair<hal::degrees, hal::degrees>(0.0f, 300.0f);
    case dynamixel_servo::mx:
      return std::make_pair<hal::degrees, hal::degrees>(0.0f, 360.0f);
  }
}
constexpr auto rpm_range = std::make_pair<rpm, rpm>(0, 114);
constexpr auto rpm_raw_range = std::make_pair<u16, u16>(0, 1023);

dynamixel_servo_protocol_1::dynamixel_servo_protocol_1(
  hal::strong_ptr<hal::serial> const& p_serial,
  hal::strong_ptr<hal::steady_clock> const& p_clock,
  config p_settings)
  : m_serial(p_serial)
  , m_clock(p_clock)
  , m_response_timeout(p_settings.response_timeout)
  , m_id(p_settings.id)
  , m_servo_type(p_settings.servo)
{
  using namespace std::chrono_literals;
  m_serial->configure({ .baud_rate = p_settings.baud_rate });
  m_range.first = p_settings.min_angle;
  m_range.second = p_settings.max_angle;
  if (not ping_id(m_id, m_serial, m_clock)) {
    safe_throw(hal::io_error(this));
  }
}

bool
dynamixel_servo_protocol_1::ping_id(
  u8 p_id,
  hal::strong_ptr<hal::serial> const& p_serial,
  hal::strong_ptr<hal::steady_clock> const& p_clock)
{
  using namespace std::chrono_literals;
  std::array<hal::byte, 6> send_bytes = { 0xFF, 0xFF, p_id, 0x02, 0x01, 0x00 };
  hal::byte const checksum = std::accumulate(&send_bytes[2], &send_bytes[5], 0);
  send_bytes[5] = ~checksum;
  hal::write(*p_serial, send_bytes, hal::never_timeout());

  try {
    auto response =
      hal::read<6>(*p_serial, hal::create_timeout(*p_clock, 50ms));
    if (response[0] == 0xFF && response[1] == 0xFF) {
      // device responded, id is valid
      return true;
    }
  } catch (hal::timed_out const&) {
    return false;
  }
  return true;
}

u8
dynamixel_servo_protocol_1::scan_for_id(
  hal::strong_ptr<hal::serial> const& p_serial,
  hal::strong_ptr<hal::steady_clock> const& p_clock)
{
  bool device_found = false;
  for (u8 i = 0; i < 254; i++) {
    device_found = dynamixel_servo_protocol_1::ping_id(i, p_serial, p_clock);
    if (device_found) {
      return i;
    }
  }
  return 254;
}

void
dynamixel_servo_protocol_1::broadcast_execute_action(
  hal::strong_ptr<hal::serial> const& p_serial)
{
  using namespace std::chrono_literals;
  std::array<hal::byte, 6> send_bytes = { 0xFF, 0xFF, 0xFE, 0x02, 0x05, 0x00 };
  hal::byte const checksum = std::accumulate(&send_bytes[2], &send_bytes[5], 0);
  send_bytes[5] = ~checksum;
  hal::write(*p_serial, send_bytes, hal::never_timeout());
}

void
dynamixel_servo_protocol_1::ping_for_status()
{
  using namespace std::chrono_literals;
  std::array<hal::byte, 6> send_bytes = { 0xFF, 0xFF, m_id, 0x02, 0x01, 0x00 };
  hal::byte const checksum = std::accumulate(&send_bytes[2], &send_bytes[5], 0);
  send_bytes[5] = ~checksum;
  hal::write(*m_serial, send_bytes, hal::never_timeout());

  m_last_error = 0;

  try {
    auto response =
      hal::read<6>(*m_serial, hal::create_timeout(*m_clock, 50ms));
    if (response[0] == 0xFF && response[1] == 0xFF) {
      // device responded, id is valid
      m_last_error = response[4];
    }
  } catch (hal::timed_out const&) {
    m_last_error = 1 << 4; // set error to
  }
}

void
dynamixel_servo_protocol_1::led(bool p_on)
{
  write_register(registers::led_toggle, std::array{ static_cast<byte>(p_on) });
}

bool
dynamixel_servo_protocol_1::is_moving()
{
  auto const response =
    dynamixel_servo_protocol_1::read_register<1>(registers::moving_status);
  if (response[0] == 0x01) {
    return true;
  }
  return false;
}

float
dynamixel_servo_protocol_1::speed()
{
  auto const bytes =
    dynamixel_servo_protocol_1::read_register<2>(registers::present_speed);
  u16 const response = (bytes[0] | (bytes[1] << 8));
  std::bitset<16> bits{ response };
  bool const clockwise = bits[9]; // 10th bit is direction
  bits.set(9, false);
  float const rpms = hal::map(response, rpm_raw_range, rpm_range);
  if (clockwise) {
    return rpms;
  }
  return -rpms;
}

float
dynamixel_servo_protocol_1::voltage()
{
  auto const response =
    dynamixel_servo_protocol_1::read_register<1>(registers::present_voltage)[0];
  return static_cast<float>(response) / 10;
}

u8
dynamixel_servo_protocol_1::temperature()
{
  return dynamixel_servo_protocol_1::read_register<1>(
    registers::present_temp)[0];
}

float
dynamixel_servo_protocol_1::torque_limit()
{
  auto const bytes =
    dynamixel_servo_protocol_1::read_register<2>(registers::torque_limit);
  u16 const response = (bytes[0] | (bytes[1] << 8));
  return (static_cast<float>(response) / 1023) * 100.0f;
}

bool
dynamixel_servo_protocol_1::torque_enable()
{
  auto const enabled_byte = dynamixel_servo_protocol_1::read_register<1>(
    dynamixel_servo_protocol_1::registers::torque_enable)[0];
  return enabled_byte == 0x01;
}

u8
dynamixel_servo_protocol_1::temperature_limit()
{
  return dynamixel_servo_protocol_1::read_register<1>(registers::temp_limit)[0];
}

float
dynamixel_servo_protocol_1::min_voltage()
{
  auto const response =
    dynamixel_servo_protocol_1::read_register<1>(registers::min_voltage)[0];
  return (static_cast<float>(response) / 10);
}

float
dynamixel_servo_protocol_1::max_voltage()
{
  auto const response =
    dynamixel_servo_protocol_1::read_register<1>(registers::max_voltage)[0];
  return (static_cast<float>(response) / 10);
}

hertz
dynamixel_servo_protocol_1::baud_rate()
{
  auto const response =
    dynamixel_servo_protocol_1::read_register<1>(registers::baud_rate)[0];
  switch (response) {
    case 1:
      return 1000000;
    case 3:
      return 500000;
    case 4:
      return 400000;
    case 7:
      return 250000;
    case 9:
      return 200000;
    case 16:
      return 115200;
    case 103:
      return 19200;
    case 207:
      return 9600;
    case 34:
    default:
      return 57600;
  }
}

std::chrono::microseconds
dynamixel_servo_protocol_1::return_delay_time()
{
  auto const response =
    dynamixel_servo_protocol_1::read_register<1>(registers::return_delay)[0];
  return std::chrono::microseconds(response * 2);
}

u8
dynamixel_servo_protocol_1::id()
{
  return m_id;
}

hal::degrees
dynamixel_servo_protocol_1::min_angle()
{
  auto const bytes =
    dynamixel_servo_protocol_1::read_register<2>(registers::cw_limit);
  u16 const angle_byte = (bytes[0] | (bytes[1] << 8));
  return hal::map(angle_byte,
                  position_raw_range(m_servo_type),
                  max_degree_range(m_servo_type));
}

hal::degrees
dynamixel_servo_protocol_1::max_angle()
{
  auto const bytes =
    dynamixel_servo_protocol_1::read_register<2>(registers::ccw_limit);
  u16 const angle_byte = (bytes[0] | (bytes[1] << 8));
  return hal::map(angle_byte,
                  position_raw_range(m_servo_type),
                  max_degree_range(m_servo_type));
}

hal::degrees
dynamixel_servo_protocol_1::position()
{
  auto const bytes =
    dynamixel_servo_protocol_1::read_register<2>(registers::present_position);
  u16 const angle_byte = (bytes[0] | (bytes[1] << 8));
  return hal::map(angle_byte,
                  position_raw_range(m_servo_type),
                  max_degree_range(m_servo_type));
}

u16
dynamixel_servo_protocol_1::punch()
{
  auto const bytes =
    dynamixel_servo_protocol_1::read_register<2>(registers::punch);
  return (bytes[0] | (bytes[1] << 8));
}

rpm
dynamixel_servo_protocol_1::moving_speed()
{
  auto const bytes =
    dynamixel_servo_protocol_1::read_register<2>(registers::moving_speed);
  u16 const response = (bytes[0] | (bytes[1] << 8));
  return hal::map(response, rpm_raw_range, rpm_range);
}

void
dynamixel_servo_protocol_1::position(hal::degrees p_angle)
{
  auto const clamped_angle = std::clamp(p_angle, m_range.first, m_range.second);
  auto const angle_byte =
    static_cast<u16>(hal::map(clamped_angle,
                              max_degree_range(m_servo_type),
                              position_raw_range(m_servo_type)));
  hal::byte const value_low = angle_byte;
  hal::byte const value_hi = (angle_byte >> 8);
  write_register(registers::goal_position, std::array{ value_low, value_hi });
}

void
dynamixel_servo_protocol_1::execute_action()
{
  using namespace std::chrono_literals;
  std::array<hal::byte, 6> send_bytes = { 0xFF, 0xFF, m_id, 0x02, 0x05, 0x00 };
  hal::byte const checksum = std::accumulate(&send_bytes[2], &send_bytes[5], 0);
  send_bytes[5] = ~checksum;

  m_last_error = 0;
  do {
    m_serial->flush();
    hal::write(*m_serial, send_bytes, hal::never_timeout());
    auto const response = hal::read<6>(
      *m_serial, hal::create_timeout(*m_clock, m_response_timeout));
    validate_response(response);
    m_last_error = response[4];
    // check if the last error had a checksum error
  } while (m_last_error & (1 << 4));
}

void
dynamixel_servo_protocol_1::queue_position(hal::degrees p_angle)
{
  auto const clamped_angle = std::clamp(p_angle, m_range.first, m_range.second);
  auto const angle_byte =
    static_cast<u16>(hal::map(clamped_angle,
                              max_degree_range(m_servo_type),
                              position_raw_range(m_servo_type)));
  hal::byte const value_low = angle_byte;
  hal::byte const value_hi = (angle_byte >> 8);
  reg_write(registers::goal_position, std::array{ value_low, value_hi });
}

void
dynamixel_servo_protocol_1::torque_enable(bool p_enable)
{
  write_register(registers::led_toggle,
                 std::array{ static_cast<byte>(p_enable) });
}

void
dynamixel_servo_protocol_1::torque_limit(float p_percent)
{
  auto const clamped_percent = std::clamp(p_percent, 0.0f, 100.0f);
  auto const value = static_cast<u16>(hal::map(
    clamped_percent, std::make_pair(0.0f, 100.0f), std::make_pair(0, 1023)));
  hal::byte const value_low = value;
  hal::byte const value_hi = (value >> 8);
  write_register(registers::torque_limit, std::array{ value_low, value_hi });
}

void
dynamixel_servo_protocol_1::temperature_limit(u8 p_temperature)
{
  auto const clamped_temp =
    std::clamp(p_temperature, static_cast<u8>(0), static_cast<u8>(100));
  write_register(registers::temp_limit, std::array{ clamped_temp });
}

void
dynamixel_servo_protocol_1::min_voltage(float p_voltage)
{
  auto const clamped_volt = std::clamp(p_voltage, 5.0f, 25.0f);
  auto const value = static_cast<u8>(clamped_volt * 10);
  write_register(registers::min_voltage, std::array{ value });
}

void
dynamixel_servo_protocol_1::max_voltage(float p_voltage)
{
  auto const clamped_volt = std::clamp(p_voltage, 5.0f, 25.0f);
  auto const value = static_cast<u8>(clamped_volt * 10);
  write_register(registers::max_voltage, std::array{ value });
}

void
dynamixel_servo_protocol_1::baud_rate(hertz p_baud)
{
  u8 baud_byte = 0x00;
  switch (static_cast<u32>(p_baud)) {
    case 1000000:
      baud_byte = 1;
      break;
    case 500000:
      baud_byte = 3;
      break;
    case 400000:
      baud_byte = 4;
      break;
    case 250000:
      baud_byte = 7;
      break;
    case 200000:
      baud_byte = 9;
      break;
    case 115200:
      baud_byte = 16;
      break;
    case 19200:
      baud_byte = 103;
      break;
    case 9600:
      baud_byte = 207;
      break;
    case 57600:
    default:
      baud_byte = 34;
      break;
  }
  write_register(registers::baud_rate, std::array{ baud_byte });
  m_serial->configure({ .baud_rate = p_baud });
}

void
dynamixel_servo_protocol_1::return_delay_time(
  std::chrono::microseconds p_microseconds)
{
  auto const value = static_cast<int>(p_microseconds.count() / 2);
  auto const clamped_value = static_cast<u8>(std::clamp(value, 0, 254));
  write_register(registers::return_delay, std::array{ clamped_value });
}

void
dynamixel_servo_protocol_1::reassign_id(u8 p_id)
{
  m_id = std::clamp(p_id, static_cast<u8>(0), static_cast<u8>(253));
  write_register(registers::id, std::array{ p_id });
}

void
dynamixel_servo_protocol_1::min_angle(hal::degrees p_angle)
{
  m_range.first = std::clamp(p_angle,
                             max_degree_range(m_servo_type).first,
                             max_degree_range(m_servo_type).second);
  auto const angle_byte =
    static_cast<u16>(hal::map(m_range.first,
                              max_degree_range(m_servo_type),
                              position_raw_range(m_servo_type)));
  hal::byte const value_low = angle_byte;
  hal::byte const value_hi = (angle_byte >> 8);
  write_register(registers::cw_limit, std::array{ value_low, value_hi });
}

void
dynamixel_servo_protocol_1::max_angle(hal::degrees p_angle)
{
  m_range.second = std::clamp(p_angle,
                              max_degree_range(m_servo_type).first,
                              max_degree_range(m_servo_type).second);
  auto const angle_byte =
    static_cast<u16>(hal::map(m_range.second,
                              max_degree_range(m_servo_type),
                              position_raw_range(m_servo_type)));
  hal::byte const value_low = angle_byte;
  hal::byte const value_hi = (angle_byte >> 8);
  write_register(registers::ccw_limit, std::array{ value_low, value_hi });
}

void
dynamixel_servo_protocol_1::speed(float p_rpms)
{
  auto const clamped_rpm =
    std::clamp(p_rpms, rpm_range.first, rpm_range.second);
  auto const speed_byte =
    static_cast<u16>(hal::map(clamped_rpm, rpm_range, rpm_raw_range));
  hal::byte const value_low = speed_byte;
  hal::byte const value_hi = (speed_byte >> 8);
  write_register(registers::moving_speed, std::array{ value_low, value_hi });
}
} // namespace hal::actuator
