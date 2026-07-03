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

// Use "#pragma once" as an include guard for headers
// This is required because it ensures that the compiler will process this file
// only once, no matter how many times it is included.
#pragma once

#include <chrono>
#include <cstdint>

#include <array>
#include <numeric>
#include <utility>

#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>
#include <libhal/units.hpp>

namespace hal::actuator {

enum class dynamixel_servo : u8
{
  rx = 0,
  mx = 1,
};

/**
 * @brief Dynamixel RX-64 is a smart actuator capable of reporting position,
 * speed, and voltage. They also have the ability to set limits for angles,
 * voltage, torque, temperature, compliance margins and slopes, and response
 * delay.
 *
 * No downloadable datasheet available, link to documentation below.
 * https://docs.robotis.com/docs/dxl/model_reference/rx_series/rx-64/
 *
 */
class dynamixel_servo_protocol_1
{
public:
  /**
   * @brief Configuration object containing servo settings
   *
   */
  struct config
  {
    /// @brief The servo family that this driver is communicating with
    dynamixel_servo servo = dynamixel_servo::mx;
    /// @brief Baud rate to communicate with servo over serial
    hertz baud_rate = 57600;
    /// @brief Id of servo
    uint8_t id;
    /// @brief Minimum angle to limit movement to
    hal::degrees min_angle = 0;
    /// @brief Maximum angle to limit movement to
    hal::degrees max_angle = 300;
    /// @brief Enable torque on setup
    bool torque_enable = false;
    /// @brief Time before assuming that there was an error response from the
    /// motor
    hal::time_duration response_timeout = std::chrono::milliseconds(50);
  };

  /**
   * @brief Construct a new servo object
   *
   * @param p_serial Serial to use to communicate with servo
   * @param p_settings Configuration object containing settings to use
   * @param p_clock A steady clock used to add delays for communication
   */
  dynamixel_servo_protocol_1(hal::strong_ptr<hal::serial> const& p_serial,
                             hal::strong_ptr<hal::steady_clock> const& p_clock,
                             config p_settings);

  /**
   * @brief Check if an ID is in use.
   *
   * @param p_id - ID to check.
   * @param p_serial - Serial to use to communicate with servo
   * @param p_clock - A steady clock used to add delays for communication
   * @return true - Servo using this ID is present on bus.
   * @return false - No connected servos using this ID.
   */
  static bool ping_id(uint8_t p_id,
                      hal::strong_ptr<hal::serial> const& p_serial,
                      hal::strong_ptr<hal::steady_clock> const& p_clock);

  /**
   * @brief Iterate through all valid IDs and return the first ID that is
   * present. If none are found, 254 (the broadcast ID) will be returned.
   *
   * @param p_serial - Serial to use to communicate with servo
   * @param p_clock - A steady clock used to add delays for communication
   * @return u8 - ID of present servo or 254 if none found
   */
  static u8 scan_for_id(hal::strong_ptr<hal::serial> const& p_serial,
                        hal::strong_ptr<hal::steady_clock> const& p_clock);

  /**
   * @brief Execute commands registered with reg write function.
   *
   * @param p_id - ID of servo to communicate with or 0xFE for broadcast
   * @param p_serial - Serial to use to communicate with servo
   */
  static void execute_registered_action(
    uint8_t p_id,
    hal::strong_ptr<hal::serial> const& p_serial);

  /**
   * @brief Same as execute_registered_action but for only this servo
   *
   */
  void execute_action();

  /**
   * @brief Use ping command to check for errors.
   *
   */
  void ping_for_status();

  /**
   * @brief Toggle LED on or off.
   *
   * @param p_on On status of LED to set.
   */
  void led(bool p_on);

  /**
   * @brief Moving status of servo.
   *
   * @return true - servo is moving.
   * @return false - servo is not moving.
   */
  [[nodiscard]] bool is_moving();

  /**
   * @brief Get current speed in RPMs. Positive values indicate spinning
   * clockwise. Negative values indicate spinning counter clockwise.
   *
   * @return rpm - Current moving speed in RPMs.
   */
  rpm speed();

  /**
   * @brief Get current voltage supplied.
   *
   * @return volts - Current voltage supplied.
   */
  volts voltage();

  /**
   * @brief Get the internal temperature.
   *
   * @return uint8_t - Temperature in Celsius.
   */
  uint8_t temperature();

  /**
   * @brief Get the maximum torque available.
   *
   * @return float - Maximum torque available as a percentage.
   */
  float torque_limit();

  /**
   * @brief Get the torque enabled flag.
   *
   * @return true - Torque usage enabled.
   * @return false - Torque usage disabled.
   */
  bool torque_enable();

  /**
   * @brief Get the temperature limit before the Overheating Error flag is set
   * to true.
   *
   * @return uint8_t - Temperature limit in Celsius.
   */
  uint8_t temperature_limit();

  /**
   * @brief Get the minimum operating voltage.
   *
   * @return volts - Minimum usable operating voltage.
   */
  volts min_voltage();

  /**
   * @brief Get the maximum operating voltage.
   *
   * @return volts - Maximum usable operating voltage.
   */
  volts max_voltage();

  /**
   * @brief Get the baud rate used for serial communication.
   *
   * @return hertz - Baud rate in Hertz.
   */
  hertz baud_rate();

  /**
   * @brief Get the time between sending an instruction and receiveing a status
   * packet.
   *
   * @return std::chrono::microseconds - Time in microseconds to wait
   * before sending status packet.
   */
  std::chrono::microseconds return_delay_time();

  /**
   * @brief Get the unique ID.
   *
   * @return uint8_t - ID of the servo
   */
  uint8_t id();

  /**
   * @brief Get the minimum angle to restrain motion to.
   *
   * @return hal::degrees - Minimum angle in degrees.
   */
  hal::degrees min_angle();

  /**
   * @brief Get the maximum angle to restrain motion to.
   *
   * @return hal::degrees - Maximum angle in degrees.
   */
  hal::degrees max_angle();

  /**
   * @brief Get the current position.
   *
   * @return hal::degrees - Current position in degrees.
   */
  hal::degrees position();

  /**
   * @brief Get the punch, or minimum current needed to operate.
   *
   * @return uint16_t - Minimum current needed to operate.
   */
  uint16_t punch();

  /**
   * @brief Get the current speed in RPMs.
   *
   * @return float - Current speed in RPMs.
   */
  rpm moving_speed();

  /**
   * @brief Move to position.
   *
   * Angle to move to must be within range of min and max angle.
   *
   * @param p_angle - Angle to move to.
   */
  void position(hal::degrees p_angle);

  /**
   * @brief Register a position move to be executed when action command is
   * called.
   *
   * Angle to move to must be within range of min and max angle.
   *
   * @param p_angle - Angle to move to.
   */
  void queue_position(hal::degrees p_angle);

  /**
   * @brief Enable or disable torque usage.
   *
   * @param p_enable - Set to true to enable torque usage.
   */
  void torque_enable(bool p_enable);

  /**
   * @brief Set the maximum torque available to use.
   *
   * Range is 0.0 - 100.0, Values exceeding these bounds will be clamped to be
   * 0.0 when lower and 100.0 when higher. Example: if 50% torque limit is
   * desired, p_percent = 50.0
   *
   * @param p_percent - Percentage to set max torque to
   */
  void torque_limit(float p_percent);

  /**
   * @brief Set the maximum temperature.
   *
   * If internal temperature goes beyond this number, the Overheating Error flag
   * is set to true. Range is 0.0 - 100.0, Values exceeding these bounds will be
   * clamped to be 0.0 when lower and 100.0 when higher.
   *
   * @param p_temperature - Temperature in Celsius.
   */
  void temperature_limit(uint8_t p_temperature);

  /**
   * @brief Set the minimum operating voltage.
   *
   * If the input voltage is below this number, the Voltage Range Error flag is
   * set to true. Range is 5.0 - 25.0, Values exceeding these bounds will be
   * clamped to be 5.0 when lower and 25.0 when higher.
   *
   * @param p_voltage - Minimum operating voltage.
   */
  void min_voltage(volts p_voltage);

  /**
   * @brief Set the maximum operating voltage.
   *
   * If the input voltage is above this number, the Voltage Range Error flag is
   * set to true. Range is 5.0 - 25.0, Values exceeding these bounds will be
   * clamped to be 5.0 when lower and 25.0 when higher.
   *
   * @param p_voltage - Maximum operating voltage.
   */
  void max_voltage(volts p_voltage);

  /**
   * @brief Set the baud rate used for serial communication. Serial used to
   * communicate will also be changed to match the baud rate.
   *
   * Available baud rates are: 57600, 9600, 19200, 115200, 200000, 250000,
   * 400000, 500000, and 1000000. If an invalid baud rate is given, the default
   * baud rate of 57600 is used.
   *
   * @param p_baud - Baud rate used for serial communication
   */
  void baud_rate(hertz p_baud);

  /**
   * @brief Set the time between sending an instruction and receiveing a status
   * packet.
   *
   * Range is 0 - 508 microseconds. Values exceeding these bounds will be
   * clamped to be 0 when lower and 508 when higher.
   *
   * @param p_microseconds - Time in microseconds to wait
   */
  void return_delay_time(std::chrono::microseconds p_microseconds);

  /**
   * @brief Reassign the ID to use when communicating.
   *
   * Range is 0 - 253. ID 254 is reserved as the broadcast ID.
   *
   * @param p_id - ID to use.
   */
  void reassign_id(uint8_t p_id);

  /**
   * @brief Set the minimum angle to restrain motion to.
   *
   * Range of motion is 0.0 - 300.0, Values exceeding these bounds will be
   * clamped to be 0.0 when lower and 300.0 when higher.
   *
   * @param p_angle - Minimum angle used to restrain motion to.
   */
  void min_angle(hal::degrees p_angle);

  /**
   * @brief Set the maximum angle to restrain motion to.
   *
   * Range of motion is 0.0 - 300.0, Values exceeding these bounds will be
   * clamped to be 0.0 when lower and 300.0 when higher.
   *
   * @param p_angle - Maximum angle used to restrain motion to.
   */
  void max_angle(hal::degrees p_angle);

  /**
   * @brief Set the speed to use when moving.
   *
   * Range is 0 - 114 RPM. Values exceeding these bounds will be clamped to be 0
   * when lower and 114 when higher.
   *
   * @param p_rpms - Speed in RPM to use when moving
   */
  void speed(rpm p_rpms);

  /**
   * @brief Returns the last error code retrieved from the servo motor
   *
   * See https://docs.robotis.com/docs/dxl/protocol/protocol1/#error for error
   * code information.
   *
   * @return auto - error code byte
   */
  [[nodiscard]] auto last_error_code() const { return m_last_error; }

private:
  /**
   * @brief Addresses for registers of servo
   *
   */
  enum class registers : hal::byte
  {
    /// @brief Model number
    model_number = 0x00,
    /// @brief Firmware version
    firmware_ver = 0x02,
    /// @brief Unique ID
    id = 0x03,
    /// @brief Baud rate of serial communication between controller and servo
    baud_rate = 0x04,
    /// @brief Time between sending an instruction and receiveing a status
    /// packet
    return_delay = 0x05,
    /// @brief Clockwise limit or minimum angle
    cw_limit = 0x06,
    /// @brief Counter clockwise limit or maximum angle
    ccw_limit = 0x08,
    /// @brief Temperature limit
    temp_limit = 0x0B,
    /// @brief Minimum voltage needed for operation
    min_voltage = 0x0C,
    /// @brief Maximum voltage needed for operation
    max_voltage = 0x0D,
    /// @brief Maximum torque
    max_torque = 0x0E,
    /// @brief Status return level, what situations to send status packets
    status_return = 0x10,
    /// @brief Which errors cause LED to blink
    alarm_led = 0x11,
    /// @brief Which errors cause servo to shutdown
    shutdown = 0x12,
    /// @brief (MX-only) controls multi turn offset
    multi_turn_offset = 0x14,
    /// @brief (MX-only) controls resolution of the rotation sensor. By dividing
    /// the resolution, one can multiply the range of the servo.
    resolution_divider = 0x16,
    /// @brief Enable torque usage
    torque_enable = 0x18,
    /// @brief Toggle LED
    led_toggle = 0x19,

    /// @brief (MX-only) Derivative gain value of PID
    d_gain = 0x1A,
    /// @brief (MX-only) Integral gain value of PID
    i_gain = 0x1B,
    /// @brief (MX-only) Proportional band gain value of PID
    p_gain = 0x1C,

    /// @brief (RX-only) Margin error between goal position and present position
    /// in the clockwise direction
    cw_compliance_margin = 0x1A,
    /// @brief (RX-only) Margin error between goal position and present position
    /// in the counter clockwise direction
    ccw_compliance_margin = 0x1B,
    /// @brief (RX-only) Level of Torque near the goal position in clockwise
    /// direction
    cw_compliance_slope = 0x1C,
    /// @brief (RX-only) Level of Torque near the goal position in counter
    /// clockwise direction
    ccw_compliance_slope = 0x1D,

    /// @brief Position to move to
    goal_position = 0x1E,
    /// @brief Moving speed to goal position
    moving_speed = 0x20,
    /// @brief Torque Limit, value of max_torque as default
    torque_limit = 0x22,
    /// @brief Present position
    present_position = 0x24,
    /// @brief Present moving speed
    present_speed = 0x26,
    /// @brief Currently applied load
    present_load = 0x28,
    /// @brief Present voltage supplied
    present_voltage = 0x2A,
    /// @brief Internal temperature in Celsius
    present_temp = 0x2B,
    /// @brief Flag for if an instruction is registered for standby execution
    instruction_registered = 0x2C,
    /// @brief Is servo moving
    moving_status = 0x2E,
    /// @brief Lock EEPROM from modification
    lock_eeprom = 0x2F,
    /// @brief Minimum current to drive motor
    punch = 0x30,

    /// @brief (MX-only) Realtime clock of MX-64
    realtime_tick = 0x32,
    /// @brief (MX-only) Consuming current
    current = 0x44,
    /// @brief (MX-only) Torque control mode enable
    torque_ctrl_mode_enable = 0x46,
    /// @brief (MX-only) Goal torque applied before stopping
    goal_torque = 0x47,
    /// @brief (MX-only) Goal acceleration
    goal_accel = 0x49,
  };

  std::pair<u16, u16> position_range();

  void validate_response(std::span<byte const> p_response)
  {
    if (p_response[0] != 0xFF or p_response[1] != 0xFF) {
      hal::safe_throw(hal::io_error(this));
    }
    hal::byte const response_checksum =
      ~std::accumulate(p_response.begin() + 2, p_response.end() - 1, 0);
    if (response_checksum != p_response.back()) {
      hal::safe_throw(hal::io_error(this));
    }
  }

  template<usize Size>
  auto read_register(registers p_register_address)
  {
    using namespace std::chrono_literals;
    m_serial->flush();

    auto const address = static_cast<hal::byte>(p_register_address);
    auto const size_byte = static_cast<hal::byte>(Size);

    std::array<hal::byte, 8> send_bytes = { 0xFF, 0xFF,    m_id,      0x04,
                                            0x02, address, size_byte, 0x00 };
    hal::byte const checksum =
      std::accumulate(&send_bytes[2], &send_bytes[7], 0);
    send_bytes[7] = ~checksum;
    hal::write(*m_serial, send_bytes, hal::never_timeout());
    std::array<hal::byte, Size> return_array{};

    auto constexpr read_size = 6 + Size;

    auto const response = hal::read<read_size>(
      *m_serial, hal::create_timeout(*m_clock, m_response_timeout));
    validate_response(response);

    for (usize i = 0; i < return_array.size(); i++) {
      return_array[i] = response[5 + i];
    }
    return return_array;
  }

  template<usize Size>
  void write_register(registers p_register_address,
                      std::array<hal::byte, Size> p_data)
  {
    m_serial->flush();

    constexpr hal::byte packet_length = 0x03 + Size;
    auto const address = static_cast<hal::byte>(p_register_address);
    std::array<hal::byte, 6> header_bytes = { 0xFF,          0xFF, m_id,
                                              packet_length, 0x03, address };

    hal::byte checksum =
      std::accumulate(header_bytes.begin() + 2, header_bytes.end(), 0);
    checksum = std::accumulate(p_data.begin(), p_data.end(), checksum);
    checksum = ~checksum;

    m_last_error = 0;
    do {
      m_serial->flush();
      hal::write(*m_serial, header_bytes, hal::never_timeout());
      hal::write(*m_serial, p_data, hal::never_timeout());
      hal::write(*m_serial, std::array{ checksum }, hal::never_timeout());
      auto const response = hal::read<6>(
        *m_serial, hal::create_timeout(*m_clock, m_response_timeout));
      validate_response(response);
      m_last_error = response[4];
      // check if the last error had a checksum error
    } while (m_last_error & (1 << 4));
  }

  template<usize Size>
  void reg_write(registers p_register_address,
                 std::array<hal::byte, Size> p_data)
  {
    m_serial->flush();

    constexpr hal::byte packet_length = 0x03 + Size;
    auto const address = static_cast<hal::byte>(p_register_address);
    std::array<hal::byte, 6> header_bytes = { 0xFF,          0xFF, m_id,
                                              packet_length, 0x04, address };

    hal::byte checksum =
      std::accumulate(header_bytes.begin() + 2, header_bytes.end(), 0);
    checksum = std::accumulate(p_data.begin(), p_data.end(), checksum);
    checksum = ~checksum;

    m_last_error = 0;
    hal::write(*m_serial, header_bytes, hal::never_timeout());
    hal::write(*m_serial, p_data, hal::never_timeout());
    hal::write(*m_serial, std::array{ checksum }, hal::never_timeout());

    auto const response = hal::read<6>(
      *m_serial, hal::create_timeout(*m_clock, m_response_timeout));
    validate_response(response);
    m_last_error = response[4];

    // check if the last error had a checksum error
    if (m_last_error & (1 << 4)) {
      hal::safe_throw(hal::io_error(this));
    }
  }

  hal::strong_ptr<hal::serial> m_serial;
  hal::strong_ptr<hal::steady_clock> m_clock;
  std::pair<hal::degrees, hal::degrees> m_range;
  hal::time_duration m_response_timeout;
  hal::byte m_id;
  hal::byte m_last_error;
  dynamixel_servo m_servo_type = dynamixel_servo::mx;
};
} // namespace hal::actuator
