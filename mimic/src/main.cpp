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

#include <cinttypes>
#include <cstddef>
#include <cstdint>

#include <array>
#include <bitset>
#include <utility>

#include <libhal-actuator/smart_servo/rmd/drc_v2.hpp>
#include <libhal-expander/tca9548a.hpp>
#include <libhal-sensor/as5600.hpp>
#include <libhal-util/map.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

#include <dynamixel.hpp>
#include <resource_list.hpp>

#define KEEP_MIMIC true
#define KEEP_ARM true
#define KEEP_PUMP true

#if KEEP_ARM
#define ARM_INFO(status) status
#define NO_ARM_INFO ""
#else
#define ARM_INFO(status) 0
#define NO_ARM_INFO ""
#endif

// ANSI Escape Codes for terminal manipulation
// Clears screen and moves cursor to top-left
#define CLEAR_SCREEN "\x1B[2J\x1B[H"
// Moves cursor to top-left without clearing (prevents flickering)
#define CURSOR_HOME "\x1B[H"

struct angle_select
{
  enum angle_names : uint8_t
  {
    spin = 0,
    shoulder_angle = 1,
    elbow_angle = 2,
    wrist_angle = 3,
    t_spin = 4,
    t_shoulder = 5,
    t_elbow = 6,
    t_wrist = 7
  };
};

hal::degrees process_angle(hal::degrees p_angle, hal::degrees p_servo_center)
{
  hal::degrees const sensor_center = 270.0f;
  // sensor cycles back to 0, but limit to only 180 degrees
  if (p_angle < 15.0f) {
    p_angle = 360.0f;
  }
  p_angle = std::clamp(p_angle, 180.0f, 360.0f);
  hal::degrees angle_diff = sensor_center - p_angle;
  return p_servo_center - angle_diff;
}

hal::degrees process_throttle_angle(hal::degrees p_angle,
                                    hal::degrees p_servo_center)
{
  hal::degrees const sensor_center = 180.0f;
  p_angle = std::clamp(p_angle, 90.0f, 270.0f);
  hal::degrees angle_diff = sensor_center - p_angle;
  return p_servo_center + angle_diff;
}

struct input_pin_inverter : public hal::input_pin
{
  input_pin_inverter(hal::strong_ptr<hal::input_pin> const& p_input_pin)
    : m_input_pin(p_input_pin)
  {
  }

  void driver_configure(hal::input_pin::settings const& p_settings)
  {
    m_input_pin->configure(p_settings);
  }

  bool driver_level()
  {
    return !m_input_pin->level();
  }

  hal::strong_ptr<hal::input_pin> m_input_pin;
};

// Pump states for the dashboard
enum class pump_state
{
  idle,
  pumping,
  atmosphere
};

int main()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  initialize_platform();

  auto const clock = resources::clock();
  auto const console = resources::console();
  hal::print(*console, "OpenSauce Mimic/Robot Arm Application Starting...\n");

  auto const i2c = resources::i2c();
  auto const uart = resources::uart2();
  // Connected to G0 on micromod
  auto const pump_button_inverted = resources::pump_button();
  input_pin_inverter pump_button(pump_button_inverted);
  // Connected to G1 on micromod
  auto const pump_power = resources::pump_power();
  pump_button.configure({ .resistor = hal::pin_resistor::pull_up });
  // Connected to G2 on micromod
  auto const valve_open = resources::valve_control();
  // Connected to G4 on micromod
  auto const control_switch = resources::control_switch();
  control_switch->configure({ .resistor = hal::pin_resistor::pull_up });
  auto const status_led = resources::status_led();
  hal::print(*console, "Acquired all resources prior to can...\n");
  auto const can_transceiver = resources::can_transceiver();
  auto const can_bus_manager = resources::can_bus_manager();
  auto const can_identifier_filter = resources::can_identifier_filter();
  hal::print(*console, "All resources acquired!\n");

  // Needs to be set to this baud rate to work with the default firmware CAN
  // baud rate.
  can_bus_manager->baud_rate(1.0_MHz);
  hal::print(*console, "Set can baud rate to 1 MHz\n");

  constexpr char const* success = "✅";
  [[maybe_unused]] constexpr char const* timeout = "⏰";
  [[maybe_unused]] constexpr char const* io_error = "📡";
  [[maybe_unused]] constexpr char const* unkown = "⁉️";

  char const* e_status = "❌";
  char const* w_status = "❌";
  char const* s_status = "❌";
  char const* o_status = "❌";
  [[maybe_unused]] char const* b_status = "✅";  // Bus is usually active/ready

  pump_state current_pump_mode = pump_state::idle;

  hal::degrees spin_offset = 0;
  hal::degrees t_spin_offset = 0;

#if KEEP_PUMP
  // ===========================================================================
  // Setup Pump
  // ===========================================================================
  pump_power->level(false);
  valve_open->level(false);
  constexpr auto inflation_time = 4s;
  auto inflation_deadline = clock->uptime();
#endif
  // ===========================================================================
  // Setup Robotic Arm
  // ===========================================================================

#if KEEP_ARM
  // ===========================================================================
  // Setup Robotic Arm
  // ===========================================================================
  hal::print(*console, "Starting RMD DRC v2 initialization...\n");
  // Setup spin servo which uses the RMD-X7
  hal::actuator::rmd_drc_v2 spin_servo(
    *can_transceiver, *can_identifier_filter, *clock, 6.0f, 0x141);

  hal::print(*console, "RMD DRC v2 initialized!\n");

  constexpr hal::actuator::dynamixel_servo_protocol_1::config wrist_config = {
    .servo = hal::actuator::dynamixel_servo::rx,
    .baud_rate = 57600,
    .id = 4,
    .min_angle = 60,
    .max_angle = 240,
    .response_timeout = 200ms
  };
  constexpr hal::actuator::dynamixel_servo_protocol_1::config elbow_config = {
    .servo = hal::actuator::dynamixel_servo::rx,
    .baud_rate = 57600,
    .id = 3,
    .min_angle = 60,
    .max_angle = 240,
  };
  constexpr hal::actuator::dynamixel_servo_protocol_1::config
    shoulder_lead_config = {
      .servo = hal::actuator::dynamixel_servo::rx,
      .baud_rate = 57600,
      .id = 1,
      .min_angle = 60,
      .max_angle = 240,
    };
  constexpr hal::actuator::dynamixel_servo_protocol_1::config
    shoulder_opposite_config = {
      .servo = hal::actuator::dynamixel_servo::rx,
      .baud_rate = 57600,
      .id = 2,
      .min_angle = 60,
      .max_angle = 240,
    };

  hal::print(*console, "Starting Dynamixel Initialization...\n");

  auto shoulder_lead_servo = hal::actuator::dynamixel_servo_protocol_1(
    uart, clock, shoulder_lead_config);
  hal::print(*console, "lead servo initialized\n");

  auto shoulder_support_servo = hal::actuator::dynamixel_servo_protocol_1(
    uart, clock, shoulder_opposite_config);
  hal::print(*console, "support servo initialized\n");

  auto elbow_servo =
    hal::actuator::dynamixel_servo_protocol_1(uart, clock, elbow_config);
  hal::print(*console, "elbow servo initialized\n");

  auto wrist_servo =
    hal::actuator::dynamixel_servo_protocol_1(uart, clock, wrist_config);
  hal::print(*console, "wrist servo initialized\n");

  wrist_servo.torque_limit(95.0f);
  hal::delay(*clock, 10ms);
  wrist_servo.torque_enable(true);
  hal::delay(*clock, 10ms);
  wrist_servo.speed(15.0f);
  hal::delay(*clock, 10ms);
  wrist_servo.led(false);

  elbow_servo.torque_limit(90.0f);
  hal::delay(*clock, 10ms);
  elbow_servo.torque_enable(true);
  hal::delay(*clock, 10ms);
  elbow_servo.speed(10.0f);
  hal::delay(*clock, 10ms);
  elbow_servo.led(false);

  shoulder_lead_servo.torque_limit(95.0f);
  hal::delay(*clock, 10ms);
  shoulder_lead_servo.torque_enable(true);
  hal::delay(*clock, 10ms);
  shoulder_lead_servo.speed(10.0f);
  hal::delay(*clock, 10ms);
  shoulder_lead_servo.led(false);

  shoulder_support_servo.torque_limit(95.0f);
  hal::delay(*clock, 10ms);
  shoulder_support_servo.torque_enable(false);
  hal::delay(*clock, 10ms);
  shoulder_support_servo.speed(10.0f);
  hal::delay(*clock, 10ms);
  shoulder_support_servo.led(false);

  hal::print(*console, "Motors initialized\n");

  do {
    try {
      elbow_servo.queue_position(210.0f);
      hal::delay(*clock, 10ms);
      wrist_servo.queue_position(185.0f);
      hal::delay(*clock, 10ms);
      shoulder_lead_servo.queue_position(150.0f);
      hal::delay(*clock, 10ms);
      shoulder_support_servo.queue_position(150.0f);
      hal::delay(*clock, 10ms);
      hal::actuator::dynamixel_servo_protocol_1::broadcast_execute_action(uart);
      hal::print(*console, "Servos in postion for 4 seconds\n");
      hal::delay(*clock, 4s);
      break;
    } catch (hal::timed_out const&) {
      hal::print<32>(*console, "❌  %s\n", timeout);
    } catch (hal::io_error const&) {
      hal::print<32>(*console, "❌  %s\n", io_error);
    } catch (...) {
      hal::print<16>(*console, "%s\n", unkown);
    }
  } while (true);
#endif

#if KEEP_MIMIC
  // ===========================================================================
  // Setup Mimic Controller
  // ===========================================================================
  auto i2c_mux = hal::expander::tca9548a(*i2c);
  std::bitset<8> init_ports{ 0x00 };
  init_ports.set(0);
  i2c_mux.set_ports(init_ports);
  hal::sensor::as5600 hall_sensor(i2c);

  spin_offset = hall_sensor.raw_angle();

  init_ports.set(0, false);
  init_ports.set(4);
  i2c_mux.set_ports(init_ports);
  t_spin_offset = hall_sensor.raw_angle();

#endif

  hal::degrees prev_spin = 0;
  hal::degrees prev_t_spin = 0;
  int8_t rotations = 0;
  int8_t t_rotations = 0;

  hal::print(*console, CLEAR_SCREEN);

  while (true) {
    // If you see flickering, change CLEAR_SCREEN to CURSOR_HOME
    hal::print(*console, CURSOR_HOME);
    status_led->level(pump_button.level());

#if KEEP_MIMIC
    using namespace std::literals;
    using namespace hal;

    bool const mimic_controls = control_switch->level();
    // =========================================================================
    // Measure Mimic
    // =========================================================================
    size_t port_start;
    if (mimic_controls) {
      port_start = 0;
    } else {
      port_start = 4;
    }
    std::array<hal::degrees, 8> sensors_angles;
    for (size_t i = port_start; i < (port_start + 4); i++) {
      std::bitset<8> ports{ 0x00 };
      ports.set(i);
      i2c_mux.set_ports(ports);

      // TODO(#15): Determine shorter delay frequency for i2c mux. The mux
      // should be able to switch at a much faster rate than 10ms.
      hal::delay(*clock, 10ms);

      auto magnet_status = hall_sensor.magnet_status();
      if (magnet_status.detected) {
        sensors_angles[i] = hall_sensor.raw_angle();
      }
    }
    // =========================================================================
    // Pass angles to mimic
    // =========================================================================
    [[maybe_unused]] hal::degrees shoulder_angle;
    [[maybe_unused]] hal::degrees elbow_angle;
    [[maybe_unused]] hal::degrees wrist_angle;
    hal::degrees spin;

    if (mimic_controls) {

      shoulder_angle =
        process_angle(sensors_angles[angle_select::shoulder_angle], 150.0f);

      elbow_angle =
        process_angle(sensors_angles[angle_select::elbow_angle], 150.0f);

      wrist_angle =
        process_angle(sensors_angles[angle_select::wrist_angle], 150.0f);

      if (prev_spin > 315 && sensors_angles[angle_select::spin] < 45) {
        rotations--;
      }
      if (prev_spin < 45 && sensors_angles[angle_select::spin] > 315) {
        rotations++;
      }

      spin =
        (360 * rotations) - (sensors_angles[angle_select::spin] - spin_offset);

      prev_spin = sensors_angles[angle_select::spin];
    } else {
      shoulder_angle = process_throttle_angle(
        sensors_angles[angle_select::t_shoulder], 150.0f);

      elbow_angle =
        process_throttle_angle(sensors_angles[angle_select::t_elbow], 150.0f);

      wrist_angle =
        process_throttle_angle(sensors_angles[angle_select::t_wrist], 150.0f);

      if (prev_t_spin > 315 && sensors_angles[angle_select::t_spin] < 45) {
        t_rotations--;
      }
      if (prev_t_spin < 45 && sensors_angles[angle_select::t_spin] > 315) {
        t_rotations++;
      }
      spin = (360 * t_rotations) -
             (sensors_angles[angle_select::t_spin] - t_spin_offset);

      prev_t_spin = sensors_angles[angle_select::t_spin];
    }
    shoulder_angle = std::clamp(shoulder_angle, 100.0f, 360.0f);
#endif

#if KEEP_ARM
    spin_servo.position_control(spin, 7.5f);
    int servo_step = 0;

    try {
      hal::delay(*clock, 10ms);
      elbow_servo.queue_position(elbow_angle);
      hal::delay(*clock, 10ms);
      servo_step++;

      e_status = success;
    } catch (hal::timed_out const&) {
      e_status = timeout;
    } catch (hal::io_error const&) {
      e_status = io_error;
    } catch (...) {
      e_status = unkown;
    }

    try {
      wrist_servo.queue_position(wrist_angle);
      hal::delay(*clock, 10ms);
      servo_step++;
      w_status = success;
    } catch (hal::timed_out const&) {
      w_status = timeout;
    } catch (hal::io_error const&) {
      w_status = io_error;
    } catch (...) {
      w_status = unkown;
    }

    try {
      shoulder_lead_servo.queue_position(shoulder_angle);
      hal::delay(*clock, 10ms);

      if (shoulder_lead_servo.last_error_code() & (1U << 5U)) {
        shoulder_lead_servo.torque_enable(false);
        hal::delay(*clock, 10ms);
        shoulder_lead_servo.torque_limit(100.0f);
        hal::delay(*clock, 10ms);
        shoulder_lead_servo.torque_enable(true);
        hal::delay(*clock, 10ms);
        shoulder_lead_servo.queue_position(shoulder_angle);
      }
      servo_step++;
      s_status = success;
    } catch (hal::timed_out const&) {
      s_status = timeout;
    } catch (hal::io_error const&) {
      s_status = io_error;
    } catch (...) {
      s_status = unkown;
    }

    try {
      auto const reverse_angle = 300 - shoulder_angle;
      shoulder_support_servo.queue_position(reverse_angle);
      hal::delay(*clock, 10ms);

      if (shoulder_support_servo.last_error_code() & (1U << 5U)) {
        shoulder_support_servo.torque_enable(false);
        hal::delay(*clock, 10ms);
        shoulder_support_servo.torque_limit(100.0f);
        hal::delay(*clock, 10ms);
        shoulder_support_servo.torque_enable(true);
        hal::delay(*clock, 10ms);
        shoulder_support_servo.queue_position(reverse_angle);
      }
      hal::delay(*clock, 10ms);
      servo_step++;

      o_status = success;

    } catch (hal::timed_out const&) {
      o_status = timeout;
    } catch (hal::io_error const&) {
      o_status = io_error;
    } catch (...) {
      o_status = unkown;
    }

    try {
      if (servo_step > 0) {
        actuator::dynamixel_servo_protocol_1::broadcast_execute_action(uart);
        hal::delay(*clock, 10ms);
        b_status = success;
      } else {
        b_status = "X";
      }
    } catch (hal::timed_out const&) {
      b_status = timeout;
    } catch (hal::io_error const&) {
      b_status = io_error;
    } catch (...) {
      b_status = unkown;
    }
#endif

#if KEEP_PUMP
    // =========================================================================
    // Handle Pump
    // =========================================================================
    if (pump_button.level()) {
      inflation_deadline = hal::future_deadline(*clock, inflation_time);
      pump_power->level(true);
      valve_open->level(false);
      current_pump_mode = pump_state::pumping;
    } else {
      pump_power->level(false);
      if (inflation_deadline > clock->uptime()) {
        valve_open->level(true);
        current_pump_mode = pump_state::atmosphere;
      } else {
        valve_open->level(false);
        current_pump_mode = pump_state::idle;
      }
    }
#endif

    // 2. Print Mimic Section
    hal::print(*console, "========================================\n");
    hal::print(*console, "    OpenSauce2026 Robot Arm DASHBOARD   \n");
    hal::print(*console, "========================================\n");

    // 3. Unified Joint Display (Works whether ARM is kept or not)
    // We use the macro to decide if we show [emoji][binary] or nothing
    hal::print<64>(*console,
                   " Elbow: %5d° [0x%02X] %s \n",
                   static_cast<int>(elbow_angle),
                   ARM_INFO(elbow_servo.last_error_code()),
                   e_status);
    hal::print<64>(*console,
                   " Wrist: %5d° [0x%02X] %s \n",
                   static_cast<int>(wrist_angle),
                   ARM_INFO(wrist_servo.last_error_code()),
                   w_status);
    hal::print<64>(*console,
                   " Lead:  %5d° [0x%02X] %s \n",
                   static_cast<int>(shoulder_angle),
                   ARM_INFO(shoulder_lead_servo.last_error_code()),
                   s_status);
    hal::print<64>(*console,
                   " Supp:  %5d° [0x%02X] %s \n",
                   static_cast<int>(300 - shoulder_angle),
                   ARM_INFO(shoulder_support_servo.last_error_code()),
                   o_status);
    hal::print<64>(*console,
                   " Spin:  %5d° [0x%02X] %s \n",
                   static_cast<int>(spin),
                   0,
                   success);

    // 4. Pump Dashboard Section
    hal::print(*console, "----------------------------------------\n");
    hal::print(*console, " PUMP: ");
    switch (current_pump_mode) {
      case pump_state::pumping:
        hal::print(*console, "🫳 [PUMPING]");
        break;
      case pump_state::atmosphere:
        hal::print(*console, "🌬️ [VENTING] ");
        break;
      case pump_state::idle:
        hal::print(*console, "💤 [IDLE]    ");
        break;
    }

    hal::print(*console, "\n========================================\n");
  }
}

extern "C"
{
  [[gnu::noreturn]] void _exit(int p_status)
  {
    [[maybe_unused]] int volatile status = p_status;
    while (true) {
      continue;
    }
  }
}
