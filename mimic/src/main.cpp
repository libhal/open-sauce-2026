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
  auto const full_range =
    std::make_pair(p_servo_center - 90.0f, p_servo_center + 90.0f);
  auto const reduced_range =
    std::make_pair(p_servo_center - 10.0f, p_servo_center + 90.0f);

  p_angle = std::clamp(p_angle, 90.0f, 270.0f);
  hal::degrees angle_diff = sensor_center - p_angle;
  hal::degrees raw_angle = p_servo_center + angle_diff;

  return hal::map(raw_angle, full_range, reduced_range);
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

int main()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  initialize_platform();

  auto const clock = resources::clock();
  auto const console = resources::console();
  hal::print(*console, "Mimic Application Starting... 1\n");

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
  hal::print(*console, "All resources acquired\n");

  // Needs to be set to this baud rate to work with the default firmware CAN
  // baud rate.
  can_bus_manager->baud_rate(1.0_MHz);
  hal::print(*console, "Set can baud rate to 1 MHz\n");

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
  // Setup spin servo which uses the RMD-X7
  hal::actuator::rmd_drc_v2 spin_servo(
    *can_transceiver, *can_identifier_filter, *clock, 6.0f, 0x141);

  hal::print(*console, "RMD DRC v2 initialized\n");

  constexpr hal::actuator::dynamixel_servo_protocol_1::config wrist_config = {
    .servo = hal::actuator::dynamixel_servo::rx,
    .baud_rate = 57600,
    .id = 3,
    .min_angle = 60,
    .max_angle = 240,
    .response_timeout = 200ms
  };
  constexpr hal::actuator::dynamixel_servo_protocol_1::config elbow_config = {
    .servo = hal::actuator::dynamixel_servo::mx,
    .baud_rate = 57600,
    .id = 0,
    .min_angle = 90,
    .max_angle = 270,
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

  hal::print(*console, "Starting Dynamixel initialization\n");

  auto wrist_servo =
    hal::actuator::dynamixel_servo_protocol_1(uart, clock, wrist_config);
  hal::print(*console, "wrist servo initialized\n");

  auto elbow_servo =
    hal::actuator::dynamixel_servo_protocol_1(uart, clock, elbow_config);
  hal::print(*console, "elbow servo initialized\n");

  auto shoulder_lead_servo = hal::actuator::dynamixel_servo_protocol_1(
    uart, clock, shoulder_lead_config);
  hal::print(*console, "lead servo initialized\n");

  auto shoulder_support_servo = hal::actuator::dynamixel_servo_protocol_1(
    uart, clock, shoulder_opposite_config);
  hal::print(*console, "support servo initialized\n");

  wrist_servo.torque_limit(95.0f);
  hal::delay(*clock, 10ms);
  wrist_servo.torque_enable(true);
  hal::delay(*clock, 10ms);
  wrist_servo.speed(15.0f);
  hal::delay(*clock, 10ms);

  elbow_servo.torque_limit(90.0f);
  hal::delay(*clock, 10ms);
  elbow_servo.torque_enable(true);
  hal::delay(*clock, 10ms);
  elbow_servo.speed(10.0f);
  hal::delay(*clock, 10ms);

  shoulder_lead_servo.torque_limit(95.0f);
  hal::delay(*clock, 10ms);
  shoulder_lead_servo.torque_enable(true);
  hal::delay(*clock, 10ms);
  shoulder_lead_servo.speed(10.0f);
  hal::delay(*clock, 10ms);

  // shoulder_support_servo.torque_limit(90.0f);
  // hal::delay(*clock, 10ms);
  // shoulder_support_servo.torque_enable(false);
  // hal::delay(*clock, 10ms);
  // shoulder_support_servo.speed(30.0f);
  // hal::delay(*clock, 10ms);

  wrist_servo.led(false);
  elbow_servo.led(false);
  shoulder_lead_servo.led(false);
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
      hal::print(*console, "❌  ⏰\n");
    } catch (hal::io_error const&) {
      hal::print(*console, "❌  📡\n");
    } catch (...) {
      hal::print(*console, "⁉️\n");
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
  int8_t rotations = 0;
  int servo_step = 0;

  while (true) {
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
        process_angle(sensors_angles[angle_select::elbow_angle], 180.0f);

      wrist_angle =
        process_angle(sensors_angles[angle_select::wrist_angle], 150.0f);

      if (prev_spin > 350 && sensors_angles[angle_select::spin] < 10) {
        rotations++;
      }
      if (prev_spin < 10 && sensors_angles[angle_select::spin] > 350) {
        rotations--;
      }
      spin =
        sensors_angles[angle_select::spin] - spin_offset + (360 * rotations);
      prev_spin = sensors_angles[angle_select::spin];
    } else {
      shoulder_angle = process_throttle_angle(
        sensors_angles[angle_select::t_shoulder], 150.0f);

      elbow_angle =
        process_throttle_angle(sensors_angles[angle_select::t_elbow], 180.0f);

      wrist_angle =
        process_throttle_angle(sensors_angles[angle_select::t_wrist], 150.0f);

      if (prev_spin > 345 && sensors_angles[angle_select::t_spin] < 15) {
        rotations++;
      }
      if (prev_spin < 15 && sensors_angles[angle_select::t_spin] > 345) {
        rotations--;
      }
      spin = ((360 * rotations) + sensors_angles[angle_select::t_spin]) -
             t_spin_offset;

      prev_spin = sensors_angles[angle_select::t_spin];
    }
    shoulder_angle = std::clamp(shoulder_angle, 120.0f, 360.0f);
    hal::print(*console, "Mimic: [");
    hal::print<64>(*console, "Spin: %.2f  ", spin);
    hal::print<64>(*console, "Shoulder: %.2f    ", shoulder_angle);
    hal::print<64>(*console, "Elbow: %.2f    ", elbow_angle);
    hal::print<64>(*console, "Wrist: %.2f ", wrist_angle);
    hal::print(*console, "]");
#endif

#if KEEP_ARM
    spin_servo.position_control(spin, 7.5f);

    hal::print(*console, "\n");
    try {

      elbow_servo.queue_position(elbow_angle);
      hal::delay(*clock, 50ms);
      servo_step = 1;

      hal::print(*console, "E: ✅ - ");
    } catch (hal::timed_out const&) {
      hal::print(*console, "E: ⏰ - ");
    } catch (hal::io_error const&) {
      hal::print(*console, "E: 📡 - ");
    } catch (...) {
      hal::print(*console, "E: ⁉️ - ");
    }
    try {
      wrist_servo.queue_position(wrist_angle);
      hal::delay(*clock, 50ms);
      servo_step = 2;
      hal::print(*console, "W: ✅ - ");
    } catch (hal::timed_out const&) {
      hal::print(*console, "W: ⏰ - ");
    } catch (hal::io_error const&) {
      hal::print(*console, "W: 📡 - ");
    } catch (...) {
      hal::print(*console, "W: ⁉️ - ");
    }

    // try {
    //   shoulder_lead_servo.queue_position(shoulder_angle);
    //   hal::delay(*clock, 50ms);

    //   if (shoulder_lead_servo.last_error_code() & (1U << 5U)) {
    //     shoulder_lead_servo.torque_enable(false);
    //     hal::delay(*clock, 50ms);
    //     shoulder_lead_servo.torque_limit(100.0f);
    //     hal::delay(*clock, 50ms);
    //     shoulder_lead_servo.torque_enable(true);
    //     hal::delay(*clock, 50ms);
    //     shoulder_lead_servo.queue_position(shoulder_angle);
    //   }
    //   servo_step = 3;
    //   hal::print(*console, "S: ✅ - ");
    // } catch (hal::timed_out const&) {
    //   hal::print(*console, "S: ⏰ - ");
    // } catch (hal::io_error const&) {
    //   hal::print(*console, "S: 📡 - ");
    // } catch (...) {
    //   hal::print(*console, "S: ⁉️ - ");
    // }

    // try {
    //   auto const reverse_angle = 300 - shoulder_angle;
    //   shoulder_support_servo.queue_position(reverse_angle);
    //   hal::delay(*clock, 50ms);

    //   if (shoulder_support_servo.last_error_code() & (1U << 5U)) {
    //     shoulder_support_servo.torque_enable(false);
    //     hal::delay(*clock, 50ms);
    //     shoulder_support_servo.torque_limit(100.0f);
    //     hal::delay(*clock, 50ms);
    //     shoulder_support_servo.torque_enable(true);
    //     hal::delay(*clock, 50ms);
    //     shoulder_support_servo.queue_position(reverse_angle);
    //   }
    //   hal::delay(*clock, 50ms);
    //   servo_step = 4;

    //   hal::print(*console, "O: ✅ - ");

    // } catch (hal::timed_out const&) {
    //   hal::print(*console, "O: ⏰ - ");
    // } catch (hal::io_error const&) {
    //   hal::print(*console, "O: 📡 - ");
    // } catch (...) {
    //   hal::print(*console, "O: ⁉️ - ");
    // }

    try {
      actuator::dynamixel_servo_protocol_1::broadcast_execute_action(uart);
      hal::delay(*clock, 100ms);
      hal::print(*console, "B: ✅ - ");
    } catch (hal::timed_out const&) {
      hal::print(*console, "B: ⏰ - ");
    } catch (hal::io_error const&) {
      hal::print(*console, "B: 📡 - ");
    } catch (...) {
      hal::print(*console, "B: ⁉️ - ");
    }

    hal::print<128>(*console,
                    "\n[W: %" PRIu8 " E: %" PRIu8 " SL: %" PRIu8 " SS: %" PRIu8
                    "]",
                    wrist_servo.last_error_code(),
                    elbow_servo.last_error_code(),
                    shoulder_lead_servo.last_error_code(),
                    shoulder_support_servo.last_error_code());
#endif

#if KEEP_PUMP
    // =========================================================================
    // Handle Pump
    // =========================================================================
    if (pump_button.level()) {
      inflation_deadline = hal::future_deadline(*clock, inflation_time);
      pump_power->level(true);
      valve_open->level(false);
    } else {
      pump_power->level(false);
      if (inflation_deadline > clock->uptime()) {
        valve_open->level(true);
      } else {
        valve_open->level(false);
      }
    }

#endif
    hal::print(*console, "\n");
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
