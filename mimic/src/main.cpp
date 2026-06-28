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
#include <utility>

#include <libhal-actuator/mx_64.hpp>
#include <libhal-actuator/rx_64.hpp>
#include <libhal-actuator/smart_servo/rmd/drc_v2.hpp>
#include <libhal-expander/tca9548a.hpp>
#include <libhal-sensor/as5600.hpp>
#include <libhal-util/map.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/input_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/units.hpp>

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
  hal::print(*console, "Mimic Application Starting...\n");

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

  hal::actuator::rx_64::config wrist_config = {
    .baud_rate = 57600, .id = 3, .min_angle = 60, .max_angle = 240
  };
  hal::actuator::mx_64::config elbow_config = {
    .baud_rate = 57600, .id = 0, .min_angle = 90, .max_angle = 270
  };
  // hal::actuator::rx_64::config shoulder_rotate_config = {
  //   .baud_rate = 57600, .id = 4, .min_angle = 0, .max_angle = 300
  // };
  hal::actuator::rx_64::config shoulder_lead_config = {
    .baud_rate = 57600, .id = 1, .min_angle = 60, .max_angle = 240
  };
  hal::actuator::rx_64::config shoulder_opose_config = {
    .baud_rate = 57600, .id = 2, .min_angle = 60, .max_angle = 240
  };

  auto wrist_servo = hal::actuator::rx_64(uart, wrist_config, clock);
  auto elbow_servo = hal::actuator::mx_64(uart, elbow_config, clock);
  auto shoulder_lead_servo =
    hal::actuator::rx_64(uart, shoulder_lead_config, clock);
  auto shoulder_opose_servo =
    hal::actuator::rx_64(uart, shoulder_opose_config, clock);

  wrist_servo.torque_limit(100.0f);
  hal::delay(*clock, 10ms);
  wrist_servo.speed(10.0f);
  hal::delay(*clock, 10ms);
  elbow_servo.torque_limit(100.0f);
  hal::delay(*clock, 10ms);
  elbow_servo.speed(10.0f);
  hal::delay(*clock, 10ms);
  shoulder_lead_servo.torque_limit(100.0f);
  hal::delay(*clock, 10ms);
  shoulder_lead_servo.speed(10.0f);
  hal::delay(*clock, 10ms);
  // shoulder_opose_servo.torque_limit(80.0f);
  // hal::delay(*clock, 10ms);
  // shoulder_opose_servo.speed(10.0f);
  // hal::delay(*clock, 10ms);

  wrist_servo.led(false);
  elbow_servo.led(false);
  shoulder_lead_servo.led(false);
  shoulder_opose_servo.led(false);

  hal::print(*console, "Motors initialized\n");
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
#endif

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

      spin = sensors_angles[angle_select::spin];

    } else {
      shoulder_angle = process_throttle_angle(
        sensors_angles[angle_select::t_shoulder], 150.0f);

      elbow_angle =
        process_throttle_angle(sensors_angles[angle_select::t_elbow], 180.0f);

      wrist_angle =
        process_throttle_angle(sensors_angles[angle_select::t_wrist], 150.0f);

      spin = sensors_angles[angle_select::t_spin];
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
    spin_servo.position_control(spin, 10.0f);
    wrist_servo.position(wrist_angle);
    elbow_servo.position(elbow_angle);
    shoulder_lead_servo.position(shoulder_angle);
    hal::print<64>(*console,
                   "[W: %" PRIu8 " S: %" PRIu8 "]",
                   wrist_servo.last_error_code(),
                   shoulder_lead_servo.last_error_code());
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
